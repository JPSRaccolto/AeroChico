# -*- coding: utf-8 -*-
import os
# Evitar erro do Qt quando rodando sem display (SSH headless)
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import cv2
import numpy as np
import serial
import time
import threading
from pathlib import Path
from picamera2 import Picamera2
from threading import Event, Lock
from collections import deque

# -------------------------------
# Configurações
# -------------------------------
PORTA = '/dev/ttyACM0'
BAUD = 115200

# Caminho do microSD montado via USB
MICROSD_PATH = Path('/media/aerochico/AERO')
ARQUIVO_DADOS = MICROSD_PATH / 'dados_planador.txt'
ARQUIVO_VIDEO = MICROSD_PATH / 'flight_overlay_video.mp4'

# Configurações de vídeo
VIDEO_WIDTH, VIDEO_HEIGHT = 640, 480
FPS = 30 # Taxa de quadros de 30 FPS // Diminuir o FPS a medida que aumenta a qualidade do video

# Mostrar preview do vídeo durante gravação?
MOSTRAR_PREVIEW = True

# Cabeçalho e unidades
CAMPOS = ["Tempo", "XGPS", "YGPS", "ZGPS", "Theta", "Phi"]
UNIDADES = ["Segundos", "m", "m", "m", "deg", "deg"]

# Tempo mínimo antes de permitir STOP
MIN_TIME_STOP = 45 # Mantido em 45 segundos

# Buffer para gravação em disco (tempo em ms)
WRITE_INTERVAL_MS = 100  # Grava a cada 100ms

# Timeout aguardando decolagem (em segundos)
TIMEOUT_AGUARDA_ZGPS = 120


class FlightDataBuffer:
    """Buffer thread-safe para dados de voo."""
    def __init__(self):
        self.lock = Lock()
        self.buffer = deque()
        self.total_written = 0

    def add(self, linha):
        """Adiciona linha ao buffer."""
        with self.lock:
            self.buffer.append(linha)

    def flush(self):
        """Retorna todas as linhas e limpa buffer."""
        with self.lock:
            dados = list(self.buffer)
            self.buffer.clear()
            return dados

    def size(self):
        """Retorna tamanho atual do buffer."""
        with self.lock:
            return len(self.buffer)

    def get_total_written(self):
        """Retorna total de amostras escritas."""
        with self.lock:
            return self.total_written

    def increment_written(self, count):
        """Incrementa contador de escritas."""
        with self.lock:
            self.total_written += count

    def peek_last(self):
        """Retorna a última linha do buffer sem removê-la (ou None)."""
        with self.lock:
            if len(self.buffer) == 0:
                return None
            return self.buffer[-1]


class HUDData:
    """Dados HUD thread-safe com sincronização de timestamp."""
    def __init__(self):
        self.lock = Lock()
        self.data = None
        self.pico_time = None
        self.pi_time = None
        self.time_offset = 0

    def update(self, hud_dict):
        """Atualiza dados HUD e sincroniza tempo."""
        with self.lock:
            self.data = hud_dict.copy() if hud_dict else None
            if hud_dict and 'time' in hud_dict:
                # Extrair timestamp do Pico (formato: MM:SS.ss ou HH:MM:SS)
                try:
                    tempo_str = hud_dict['time']
                    # Tentar suportar vários formatos; aqui simplificamos:
                    partes = tempo_str.split(':')
                    if len(partes) == 3:
                        # assumir HH:MM:SS
                        h, m, s = float(partes[0]), float(partes[1]), float(partes[2])
                        self.pico_time = h * 3600 + m * 60 + s
                        self.pi_time = time.time()
                except Exception:
                    pass

    def get(self):
        """Retorna cópia dos dados HUD atuais."""
        with self.lock:
            return self.data.copy() if self.data else None

    def get_time_offset(self):
        """Retorna o offset de sincronização de tempo."""
        with self.lock:
            if self.pico_time is not None and self.pi_time is not None:
                return self.pi_time - self.pico_time
            return 0


def gravar_cabecalho():
    """Grava o cabeçalho no arquivo se ele ainda não existir."""
    if not ARQUIVO_DADOS.exists():
        if not MICROSD_PATH.exists():
            print(f"[ERRO] MicroSD não encontrado em {MICROSD_PATH}")
            return False
        with open(ARQUIVO_DADOS, 'w', encoding='utf-8') as f:
            f.write('\t'.join(CAMPOS) + '\n')
            f.write('\t'.join(UNIDADES) + '\n')
        print(f"✔ Cabeçalho gravado em {ARQUIVO_DADOS}")
    return True


def thread_leitura_serial(ser, data_buffer, hud_data, stop_event, stats):
    """Thread dedicada para leitura serial sem bloqueio."""
    print("[THREAD SERIAL] Iniciada")

    try:
        while not stop_event.is_set():
            try:
                if ser.in_waiting > 0:
                    linha = ser.readline().decode('utf-8', errors='ignore').strip()

                    # Dados DATA - salvar imediatamente COM TIMESTAMP DO PICO
                    if linha.startswith("DATA"):
                        dados = linha.replace("DATA,", "")
                        valores = dados.split(',')

                        if len(valores) == 6:
                            # Usar tempo do Pico, não do Pi
                            tempo, xgps, ygps, zgps, theta, phi = valores
                            # Validar que é número
                            try:
                                float(tempo)
                                linha_formatada = f"{tempo}\t{xgps}\t{ygps}\t{zgps}\t{theta}\t{phi}"
                                data_buffer.add(linha_formatada)
                                stats['data_recebidas'] += 1
                            except ValueError:
                                pass

                    # Dados HUD - atualizar overlay
                    elif linha.startswith("HUD"):
                        partes = linha.split('|')
                        # esperado: HUD|time|alt|vel|gz|status
                        if len(partes) >= 6:
                            novo_hud = {
                                'time': partes[1],
                                'altitude': partes[2],
                                'velocity': partes[3],
                                'g_z': partes[4],
                                'status': partes[5]
                            }
                            hud_data.update(novo_hud)
                            stats['hud_recebidas'] += 1

                    # Comando STOP
                    elif linha == "STOP":
                        stats['stop_recebido'] = True

                time.sleep(0.001)

            except Exception as e:
                print(f"[THREAD SERIAL] Erro: {e}")
                time.sleep(0.01)

    except Exception as e:
        print(f"[THREAD SERIAL] Erro crítico: {e}")

    finally:
        print("[THREAD SERIAL] Encerrada")


def thread_escrita_disco(data_buffer, stop_event, stats):
    """Thread dedicada para escrita em disco."""
    print("[THREAD DISCO] Iniciada")

    try:
        while not stop_event.is_set():
            # Aguardar intervalo de escrita
            time.sleep(WRITE_INTERVAL_MS / 1000.0)

            # Recuperar dados do buffer
            dados = data_buffer.flush()

            if dados:
                try:
                    with open(ARQUIVO_DADOS, 'a', encoding='utf-8') as f:
                        f.write('\n'.join(dados) + '\n')

                    data_buffer.increment_written(len(dados))
                    stats['disco_escritas'] += 1
                    stats['ultimo_write'] = len(dados)

                except Exception as e:
                    print(f"[THREAD DISCO] Erro ao gravar: {e}")
                    # Re-adicionar dados ao buffer em caso de erro
                    for linha in dados:
                        data_buffer.add(linha)

        # FLUSH FINAL - gravar qualquer dado residual
        dados_final = data_buffer.flush()
        if dados_final:
            try:
                with open(ARQUIVO_DADOS, 'a', encoding='utf-8') as f:
                    f.write('\n'.join(dados_final) + '\n')
                data_buffer.increment_written(len(dados_final))
                print(f"[THREAD DISCO] Flush final: {len(dados_final)} amostras")
            except Exception as e:
                print(f"[THREAD DISCO] Erro no flush final: {e}")

    except Exception as e:
        print(f"[THREAD DISCO] Erro crítico: {e}")

    finally:
        print("[THREAD DISCO] Encerrada")


def draw_overlay(frame, hud_dict, awaiting_decolagem_text=None):
    """Desenha overlay limpo estilo HUD profissional. Pode receber texto de espera."""
    h, w = frame.shape[:2]

    # Fonte e cores
    font = cv2.FONT_HERSHEY_SIMPLEX
    text_color = (0, 255, 0)  # Verde brilhante
    line_color = (0, 200, 0)  # Verde mais escuro para linhas

    if not hud_dict:
        # Tela de espera (sem dados HUD)
        if awaiting_decolagem_text:
            cv2.putText(frame, awaiting_decolagem_text, (w//2 - 300, h//2 - 20), font, 0.7,
                        (0, 255, 255), 2, cv2.LINE_AA)
        else:
            msg1 = "AGUARDANDO DADOS DO PICO 2W"
            msg2 = "Conecte o dispositivo e aguarde..."
            cv2.putText(frame, msg1, (w//2 - 250, h//2 - 20), font, 0.8,
                        text_color, 2, cv2.LINE_AA)
            cv2.putText(frame, msg2, (w//2 - 200, h//2 + 20), font, 0.6,
                        text_color, 1, cv2.LINE_AA)
        return frame

    # ============ TEMPO - CENTRO SUPERIOR ============
    tempo = hud_dict.get('time', '--:--:--')
    (tw, th), _ = cv2.getTextSize(tempo, font, 0.75, 2)
    cv2.putText(frame, tempo, (w//2 - tw//2, 40), font, 0.75,
                text_color, 2, cv2.LINE_AA)

    # ============ ALTITUDE - ESQUERDA ============
    alt_value = hud_dict.get('altitude', '---')
    alt_x = 20
    alt_y = h//2

    # Label acima
    cv2.putText(frame, "ALT", (alt_x, alt_y - 40), font, 0.35,
                text_color, 1, cv2.LINE_AA)
    # Valor grande
    cv2.putText(frame, alt_value, (alt_x, alt_y), font, 0.9,
                text_color, 2, cv2.LINE_AA)
    # Unidade distante (abaixo)
    cv2.putText(frame, "m", (alt_x + 10, alt_y + 40), font, 0.5,
                text_color, 1, cv2.LINE_AA)

    # ============ VELOCIDADE (CAS) - DIREITA ============
    vel_value = hud_dict.get('velocity', '---')
    vel_x = w - 90
    vel_y = h//2

    # Label acima
    cv2.putText(frame, "CAS", (vel_x, vel_y - 40), font, 0.35,
                text_color, 1, cv2.LINE_AA)
    # Valor grande
    cv2.putText(frame, vel_value, (vel_x, vel_y), font, 0.9,
                text_color, 2, cv2.LINE_AA)
    # Unidade distante (abaixo)
    cv2.putText(frame, "m/s", (vel_x, vel_y + 40), font, 0.45,
                text_color, 1, cv2.LINE_AA)

    # ============ G-FORCE - INFERIOR ESQUERDO ============
    g_value = hud_dict.get('g_z', '---')
    g_x = 20
    g_y = h - 60

    # Label acima
    cv2.putText(frame, "G-Z", (g_x, g_y - 40), font, 0.35,
                text_color, 1, cv2.LINE_AA)
    # Valor grande
    cv2.putText(frame, g_value, (g_x, g_y), font, 0.9,
                text_color, 2, cv2.LINE_AA)
    # Unidade distante (abaixo)
    cv2.putText(frame, "g", (g_x + 10, g_y + 40), font, 0.5,
                text_color, 1, cv2.LINE_AA)

    # ============ STATUS - INFERIOR DIREITO ============
    status = hud_dict.get('status', 'N/A')
    status_x = w - 150
    status_y = h - 25
    cv2.putText(frame, status, (status_x, status_y), font, 0.5,
                text_color, 1, cv2.LINE_AA)

    # ============ LINHAS DE HORIZONTE - CENTRO ============
    center_y = h // 2
    cv2.line(frame, (w//2 - 80, center_y), (w//2 - 30, center_y),
             line_color, 3)
    cv2.line(frame, (w//2 + 30, center_y), (w//2 + 80, center_y),
             line_color, 3)
    cv2.circle(frame, (w//2, center_y), 4, text_color, -1)

    # ============ MARCADORES LATERAIS ============
    pitch_spacing = 40
    for i in range(-2, 3):
        if i == 0:
            continue
        y_pos = center_y + (i * pitch_spacing)
        if 50 < y_pos < h - 50:
            cv2.line(frame, (w//2 - 100, y_pos), (w//2 - 85, y_pos),
                     line_color, 2)
            cv2.line(frame, (w//2 + 85, y_pos), (w//2 + 100, y_pos),
                     line_color, 2)

    return frame


def aguardar_dados_iniciais(ser, picam2, data_buffer, hud_data, stop_event, stats):
    """Aguarda até que o ZGPS > 0 para iniciar gravação."""
    print("\n" + "="*60)
    print("AGUARDANDO CONDIÇÕES DE DECOLAGEM (ZGPS > 0)...")
    print("="*60)

    # temp_frame_count = 0 # Não é mais necessário
    timeout_start = time.time()

    while not stop_event.is_set():
        try:
            # Mostrar preview com overlay (opcional)
            if MOSTRAR_PREVIEW:
                # --------------------------------------------------------------
                # MUDANÇA 1: Capturar *sempre*.
                # Esta chamada agora bloqueia por 1/30s (pois a câmera
                # está em modo vídeo), ditando o ritmo do loop.
                frame = picam2.capture_array()
                # --------------------------------------------------------------

                # Pegar HUD atual se disponível
                hud_atual = hud_data.get()
                # Tentar obter ZGPS mais recente do buffer
                ultima_data = data_buffer.peek_last()
                zgps_val = None
                if ultima_data:
                    partes = ultima_data.split('\t')
                    if len(partes) >= 4:
                        try:
                            zgps_val = float(partes[3])
                        except:
                            zgps_val = None

                texto_aguarda = "AGUARDANDO DECOLAGEM..."
                if zgps_val is not None:
                    texto_aguarda = f"Aguardando decolagem (ZGPS = {zgps_val:.2f} m)"
                elif hud_atual and 'altitude' in hud_atual:
                    # fallback: usar altitude do HUD se contiver valor numérico
                    try:
                        texto_aguarda = f"Aguardando decolagem (ALT HUD = {float(hud_atual['altitude']):.2f} m)"
                    except:
                        pass

                frame_display = draw_overlay(frame, hud_atual, awaiting_decolagem_text=texto_aguarda)
                cv2.imshow("Flight Data Recorder - Aguardando ZGPS", frame_display)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Cancelado pelo usuário (q pressionado)")
                    return False
            
            # temp_frame_count += 1 # Não é mais necessário

            # Checar se recebeu dados DATA suficientes para inspecionar ZGPS
            if stats['data_recebidas'] > 0:
                ultima = data_buffer.peek_last()
                if ultima:
                    partes = ultima.split('\t')
                    if len(partes) >= 4:
                        try:
                            zgps = float(partes[3])
                            if zgps > 0:
                                print(f"\n✔ Condição atendida: ZGPS = {zgps:.2f} m > 0")
                                return True
                        except ValueError:
                            pass

            # Timeout opcional
            if time.time() - timeout_start > TIMEOUT_AGUARDA_ZGPS:
                print("\n✖ Timeout: ZGPS não ultrapassou 0 dentro do tempo limite.")
                return False

            # --------------------------------------------------------------
            # MUDANÇA 2: Remover o sleep manual.
            # O ritmo é dado pelo bloqueio de picam2.capture_array()
            # time.sleep(0.05) # REMOVIDO
            # --------------------------------------------------------------

        except Exception as e:
            print(f"Erro aguardando ZGPS: {e}")
            time.sleep(0.1) # Manter sleep apenas em caso de erro

    return False


def main():
    """Função principal com threads de leitura e escrita."""

    # Verificar e gravar cabeçalho
    if not gravar_cabecalho():
        return

    # Inicializar câmera
    print("Inicializando câmera...")
    picam2 = Picamera2()
    
    # ------------------------------------------------------------------
    # MUDANÇA 3: Usar create_video_configuration e setar FrameRate
    # Isso força a CÂMERA a gerar frames na taxa exata do VÍDEO.
    config = picam2.create_video_configuration(
        main={"size": (VIDEO_WIDTH, VIDEO_HEIGHT), "format": "RGB888"},
        controls={"FrameRate": FPS}
    )
    # ------------------------------------------------------------------
    
    picam2.configure(config)
    picam2.start()
    time.sleep(2)
    print(f"✔ Câmera inicializada (Modo Vídeo @ {FPS} FPS)")

    # Estruturas thread-safe
    data_buffer = FlightDataBuffer()
    hud_data = HUDData()
    stop_event = Event()

    # Estatísticas
    stats = {
        'data_recebidas': 0,
        'hud_recebidas': 0,
        'stop_recebido': False,
        'disco_escritas': 0,
        'ultimo_write': 0
    }

    try:
        with serial.Serial(PORTA, BAUD, timeout=0.1) as ser:
            print(f"✔ Conectado ao Pico em {PORTA}")

            # Iniciar thread de leitura serial
            thread_serial = threading.Thread(
                target=thread_leitura_serial,
                args=(ser, data_buffer, hud_data, stop_event, stats),
                daemon=True
            )
            thread_serial.start()

            # Aguardar dados iniciais (agora aguarda ZGPS > 0)
            # Esta função foi ajustada para rodar com a câmera em modo vídeo
            if not aguardar_dados_iniciais(ser, picam2, data_buffer, hud_data, stop_event, stats):
                print("\nGravação cancelada (sem decolagem).")
                stop_event.set()
                picam2.stop()
                cv2.destroyAllWindows()
                return

            # Iniciar thread de escrita em disco
            thread_disco = threading.Thread(
                target=thread_escrita_disco,
                args=(data_buffer, stop_event, stats),
                daemon=False
            )
            thread_disco.start()

            # INICIAR GRAVAÇÃO
            print("\n" + "="*50)
            print("INICIANDO GRAVAÇÃO")
            print("="*50)

            fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec MP4
            out = cv2.VideoWriter(
                str(ARQUIVO_VIDEO),
                fourcc,
                FPS,
                (VIDEO_WIDTH, VIDEO_HEIGHT)
            )

            if not out.isOpened():
                print(f"[ERRO] Não foi possível criar arquivo de vídeo {ARQUIVO_VIDEO}")
                stop_event.set()
                picam2.stop()
                return

            print(f"✔ Vídeo: {ARQUIVO_VIDEO}")
            print(f"✔ Dados: {ARQUIVO_DADOS}")
            print(f"✔ Taxa de escrita em disco: {WRITE_INTERVAL_MS}ms")
            print("\nGravando... Pressione Ctrl+C ou 'q' para parar\n")

            start_time = time.time()
            frame_count = 0
            last_stats_time = time.time()

            # ------------------------------------------------------------------
            # MUDANÇA 4: REMOVIDO O CONTROLE DE TEMPO MANUAL (next_frame_time)
            # O loop agora depende do bloqueio de picam2.capture_array()
            # ------------------------------------------------------------------

            while True:
                elapsed = time.time() - start_time

                # Parar após MIN_TIME_STOP E se receber STOP do Pico
                if stats['stop_recebido']:
                    if elapsed >= MIN_TIME_STOP:
                        print(f"\n✔ STOP recebido após {elapsed:.1f}s")
                        break
                    # Se recebeu STOP, continua rodando até atingir o tempo mínimo
                
                # Parar se atingir MIN_TIME_STOP e o STOP NÃO for recebido
                if elapsed >= MIN_TIME_STOP and not stats['stop_recebido']:
                    print(f"\n✔ Tempo mínimo ({MIN_TIME_STOP}s) atingido, parando automaticamente.")
                    break


                # 1. Capturar frame com overlay
                try:
                    # --------------------------------------------------------------
                    # MUDANÇA 5: Esta chamada agora bloqueia por 1/30s
                    frame = picam2.capture_array()
                    # --------------------------------------------------------------
                    
                    hud_atual = hud_data.get()
                    frame_with_overlay = draw_overlay(frame, hud_atual)
                    out.write(frame_with_overlay)
                    frame_count += 1

                    if MOSTRAR_PREVIEW:
                        cv2.imshow("Flight Data Recorder", frame_with_overlay)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            print("\n✔ Tecla 'q' pressionada")
                            break

                except Exception as e:
                    print(f"Erro frame: {e}")

                # Imprimir estatísticas a cada 5s
                if time.time() - last_stats_time > 5:
                    time_offset = hud_data.get_time_offset()
                    print(f"[{elapsed:.1f}s] DATA: {stats['data_recebidas']} | "
                          f"HUD: {stats['hud_recebidas']} | "
                          f"Vídeo: {frame_count} frames | "
                          f"Buffer: {data_buffer.size()} | "
                          f"Offset: {time_offset:.3f}s")
                    last_stats_time = time.time()
                
                # --------------------------------------------------------------
                # MUDANÇA 6: Garantir que não há NENHUM time.sleep()
                # --------------------------------------------------------------


    except KeyboardInterrupt:
        print("\n✔ Interrompido pelo usuário")

    except Exception as e:
        print(f"\n✖ Erro: {e}")

    finally:
        print("\n" + "="*50)
        print("FINALIZANDO")
        print("="*50)

        # Sinalizar threads para parar
        stop_event.set()

        # Aguardar thread de disco finalizar (importante!)
        # O tempo de espera é importante para que o flush final ocorra
        if 'thread_disco' in locals() and thread_disco.is_alive():
            thread_disco.join(timeout=2) # Espera no máximo 2s

        # Fechar recursos
        if 'picam2' in locals():
            picam2.stop()
        if 'out' in locals():
            out.release()
        cv2.destroyAllWindows()

        if 'start_time' in locals():
            elapsed_total = time.time() - start_time
            total_amostras = data_buffer.get_total_written()

            print(f"\n✔ Coleta finalizada:")
            print(f"  - Tempo total: {elapsed_total:.1f}s")
            print(f"  - Amostras DATA salvas: {total_amostras}")
            print(f"  - Frames vídeo: {frame_count}")
            # A taxa média é calculada sobre o total escrito pelo tempo total
            if elapsed_total > 0:
                print(f"  - Taxa média (Dados): {total_amostras/elapsed_total:.1f} Hz (esperado: 50 Hz)")
            print(f"  - Dados: {ARQUIVO_DADOS}")
            print(f"  - Vídeo: {ARQUIVO_VIDEO}")


if __name__ == "__main__":
    main()
