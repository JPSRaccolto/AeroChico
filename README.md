# Andorinhão

**Andorinhão** é um sistema de monitoramento de voo modular e de código aberto, projetado para registrar dados de sensores e criar um *overlay* (HUD) em vídeo para análise pós-voo de planadores ou sondas atmosféricas. O sistema é dividido em duas unidades principais: a Estação de Monitoramento (**Raspberry Pi Pico 2 W**) e a Estação de Gravação e Visualização (**Raspberry Pi 4 Model B**).

## 🚀 Funcionalidades Principais

| Módulo | Funcionalidade | Implementação |
| :--- | :--- | :--- |
| **Pico (Aquisição)** | Aquisição e Processamento de Sensores | C/C++ com Pico SDK |
| **Pico (Aquisição)** | Determinação de Ângulos de Atitude (`Theta`, `Phi`) | Filtro Complementar (MPU6500) |
| **Pico (Aquisição)** | Medição de Altitude e Velocidade | Altitude Barométrica (BME680) e Velocidade Calibrada (CAS) |
| **Pico (Aquisição)** | Detecção de Estado de Voo | Lógica baseada em Altitude e Velocidade (ATT/DPL/LND) |
| **Pi 4 (Gravação)** | Registro de Dados | Salva dados brutos em `dados_planador.txt` no microSD/USB montado. |
| **Pi 4 (Gravação)** | Visualização em Tempo Real | Gera vídeo `flight_overlay_video.mp4` com overlay HUD (alt, CAS, g-force, status, tempo) usando Picamera2 e OpenCV. |
| **Integração** | Comunicação | Serial (USB/UART) para troca de dados `HUD` e `DATA`. |
| **Integração** | Lógica de Voo | Aguarda condição de decolagem (`ZGPS > 0`) para iniciar a gravação. |

## 📐 Arquitetura do Sistema

O projeto é dividido em dois sub-repositórios de código, correspondentes aos microcontroladores utilizados:

### 1. Estação de Monitoramento (`aero_unificado/`) - Raspberry Pi Pico 2 W

**Hardware:**
* **Microcontrolador:** Raspberry Pi Pico 2 W.
* **IMU:** MPU6500 (Acelerômetro e Giroscópio).
* **Barômetro:** BME680 (Pressão, Temperatura, Umidade, Gás).
* **GPS:** NEO-6M.

**Software:**
* **Linguagem:** C/C++.
* **SDK:** Raspberry Pi Pico SDK.
* **Função:** Coleta dados de todos os sensores, processa variáveis como atitude (Pitch e Roll) e CAS, e envia as informações para o Pi 4 via Serial (USB/UART).

**Possibilidades de Expansão (Módulo Pico):**
* Controle de Servo Motores.
* Comunicação de Longo Alcance (LoRa).

### 2. Estação de Gravação e Visualização (`aero_pi4.py`) - Raspberry Pi 4 Model B

**Hardware:**
* **Computador:** Raspberry Pi 4 Model B.
* **Câmera:** Módulo de Câmera (usando `picamera2`).
* **Armazenamento:** MicroSD/Pen Drive montado em `/media/aerochico/AERO`.

**Software:**
* **Linguagem:** Python.
* **Bibliotecas:** `cv2` (OpenCV), `numpy`, `serial`, `picamera2`.
* **Função:** Gerencia a comunicação Serial, captura *frames* de vídeo, desenha o *overlay* com os dados recebidos do Pico e salva os dados (`dados_planador.txt` e `flight_overlay_video.mp4`) no dispositivo de armazenamento.

## ⚙️ Configuração e Instalação

#### 1. Módulo de Aquisição (Raspberry Pi Pico 2 W)

Este projeto é configurado para ser compilado usando o **Raspberry Pi Pico SDK** e a extensão oficial do VS Code.

1.  **Clone o Repositório:**
    ```bash
    git clone https://github.com/JPSRaccolto/AeroChico.git
    cd AeroChico
    ```
2.  **Instale o VS Code e Extensões:** Instale o [Visual Studio Code](https://code.visualstudio.com/) e as extensões recomendadas (listadas no arquivo `.vscode/extensions.json`), como a `Raspberry Pi Pico` e `C/C++ Extension Pack`.
3.  **Abra o Projeto no VS Code:**
    * No VS Code, vá em `File` > `Open Folder...` e selecione a pasta `aero_unificado`.
    * A extensão Raspberry Pi Pico deve detectar automaticamente o projeto CMake. Se necessário, siga o prompt da extensão para configurar o SDK e a Toolchain.
4.  **Configure e Compile (Build):**
    * Pressione `Ctrl+Shift+P` (ou `Cmd+Shift+P`) e selecione **"CMake: Configure"**. Isso irá criar a pasta `build` e configurar o projeto.
    * Para compilar, pressione `Ctrl+Shift+P` e selecione **"CMake: Build"** (ou use o botão "Build" na barra de status inferior). A compilação será executada usando o `ninja`.
5.  **Upload (Flash):**
    * Coloque o Pico em modo BOOTSEL (segure o botão BOOTSEL e conecte ao PC).
    * No VS Code, use o comando **"Raspberry Pi Pico: Flash"** (ou o comando de `tasks.json` se configurado) para carregar o firmware `aero_unificado.uf2`.

#### 2. Módulo de Gravação (Raspberry Pi 4 Model B)

1.  **Instale Dependências Python:**
    ```bash
    # Instale o OpenCV (usado pelo picamera2 para o overlay)
    sudo apt update
    sudo apt install python3-opencv
    # Instale as bibliotecas Python
    pip install pyserial numpy picamera2
    ```
2.  **Configuração do Armazenamento:** Verifique se o pendrive/microSD está montado no caminho esperado pelo script, conforme definido em `aero_pi4.py`:
    ```python
    MICROSD_PATH = Path('/media/aerochico/AERO')
    ```
3.  **Execução:** Conecte o Raspberry Pi Pico (com o firmware carregado) ao Pi 4 via USB e execute o script Python.
    ```bash
    python3 aero_pi4.py
    ```

## 📝 Formato dos Dados Seriais

O Pico envia dois tipos de *string* para o Pi 4 via USB Serial (em `/dev/ttyACM0`):

1.  **Dados Brutos (`DATA`)** - Para registro em arquivo:
    ```
    DATA,<Tempo>,<XGPS>,<YGPS>,<ZGPS>,<Theta>,<Phi>
    # Ex: DATA,124.5,10.25,5.12,350.00,-1.50,0.50
    ```

2.  **Dados HUD (`HUD`)** - Para visualização em vídeo:
    ```
    HUD|<time>|<altitude>|<velocity>|<g_z>|<status>
    # Ex: HUD|19:03:44|424.7|15.5|1.02|ATT
    ```
    Onde `<status>` pode ser `ATT` (Acoplado), `DPL` (Em Voo) ou `LND` (Em Solo).
