import numpy as np
import sounddevice as sd
from collections import deque
from openwakeword.model import Model
import webrtcvad

# 🔊 CONFIGURACIÓN
DEVICE_INDEX = 6          # índice del micrófono (ajusta si cambia)
SAMPLE_RATE = 16000
BLOCKSIZE = 160           # 10 ms @ 16 kHz
BUFFER_SECONDS = 1.0
THRESHOLD = 0.25          # ajusta sensibilidad

WAKEWORD_NAME = "Minicernbot"  # exactamente como aparece en model.wakeword_models
MODEL_PATH = "/home/josep/robotics/ModelDownload/wakewords/ModelMiniCernBot/Minicernbot.tflite"

# ✅ Cargar modelo personalizado TFLite
model = Model(
    wakeword_models=[MODEL_PATH],
    inference_framework="tflite"
)

# ✅ Voice Activity Detector
vad = webrtcvad.Vad(2)  # sensibilidad (0–3)
buffer = deque(maxlen=int(SAMPLE_RATE * BUFFER_SECONDS))

print("🎙️ Escuchando... (VAD + MiniCernBot wakeword)")

def callback(indata, frames, time, status):
    global buffer

    # Convertir stream de micrófono a PCM int16 (lo que espera el modelo)
    audio_int16 = (indata[:, 0] * 32767).astype(np.int16)

    # VAD funciona con bytes
    if vad.is_speech(audio_int16.tobytes(), SAMPLE_RATE):
        buffer.extend(audio_int16)

    # Cuando acumulamos 1s de voz -> evaluar wakeword
    if len(buffer) >= SAMPLE_RATE:
        samples = np.array(buffer, dtype=np.int16)
        buffer.clear()

        # 🚀 Ejecutar inferencia
        scores = model.predict_clip(samples)

        score = max(frame[WAKEWORD_NAME] for frame in scores)
        print(f"score={score:.4f}")

        if score > THRESHOLD:
            print(f"✅🚀 DETECTADO → MiniCernBot (score={score:.3f})")


# 🎤 Captura de audio en streaming
with sd.InputStream(
    samplerate=SAMPLE_RATE,
    channels=1,
    dtype="float32",        # sounddevice da float32, luego convertimos
    blocksize=BLOCKSIZE,
    device=DEVICE_INDEX,
    callback=callback
):
    while True:
        sd.sleep(100)
