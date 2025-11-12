import numpy as np
import sounddevice as sd
from collections import deque
from openwakeword.model import Model
import webrtcvad

# === CONFIGURACIÓN ===
DEVICE_INDEX = 6          # Cambia si tu micro está en otro índice
SAMPLE_RATE = 16000       # OpenWakeWord requiere 16 kHz
BLOCKSIZE = 160           # 10 ms @ 16 kHz
BUFFER_SECONDS = 2.0      # Ventana de audio para procesar
THRESHOLD = 0.01          # Sensibilidad (0.1–0.25 recomendable)

WAKEWORD = "alexa"        # nombre EXACTO del modelo

# ✅ Cargar modelo Timer usando ONNX
model = Model(
    wakeword_models=["alexa"],
    inference_framework="onnx"
)

# ✅ Activación por voz (Voice Activity Detection)
vad = webrtcvad.Vad(2)  # 0=menos sensible, 3=muy sensible

buffer = deque(maxlen=int(SAMPLE_RATE * BUFFER_SECONDS))

print("🎙️ Escuchando... Di 'Alexa' (con modelo oficial)\n")

def callback(indata, frames, time, status):
    global buffer

    audio16 = (indata[:, 0] * 32768).astype(np.int16).tobytes()

    # Guardamos audio SOLO cuando hay voz
    if vad.is_speech(audio16, SAMPLE_RATE):
        buffer.extend(indata[:, 0])

    # Cuando acumulamos 1 seg → procesamos
    if len(buffer) >= SAMPLE_RATE:
        samples = np.array(buffer, dtype=np.float32)
        buffer.clear()

        # Normalizamos
        max_val = np.max(np.abs(samples))
        if max_val > 0:
            samples = samples / max_val

        # 🔍 Ejecutamos modelo
        scores = model.predict_clip(samples)
        score = max(frame[WAKEWORD] for frame in scores)

        print(f"score={score:.4f}")

        if score > THRESHOLD:
            print(f"🚀 WAKEWORD DETECTADA: '{WAKEWORD.upper()}' (score={score:.3f})")


with sd.InputStream(
    samplerate=SAMPLE_RATE,
    channels=1,
    dtype="float32",
    blocksize=BLOCKSIZE,
    device=DEVICE_INDEX,
    callback=callback
):
    while True:
        sd.sleep(100)
