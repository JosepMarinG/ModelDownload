import numpy as np
import sounddevice as sd
from collections import deque
from openwakeword.model import Model

DEVICE_INDEX = 6
SAMPLE_RATE = 16000
BUFFER_SECONDS = 1.0
BLOCKSIZE = 1024
THRESHOLD = 0.05

buffer = deque(maxlen=int(SAMPLE_RATE * BUFFER_SECONDS))

model = Model(wakeword_models=["alexa"], inference_framework="onnx")

print("🎙️ Streaming usando predict_clip() interno...")

def callback(indata, frames, time, status):
    buffer.extend(indata[:, 0])

    if len(buffer) < SAMPLE_RATE:
        return

    samples = np.array(buffer, dtype=np.float32).copy()
    buffer.clear()

    # NORMALIZACIÓN, como hace predict_clip internamente
    max_val = np.max(np.abs(samples))
    if max_val > 0:
        samples = samples / max_val

    # 🔥 ahora usamos predict_clip sobre array (lo toma igual que un wav)
    result = model.predict_clip(samples)

    # predict_clip devuelve lista → cogemos máximo
    score = max(frame["alexa"] for frame in result)
    print(f"score={score:.4f}")

    if score > THRESHOLD:
        print(f"🚀 DETECTADA → ALEXA (score={score:.3f})")

with sd.InputStream(
    samplerate=SAMPLE_RATE, channels=1, dtype="float32",
    blocksize=BLOCKSIZE, device=DEVICE_INDEX, callback=callback
):
    while True:
        sd.sleep(100)
