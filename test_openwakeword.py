import numpy as np
import sounddevice as sd
from openwakeword.model import Model
from collections import deque

model = Model(inference_framework="onnx")
print("✅ Modelos cargados:", list(model.models.keys()))

SAMPLE_RATE = 16000
BLOCKSIZE = 1024
BUFFER_SECONDS = 1.0

buffer = deque(maxlen=int(SAMPLE_RATE * BUFFER_SECONDS))
DEVICE_INDEX = 6   # cambia esto si tu micrófono aparece en otro índice

print(f"🎙️ Escuchando (device={DEVICE_INDEX}, 16kHz PCM mono)...")

def callback(indata, frames, time, status):
    samples = indata[:, 0].astype(np.float32)
    buffer.extend(samples)

    if len(buffer) >= SAMPLE_RATE:
        audio_chunk = np.array(buffer)
        buffer.clear()

        score = model.predict(audio_chunk)

        # Mostrar valores en tiempo real (depuración)
        print(score)

        for name, value in score.items():
            if value > 0.25:  # umbral más permisivo
                print(f"🚀 Wake word detectada: {name} ({value:.2f})")

with sd.InputStream(
    samplerate=SAMPLE_RATE,
    channels=1,
    dtype="float32",
    blocksize=BLOCKSIZE,
    callback=callback,
    device=DEVICE_INDEX
):
    while True:
        sd.sleep(100)
