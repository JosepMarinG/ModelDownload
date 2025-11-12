from openwakeword.model import Model
from openwakeword.utils import bulk_predict

FILE = "/home/josep/robotics/ModelDownload/audiosPrueba/alexJarvis_16k.wav"

model = Model(
    wakeword_models=["alexa"],     # <--- fuerza solo Alexa
    inference_framework="onnx"     # <--- evita problemas de tflite runtime
)

print("✅ Modelo cargado")

# --- PREDICCIÓN DIRECTA DE UN AUDIO ---
result = model.predict_clip(FILE)
print("🔍 Resultado predict_clip():")
print(result)

# --- BULK PREDICT ---
print("\n🚀 Bulk predict:")
results = bulk_predict(
    file_paths=[FILE],
    wakeword_models=["alexa"],
    ncpu=2
)

print(results)
