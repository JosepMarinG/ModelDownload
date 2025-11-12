from openwakeword.model import Model

MODEL_PATH = "/home/josep/robotics/ModelDownload/wakewords/ModelMiniCernBot/Minicernbot.tflite"

model = Model(
    wakeword_models=[MODEL_PATH],      # ← lista con la ruta, NO diccionario
    inference_framework="tflite"
)

print("Modelos cargados:", model.models.keys())
