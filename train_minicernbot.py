from openwakeword.model import Model
from openwakeword.utils import train_custom_wakeword

# Carpeta con los audios
audio_dir = "/home/josep/robotics/ModelDownload/wakewords/minicernbot_raw"

train_custom_wakeword(
    wakeword_name="minicernbot",
    audio_dir=audio_dir,
    output_dir="./models/minicernbot",
)

print("✅ Entrenamiento completado")
