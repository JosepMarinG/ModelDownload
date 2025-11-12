"""# check_on_file.py
from openwakeword.model import Model
import numpy as np
import soundfile as sf

MODEL = "/home/josep/robotics/ModelDownload/wakewords/ModelMiniCernBot/Minicernbot.onnx"
WAKE = "Minicernbot"
m = Model(wakeword_models=[MODEL], inference_framework="onnx")

audio, sr = sf.read("/home/josep/robotics/ModelDownload/wakewords/minicernbot_raw/minicernbot_4.wav", dtype="float32")
assert sr == 16000
# 🔥 CLAVE: convertir a int16
audio_int16 = (audio * 32767).astype(np.int16)

scores = m.predict_clip(audio_int16)  # lista de dicts por frames ~80ms
mx = max(f[WAKE] for f in scores)
print("max score:", mx)
"""

from openwakeword.model import Model
MODEL = "/home/josep/robotics/ModelDownload/wakewords/ModelMiniCernBot/Minicernbot.onnx"
model = Model(wakeword_models=[MODEL], inference_framework="onnx")
scores = model.predict_clip("/home/josep/robotics/ModelDownload/audiosPrueba/alexa_test.wav")
#scores = model.predict_clip("/home/josep/robotics/ModelDownload/wakewords/minicernbot_raw/minicernbot_4.wav")
mx = max(f["Minicernbot"] for f in scores)
print("max score:", mx)
