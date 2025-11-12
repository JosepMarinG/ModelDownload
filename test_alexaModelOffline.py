from openwakeword.model import Model
import soundfile as sf
import numpy as np

m = Model(wakeword_models=["alexa"], inference_framework="onnx")

audio, sr = sf.read("/home/josep/robotics/ModelDownload/audiosPrueba/alexa_test.wav")
#audio, sr = sf.read("/home/josep/robotics/ModelDownload/wakewords/minicernbot_raw/minicernbot_4.wav")
assert sr == 16000

# 🔥 CLAVE: convertir a int16
audio_int16 = (audio * 32767).astype(np.int16)

scores = m.predict_clip(audio_int16)
mx = max(f["alexa"] for f in scores)
print("max score:", mx)


"""
from openwakeword.model import Model

model = Model(wakeword_models=["alexa"], inference_framework="onnx")
scores = model.predict_clip("/home/josep/robotics/ModelDownload/audiosPrueba/alexa_test.wav")
mx = max(f["alexa"] for f in scores)
print("max score:", mx)"""
