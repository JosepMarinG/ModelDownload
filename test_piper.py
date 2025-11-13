import wave
import sounddevice as sd
import numpy as np
from piper import PiperVoice, SynthesisConfig

MODEL_PATH = "/home/josep/robotics/ModelDownload/voices/es_ES/en_US-lessac-medium.onnx"

# Cargar la voz (usa GPU si quieres más velocidad)
voice = PiperVoice.load(MODEL_PATH, use_cuda=False)

syn_config = SynthesisConfig(
    volume=1.0,
    length_scale=1.1,     # un poco más lento
    noise_scale=0.6,      # más natural
    noise_w_scale=0.8,
    normalize_audio=True
)

text = "Hello Josep! I am MiniCernBot, ready for action."

print("🎙️ Generando voz...")
wav_path = "test_piper.wav"

# Guardar el WAV
with wave.open(wav_path, "wb") as wav_file:
    voice.synthesize_wav(text, wav_file, syn_config=syn_config)

# 🔊 Leer el WAV y reproducirlo directamente
with wave.open(wav_path, "rb") as wf:
    sr = wf.getframerate()
    channels = wf.getnchannels()
    dtype = np.int16 if wf.getsampwidth() == 2 else np.int8
    audio = np.frombuffer(wf.readframes(wf.getnframes()), dtype=dtype)
    if channels > 1:
        audio = audio.reshape(-1, channels)
    audio = audio.astype(np.float32) / 32768.0  # normaliza a [-1, 1]

print("🔈 Reproduciendo...")
sd.play(audio, samplerate=sr)
sd.wait()

print("✅ Listo. Se generó y reprodujo correctamente.")
