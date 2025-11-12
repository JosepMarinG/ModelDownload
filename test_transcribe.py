from faster_whisper import WhisperModel

# Carga el modelo (usa "cuda" si tienes GPU)
model = WhisperModel("small", device="cpu")

# Transcribe tu archivo grabado
segments, info = model.transcribe("test.wav")

print(f"Detected language: {info.language}")

for segment in segments:
    print(f"[{segment.start:.2f}s → {segment.end:.2f}s] {segment.text}")

