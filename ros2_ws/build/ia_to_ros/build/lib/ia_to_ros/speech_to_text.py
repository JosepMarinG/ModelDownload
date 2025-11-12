#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import numpy as np
import sounddevice as sd
from collections import deque
from openwakeword.model import Model
import webrtcvad
import time
from faster_whisper import WhisperModel


WAKEWORD = "alexa"
SAMPLE_RATE = 16000
BLOCK = 160
BUFFER_SECONDS = 2.0
THRESHOLD = 0.01

AFTER_WAKEWORD_SECONDS = 4.0


class VoiceNode(Node):

    def __init__(self):
        super().__init__("voice_control")

        self.publisher = self.create_publisher(String, "/voice/text", 10)

        self.model = Model(wakeword_models=[WAKEWORD], inference_framework="onnx")
        self.vad = webrtcvad.Vad(2)

        self.whisper = WhisperModel("small", device="cpu")

        self.buffer = deque(maxlen=int(SAMPLE_RATE * BUFFER_SECONDS))

        self.recording = False
        self.recorded_audio = []
        self.record_start = 0

        self.get_logger().info("🎤 Wakeword + Whisper node iniciado")

        sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype="float32",
            blocksize=BLOCK,
            device=6,
            callback=self.audio_callback
        ).start()

    def audio_callback(self, indata, frames, timestamp, status):
        audio16 = (indata[:, 0] * 32768).astype(np.int16).tobytes()

        # 1️⃣ Aún no se dijo la wakeword: solo detectar
        if not self.recording:
            if self.vad.is_speech(audio16, SAMPLE_RATE):
                self.buffer.extend(indata[:, 0])

            if len(self.buffer) >= SAMPLE_RATE:
                samples = np.array(self.buffer, dtype=np.float32)
                self.buffer.clear()

                max_val = np.max(np.abs(samples))
                if max_val > 0:
                    samples /= max_val

                scores = self.model.predict_clip(samples)
                score = max(frame[WAKEWORD] for frame in scores)

                print(f"score={score:.4f}")

                if score > THRESHOLD:
                    self.get_logger().info(f"🚀 WAKEWORD DETECTADA: {WAKEWORD.upper()}")
                    self.recording = True
                    self.record_start = time.time()
                    self.recorded_audio = []
            return

        # 2️⃣ Grabación después de wakeword
        self.recorded_audio.extend(indata[:, 0])

        if time.time() - self.record_start > AFTER_WAKEWORD_SECONDS:
            audio_np = np.array(self.recorded_audio, dtype=np.float32)
            text = self.run_whisper(audio_np)
            self.publish_text(text)

            self.recorded_audio = []
            self.recording = False

    def run_whisper(self, audio_np):
        print("📡 Enviando audio a Whisper...")
        segments, info = self.whisper.transcribe(audio_np)
        text = ""
        for segment in segments:
            print(f"[{segment.start:.2f}s → {segment.end:.2f}s] {segment.text}")
            text += segment.text + " "
        print(f"📝 Transcripción completa: {text}")
        return text

    def publish_text(self, text):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.get_logger().info(f"✅ Publicado: {text}")


def main():
    rclpy.init()
    node = VoiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
