#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sounddevice as sd
import numpy as np
import wave
import tempfile
from piper import PiperVoice, SynthesisConfig


class TextToSpeechNode(Node):

    def __init__(self):
        super().__init__("text_to_speech_node")

        # 📥 Suscripción al topic /robot_speech
        self.subscription = self.create_subscription(
            String,
            "/robot_speech",
            self.speak_callback,
            10
        )

        # 🎤 Cargar modelo Piper (ajusta ruta según tu sistema)
        MODEL_PATH = "/home/josep/robotics/ModelDownload/voices/es_ES/en_US-lessac-medium.onnx"

        self.get_logger().info("🔊 Cargando modelo de voz...")
        self.voice = PiperVoice.load(MODEL_PATH, use_cuda=False)

        # ⚙️ Configuración de voz
        self.syn_config = SynthesisConfig(
            volume=1.0,
            length_scale=1.1,
            noise_scale=0.6,
            noise_w_scale=0.8,
            normalize_audio=True
        )

        self.get_logger().info("✅ Nodo Text-to-Speech iniciado y escuchando en /robot_speech")

    def speak_callback(self, msg):
        """Callback: recibe texto y lo convierte en audio"""
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"🗣️ Texto recibido: {text}")

        # 🔊 Generar WAV temporal con Piper
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp_wav:
            with wave.open(tmp_wav.name, "wb") as wav_file:
                self.voice.synthesize_wav(text, wav_file, syn_config=self.syn_config)

            # Leer y reproducir el audio
            with wave.open(tmp_wav.name, "rb") as wf:
                sr = wf.getframerate()
                channels = wf.getnchannels()
                dtype = np.int16 if wf.getsampwidth() == 2 else np.int8
                audio = np.frombuffer(wf.readframes(wf.getnframes()), dtype=dtype)
                if channels > 1:
                    audio = audio.reshape(-1, channels)
                audio = audio.astype(np.float32) / 32768.0

            self.get_logger().info("🔈 Reproduciendo...")
            sd.play(audio, samplerate=sr)
            sd.wait()
            self.get_logger().info("✅ Reproducción completada.")


def main(args=None):
    rclpy.init(args=args)
    node = TextToSpeechNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
