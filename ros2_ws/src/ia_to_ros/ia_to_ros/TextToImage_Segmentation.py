import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from openai import OpenAI
import requests
import io
import numpy as np
from PIL import Image as PILImage
from cv_bridge import CvBridge

initial_prompt = "Eres un robot a 30 cm de altura del suelo. Para enviar la imagen por la red hasta el teleoperador se ha decidido enviar una descripcion de la imagen en texto para ahora reconstruirla en el ordenador del teleoperador. Crea la imagen basada en la siguiente descripcion:"



class ImageReconstructor(Node):
    def __init__(self):
        super().__init__('image_reconstructor_node')
        self.subscription = self.create_subscription(String, 'modelOutput', self.description_callback, 10)
        self.publisher_ = self.create_publisher(Image, 'reconstructed_image_raw', 10)
        self.client = OpenAI(api_key="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")
        self.bridge = CvBridge()

    def description_callback(self, msg):
        self.get_logger().info(f'Recibida descripción: {msg.data}')
        image_url = self.generate_image(msg.data)

        if image_url:
            self.publish_image(image_url)

    def generate_image(self, description):
        try:
            response = self.client.images.generate(
                model="dall-e-3",
                prompt=initial_prompt+description,
                size="1024x1024",
                quality="standard",
                n=1
            )
            return response.data[0].url  # Devuelve la URL de la imagen generada
        except Exception as e:
            self.get_logger().error(f'Error generando imagen: {e}')
            return None

    def publish_image(self, image_url):
        try:
            # Descargar la imagen desde la URL generada
            response = requests.get(image_url)
            image = PILImage.open(io.BytesIO(response.content))

            # Convertir la imagen a un array de NumPy
            image_np = np.array(image.convert("RGB"))  # Convertir a RGB (3 canales)

            # Convertir a mensaje de ROS2
            msg = self.bridge.cv2_to_imgmsg(image_np, encoding="rgb8")

            # Publicar la imagen en el tópico
            self.publisher_.publish(msg)
            self.get_logger().info('Imagen reconstruida publicada en /reconstructed_image_raw')
        except Exception as e:
            self.get_logger().error(f'Error publicando imagen: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageReconstructor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
