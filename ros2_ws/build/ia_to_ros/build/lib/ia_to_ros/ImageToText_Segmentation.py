import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from openai import OpenAI
import base64

client = OpenAI(
  api_key="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"
)

initial_prompt = "Estas viendo una imagen tomada desde un robot a 30 cm de altura del suelo. Para enviar la imagen por la red hasta el teleoperador se ha decidido enviar una descripcion de la imagen en texto paara luego reconstruirla en el ordenador del teleoperador. Describe la imagen con detalle para poder reconstruirla bien despues usando dall-e-3."


class RelayNode(Node):
    def __init__(self):
        super().__init__('chatGPT_node')
        self.subscription_image = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        self.publisher_ = self.create_publisher(String, 'modelOutput', 10)
        self.conversation_history = [{"role": "system", "content": initial_prompt}]

    def image_callback(self, msg):
        self.get_logger().info('Imagen recibida, enviando a ChatGPT')
        image_base64 = base64.b64encode(msg.data).decode('utf-8')
        self.conversation_history = [{"role": "system", "content": initial_prompt}]
        self.conversation_history.append({
            "role": "user",
            "content": [
                {"type": "text", "text": "Aquí tienes la imagen solicitada"},
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_base64}"}}
            ]
        })
        self.get_logger().info(f"Imagen convertida a base64: {image_base64[:100]}...")  # Muestra los primeros 100 caracteres
        self.send_to_chatgpt()

    def send_to_chatgpt(self):
        response = client.chat.completions.create(
            model="gpt-4o",
            messages=self.conversation_history
        )
        response_text = response.choices[0].message.content
        new_msg = String()
        new_msg.data = response_text
        self.publisher_.publish(new_msg)
        self.get_logger().info(f'Publicado en modelOutput: {response_text}')


def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


