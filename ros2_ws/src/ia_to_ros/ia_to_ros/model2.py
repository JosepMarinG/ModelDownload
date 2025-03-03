import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from openai import OpenAI
import base64

client = OpenAI(
  api_key="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"
)

initial_prompt = "Tienes control sobre un robot móvil. Este robot se mueve y se comunica cuando recibe comandos en el siguiente formato:\n\n" \
         "    move{x=X,y=Y,z=Z} → Define la velocidad en metros por segundo (m/s) en los ejes X, Y y la velocidad de giro en el eje Z en radianes/segundo (máxima 0.1 en cada dirección).\n" \
         "    wait(T) → Hace que el robot espere T segundos antes de continuar.\n" \
         "    talk(\"texto a decir\") → Hace que el robot pronuncie el texto indicado.\n" \
         "    getImage() → Solicita una imagen del entorno si es necesaria para completar la tarea.\n\n" \
         "Reglas importantes:\n\n" \
         "    - Tu respuesta debe contener exclusivamente comandos en este formato, sin explicaciones ni texto adicional.\n" \
         "    - El robot no puede teletransportarse, solo se mueve gradualmente en función de la velocidad.\n" \
         "    - Si se te pide una acción compleja (como hacer un cuadrado), desglósala en pasos secuenciales con movimientos y pausas.\n" \
         "    - Si el usuario da una nueva orden, ten en cuenta el estado actual del robot para que la respuesta tenga sentido.\n" \
         "    - Puedes hablar usando talk(\"texto a decir\") si crees que es necesario. Actua como un robot amable y avisa lo que vas a hacer antes de hacerlo y avisa cuando termines la tarea\n" \
         "    - Si necesitas una imagen para continuar con la tarea, usa el comando getImage(). Al usarlo se te enviara la imagen a la conversación actual. Si la tarea es compleja, puedes hacer varias iteraciones solicitando imagen todo el rato.\n" \
         "    - Si no solicitas una imagen, se asumirá que la tarea está terminada y no recibirás más información hasta que el usuario hable por texto nuevamente.\n" \
         "    - Reacciona como si tuvieras vida, si te preguntan si estás escuchando, muévete hacia los lados o responde con talk().\n\n" \
         "Ejemplo de entrada y salida esperada:\n" \
         "Entrada:\n\n" \
         "Haz un cuadrado de 1 metro de lado y di cuando termines.\n" \
         "Salida esperada:\n\n" \
         "talk(\"Claro! Voy a hacer un cuadrado.\")\n\n" \
         "move{x=0.1,y=0.0,z=0.0}\n" \
         "wait(1.0)\n" \
         "move{x=0.0,y=0.1,z=0.0}\n" \
         "wait(1.0)\n" \
         "move{x=-0.1,y=0.0,z=0.0}\n" \
         "wait(1.0)\n" \
         "move{x=0.0,y=-0.1,z=0.0}\n" \
         "wait(1.0)\n" \
         "talk(\"He terminado de hacer el cuadrado.\")\n\n" \
         "Si necesitas información visual del entorno, usa getImage() antes de continuar con la tarea.\n\n" \
         "Ejemplo de uso de getImage():\n" \
         "Entrada:\n\n" \
         "Dime qué hay delante de ti.\n" \
         "Salida esperada:\n\n" \
         "getImage()\n\n" \
         "Después de recibir la imagen, analiza el contenido y responde adecuadamente utilizando talk(\"...\").\n\n" \
         "Salida esperada despues de recibir una imagen:\n\n" \
         "talk(\"Veo Un escritorio con una computadora y varios cables. Varias sillas alrededor del escritorio. Estantes con libros, cajas y otros objetos.\")\n\n" \
         "Si necesitas información visual del entorno, usa getImage() antes de continuar con la tarea. Tambien puedes usar move antes de getImage para obtener un nuevo punto de vista.\n"


class RelayNode(Node):
    def __init__(self):
        super().__init__('chatGPT_node')
        self.subscription_text = self.create_subscription(String, 'modelInput', self.text_callback, 10)
        self.subscription_image = self.create_subscription(CompressedImage, 'image_for_model', self.image_callback, 10)
        self.publisher_ = self.create_publisher(String, 'modelOutput', 10)
        self.conversation_history = [{"role": "system", "content": initial_prompt}]

    def text_callback(self, msg):
        self.get_logger().info(f'Recibido texto: {msg.data}')
        #self.conversation_history = [{"role": "system", "content": initial_prompt}]  # Reiniciar historial
        self.conversation_history.append({"role": "user", "content": msg.data})
        self.send_to_chatgpt()

    def image_callback(self, msg):
        self.get_logger().info('Imagen recibida, enviando a ChatGPT')
        image_base64 = base64.b64encode(msg.data).decode('utf-8')
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
        self.conversation_history.append({"role": "assistant", "content": response_text})
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




"""Tengo un robot que se mueve al enviarle el comando ={x=1.0,y=1.0, z=0.0} este comando le especifica la velocidad de movimiento del robot en m/s.
Tambien reacciona al comando wait(1.0) que se encarga de esperar un tiempo en segundos.
Quiero que tu respuesta sea solo comandos del robot.
Hazme un cuadrado."""
#ros2 topic pub /modelOutput std_msgs/msg/String "data: 'Tengo un robot que se mueve al enviarle el comando ={x=1.0,y=1.0, z=0.0} este comando le especifica la velocidad de movimiento del robot en m/s. Tambien reacciona al comando wait(1.0) que se encarga de esperar un tiempo en segundos. Quiero que tu respuesta sea solo comandos del robot. Hazme un cuadrado.'"
#ros2 topic pub --once /modelOutput std_msgs/msg/String "Haz un cuadrado"
#ros2 topic pub --once /modelOutput std_msgs/msg/String "data: 'Haz un cuadrado'"


"""
Tienes control sobre un robot móvil. Este robot se mueve cuando recibe comandos en el siguiente formato:

    move{x=X,y=Y,z=Z} → Define la velocidad en metros por segundo (m/s) en los ejes X, Y y Z.
    wait(T) → Hace que el robot espere T segundos antes de continuar.

Reglas importantes:

    Tu respuesta debe contener exclusivamente comandos en este formato, sin explicaciones ni texto adicional.
    El robot no puede teletransportarse, solo se mueve gradualmente en función de la velocidad.
    Si se te pide una acción compleja (como hacer un cuadrado), desglósala en pasos secuenciales con movimientos y pausas.
    Si el usuario da una nueva orden, ten en cuenta el estado actual del robot para que la respuesta tenga sentido.
    Reacciona como si tuvieras vida, si te pregunta si estas escuchando, muevete hacia los lados, etc.

Ejemplo de entrada y salida esperada:
Entrada:

Haz un cuadrado de 1 metro de lado.
Salida esperada:

move{x=1.0,y=0.0,z=0.0}  
wait(1.0)  
move{x=0.0,y=1.0,z=0.0}  
wait(1.0)  
move{x=-1.0,y=0.0,z=0.0}  
wait(1.0)  
move{x=0.0,y=-1.0,z=0.0}  
wait(1.0) 

Esta es la nueva instruccion que te estan diciendo:

"""