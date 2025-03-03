import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI

client = OpenAI(
  api_key="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"
)

initial_prompt = "Tienes control sobre un robot móvil. Este robot se mueve cuando recibe comandos en el siguiente formato:\n\n" \
         "    move{x=X,y=Y,z=Z} → Define la velocidad en metros por segundo (m/s) (maxima 0.1) en los ejes X, Y y la velocidad de giro en el eje Z en radianes/segundo, velocidad maxima de 0.1 tanto lineal como angular.\n" \
         "    wait(T) → Hace que el robot espere T segundos antes de continuar.\n\n" \
         "Reglas importantes:\n\n" \
         "    - Tu respuesta debe contener exclusivamente comandos en este formato, sin explicaciones ni texto adicional.\n" \
         "    - El robot no puede teletransportarse, solo se mueve gradualmente en función de la velocidad.\n" \
         "    - Si se te pide una acción compleja (como hacer un cuadrado), desglósala en pasos secuenciales con movimientos y pausas.\n" \
         "    - Si el usuario da una nueva orden, ten en cuenta el estado actual del robot para que la respuesta tenga sentido.\n" \
         "    - Reacciona como si tuvieras vida, si te pregunta si estás escuchando, muévete hacia los lados, etc.\n\n" \
         "Ejemplo de entrada y salida esperada:\n" \
         "Entrada:\n\n" \
         "Haz un cuadrado de 1 metro de lado.\n" \
         "Salida esperada:\n\n" \
         "move{x=0.1,y=0.0,z=0.0}\n" \
         "wait(1.0)\n" \
         "move{x=0.0,y=0.1,z=0.0}\n" \
         "wait(1.0)\n" \
         "move{x=-0.1,y=0.0,z=0.0}\n" \
         "wait(1.0)\n" \
         "move{x=0.0,y=-0.1,z=0.0}\n" \
         "wait(1.0)\n\n" \
         "Esta es la nueva instrucción que te están diciendo:\n"\
         "velocidad maxima de 0.1\n"



class RelayNode(Node):
    def __init__(self):
        super().__init__('chatGPT_node')
        # Suscriptor al topic 'modelInput'
        self.subscription = self.create_subscription(
            String,
            'modelInput',
            self.listener_callback,
            10
        )
        # Publicador en el topic 'modelOutput'
        self.publisher_ = self.create_publisher(String, 'modelOutput', 10)

    def listener_callback(self, msg):
        self.get_logger().info('Recibido en modelInput: "%s"' % msg.data)

        prompt = initial_prompt + msg.data

        completion = client.chat.completions.create(
            model="gpt-3.5-turbo",
            store=True,
            messages=[
                {"role": "user", "content": prompt}
            ]
        )

        #print(completion.choices[0].message.content);

        # Se crea un mensaje nuevo para publicar en modelOutput
        new_msg = String()
        new_msg.data = completion.choices[0].message.content
        self.publisher_.publish(new_msg)
        self.get_logger().info('Publicado en modelOutput: "%s"' % new_msg.data)

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