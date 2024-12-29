#!/usr/bin/env python3

# # # Módulos (dependencias) # # #
import rclpy # Importa ROS Client Library para Python.
from rclpy.node import Node # Importa la clase Node para la creación de nodos.

from std_msgs.msg import String # Importa la "clase" msg para dar formato a los mensajes de los tópicos que publicará el publicador.


# # # Definición de la clase Publicador # # #
# Se define una clase "Publicador" que hereda de la clase padre Node antes importada.
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher') # Inicialización como se tratara de la clase padre, se crea un nodo con el nombre "minimal_publisher".
        self.publisher_ = self.create_publisher(
            String,
            'topic', 
            10) # Crea un "publicador", una parte del nodo que tiene salida al tópico. Indica que publicará mensajes de tipo String (clase importada) a un topico llamado "topic". (10 es el tamaño del queue)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Se creará un timer, cada periodo de tiempo (0.5s) se llamará a la función callback
        self.i = 0

    def timer_callback(self):
        msg = String() # Clase String importada
        msg.data = 'Hello World: %d' % self.i # Se actualiza el tributo de la clase String del objeto msg
        self.publisher_.publish(msg) # Se llama al método del publicador que publica una instancia de la clase mensaje con el mensaje.
        self.get_logger().info('Publishing: "%s"' % msg.data) # El nodo arroja un log con el mensaje publicado.
        self.i += 1 # Parte del programa es aumentar el contador

def main(args=None):
    rclpy.init(args=args) # Inicializa la comunicación con el sistema de comunicación de ROS2. 

    minimal_publisher = MinimalPublisher() # Se crea una instancia de la clase desarrollada.

    rclpy.spin(minimal_publisher) # Mantiene un bucle que permite mantener al nodo minimal_publisher "vivo".

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node() # Cuando se acaba el bucle se autodestruye el nodo.
    rclpy.shutdown() # Termina la sesión de comunicación con el entorno ROS2.

if __name__ == '__main__':
    main()