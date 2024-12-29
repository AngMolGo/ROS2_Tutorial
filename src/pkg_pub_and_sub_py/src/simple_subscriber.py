#!/usr/bin/env python3

# # # Módulos (dependencias) # # #
import rclpy # Importa ROS Client Library para Python.
from rclpy.node import Node # Importa la clase Node para la creación de nodos.

from std_msgs.msg import String # Importa la "clase" msg para dar formato a los mensajes de los tópicos que leerá el suscriptor.


# # # Definición de la clase Suscriptor # # #
# Se define una clase "Subscriptor" que hereda de la clase padre Node antes importada.
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber') # Inicialización como se tratara de la clase padre, se crea un nodo con el nombre "minimal_subscriber".
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10) # Crea un "suscriptor", una parte del nodo que tiene entrada del tópico. Indica que se suscribirá a mensajes de tipo String (clase importada) de un topico llamado "topic". (10 es el tamaño del queue). Cada vez que escuche un mensaje ejecutará la función "listener_callback".
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg): # Uno de los parámetros de la función es el mensaje recibido en el tópico.
        self.get_logger().info('I heard: "%s"' % msg.data) # Cada vez que "escuche" un mensaje del tópico, indicará con un log el mensaje recibido.


# # # Código main # # #
def main(args=None):
    rclpy.init(args=args) # Inicializa la comunicación con el sistema de comunicación de ROS2. 

    minimal_subscriber = MinimalSubscriber()  # Se crea una instancia de la clase desarrollada.

    rclpy.spin(minimal_subscriber) # Mantiene un bucle que permite mantener al nodo minimal_subscriber "vivo".

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node() # Cuando se acaba el bucle se autodestruye el nodo.
    rclpy.shutdown() # Termina la sesión de comunicación con el entorno ROS2.

if __name__ == '__main__':
    main()