#!/usr/bin/env python3

# # # Módulos (dependencias) # # #
from example_interfaces.srv import AddTwoInts # Importa la "clase" AddTwoInts para dar formato a la interfaz (mensajes de comunicación entre un cliente y un servidor) cliente-servidor.

import rclpy # Importa ROS Client Library para Python.
from rclpy.node import Node # Importa la clase Node para la creación de nodos.

# # # Definición de la clase Servidor # # #
# Se define una clase "Servidor" que hereda de la clase padre Node antes importada. Se podría decir que un servidor es un nodo que brinda un servicio.
class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service') # Inicialización como se tratara de la clase padre, se crea un nodo con el nombre "minimal_service".
        self.srv = self.create_service(  # Crea un "servidor" una parte del nodo que brinda un servicio.  (10 es el tamaño del queue). Cada vez que escuche un mensaje ejecutará la función "listener_callback".
            AddTwoInts, # Indica que sus "mensajes" (interfaz cliente-servidor) son de tipo AddTwoInts (clase importada).
            'add_two_ints', # Ahorita vemos qué significa
            self.add_two_ints_callback) # Ejecuta el código del método "callback" cada vez que se hace una petición.

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b # El servicio suma dos enteros, que fueron pasados como parámetros.
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b)) # Realiza un Log y muestra los datos obtenidos.

        return response # Resgresa la respuesta con el formato response proveniente de la clase AddTwoInts


def main():
    rclpy.init() # Inicializa la comunicación con el sistema de comunicación de ROS2. 

    minimal_service = MinimalService() # Se crea una instancia de la clase desarrollada.

    rclpy.spin(minimal_service) # Mantiene un bucle que permite mantener al nodo minimal_servidor "vivo".

    rclpy.shutdown() # Termina la sesión de comunicación con el entorno ROS2.

if __name__ == '__main__':
    main()