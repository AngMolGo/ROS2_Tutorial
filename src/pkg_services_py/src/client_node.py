#!/usr/bin/env python3

# # # Módulos (dependencias) # # #
import sys # Permite la interacción con el cmd

from example_interfaces.srv import AddTwoInts # Importa la "clase" AddTwoInts para dar formato a la interfaz (mensajes de comunicación entre un cliente y un servidor) cliente-servidor.

import rclpy # Importa ROS Client Library para Python.
from rclpy.node import Node # Importa la clase Node para la creación de nodos.

# # # Definición de la clase Cliente # # #
class MinimalClientAsync(Node): # Se define una clase "Cliente" que hereda de la clase padre Node antes importada.

    def __init__(self):
        super().__init__('minimal_client_async')  # Inicialización como se tratara de la clase padre, se crea un nodo con el nombre "minimal_client_async".
        self.cli = self.create_client(  # Crea un nodo "cliente", una parte del nodo que tiene salida a una interfaz hacia el servidor "add_two_ints". Indica que los mensajes hacia el servidor son de tipo AddTwoInts (clase importada). Retorna una clase cliente.
            AddTwoInts,  # Tipo de mensaje que interpreta el servidor.
            'add_two_ints') # Servidor al que realizará la petición.
        while not self.cli.wait_for_service(timeout_sec=1.0): # Mientra el cliente no detecte una conexión con el servidor va a esperar mientras imprime un log.
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request() # Crea una clase del mensaje ("AddTwoInts") que va a ser el que envíe al servidor.

    def send_request(self, a, b): # Método que manda un request al servidor
        self.req.a = a # Parte de la petición de la interfaz AddTwoInts
        self.req.b = b # Parte de la petición de la interfaz AddTwoInts
        return self.cli.call_async(self.req) # Hace el request por medio de la clase Cliente


def main():
    rclpy.init() # Inicializa la comunicación con el sistema de comunicación de ROS2. 

    minimal_client = MinimalClientAsync() # Se crea una instancia de la clase desarrollada.

    future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2])) # Mediante el método de petición del cliente se obtiene la respuesta del servidor. Un objeto de tipo AddTwoInts que alberga el mensaje.
    rclpy.spin_until_future_complete(minimal_client, future) # Espera el resultado del servidor de minimal_client con el mensaje future
    response = future.result() # La respuesta de la solicitud que se alberga dentro del objeto future se asigna a response.
    minimal_client.get_logger().info( # Hace un log con el resultado
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node() # Se autodestruye el nodo.
    rclpy.shutdown() # Termina la comunicación con ROS2.

if __name__ == '__main__':
    main()