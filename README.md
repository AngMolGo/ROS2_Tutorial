# ROS2_Tutorial

### Para crear workspaces:

``` bash
# 1. Creamos el directorio del workspace:
mkdir <name-workspace>
cd <name-workspace>
# 2. Indicamos el directorio del workspace como variable de entorno
export WS_DEV=$PWD
# 3. Creamos el directorio /src para organizar:
mkdir -p $WS_DEV/src
```

---

### Para crear paquetes:

``` bash
# 1. Nos posicionamos en directorio /src:
cd $WS_DEV/src

# 2. Inicializamos ROS2:
source /opt/ros/jazzy/setup.bash

# 3. Creamos un nuevo paquete:
ros2 pkg create --build-type ament_cmake --license Apache-2.0 <name-package>

# 4. Completamos meta-datos del paquete:
nano $WS_DEV/src/<name-package>/package.xml
```

Rellenamos metadatos en el archivo `package.xml` del paquete:

``` xml
  <!-- Nombre del paquete -->
  <name>"name-package"</name>
  <!-- La versión del paquete -->
  <version>0.0.0</version>
  <!-- La descripción general de lo que hace el paquete -->
  <description>"TODO: Package description"</description>
  <!-- El contacto de la persona encargada del mantenimiento del paquete -->
  <maintainer email="maintainer-mail@mail.com">"name-of-maintainer"</maintainer>
  <!-- Licencia con la que se puede distribuir el paquete -->
  <license>Apache-2.0</license>
```

---

### Generar código

``` bash
# Nos movemos al archivo /src del paquete
cd $WS_DEV/src/<name-package>/src
# Y creamos todos los códigos fuente para la aplicación
touch publisher.py subscriber.cpp service.py action.cpp etc
```

> [!IMPORTANT]
> Para scripts de python es importante:\
> 1.- Agregar `#!/usr/bin/python3` al inicio del script.\
> 2.- Agregar permisos de ejecución al sistema:
> ``` bash
> chmod +x hola_mundo.py
> ```

#### Códigos básicos

[> Basic Publisher Python]()\
[> Basic Subscriber Python]()\
[> Basic Server Python]()\
[> Basic Action Python]()\
[> Basic Message Python]()

[> Basic Publisher C++]()\
[> Basic Subscriber C++]()\
[> Basic Server C++]()\
[> Basic Action C++]()\
[> Basic Message C++]()

---

### Compilar paquete

#### `Package.xml`

Modificamos el archivo `package.xml` para indicar que el paquete depende de otros paquetes:

``` xml
  <!-- Herramienta de compilación -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- DEpendencias del paquete -->
  <depend>rclcpp</depend> <!-- ROS Client Library para C++ -->
  <depend>rclpy</depend>  <!-- ROS Client Library para Python -->
  <depend>std_msgs</depend>  <!-- Formato de mensajes estándar -->
  <!-- <depend>other-package</depend>  Otras dependencias, paquetes, etc, etc... -->
```

#### `CMakeLists.txt`

Modificamos el archivo `CMakeLists.txt` para indicarle al compilador (`colcon`) las instrucciones para compilar el paquete.

Le indicamos qué paquetes debe buscar:

``` CMake
# Buscar dependencias
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED) # ROS Client Library para C++
find_package(std_msgs REQUIRED) # ROS Client Library para Python
# find_package(<dependency> REQUIRED)
```

Le indicamos instalar los ejecutables del paquete:

``` CMake
# # # # # # # PYTHON # # # # # # #

# Los scripts .py se instalan como ejecutables del paquete directamente

install(
  FILES src/hola_mundo.py # "Agrega el archivo 'src/hola_mundo.cpp' como ejecutable del paquete"
  RENAME alias_executable # Renómbralo con un alias.
  DESTINATION lib/${PROJECT_NAME} # "Los vas a instalar en este directorio de la instalación del paquete."
)

# # # # # # # # C++ # # # # # # # #

# Los archivos .cpp tienen que ser compilados, crear sus ejecutables y luego instalarlos como ejecutables del paquete.

add_executable(wellcome src/hola_mundo.cpp)  # "Agrega un ejecutable llamado 'wellcome' que ejecuta el código 'src/hola_mundo.cpp'"

install(
  TARGETS wellcome  # "Estos son los ejecutables que vas a instalar en la instalación del paquete."
  DESTINATION lib/${PROJECT_NAME}  # "Los vas a instalar en este directorio de la instalación del paquete."
  )
```

Guardamos todos los archivos y ejecutamos el compilador:

``` bash
cd $WS_DEV
colcon build # Compilamos todos los paquetes del workspace 
# colcon build --packages-select <package-name> (para compilar un package específico)
```

---

### Correr ejecutables (`ros2 run`)

``` bash
cd $WS_DEV # Se cambia a workspace
. install/setup.bash # Se cargan variables locales del workspace
ros2 run <package-name> <executable> # package > executable
```

---

### Mensajes y servicios (interfaces) personalizados


> [!TIP]
> Es buena práctica tener un paquete dedicado a las interfaces.

Los archivos `.msg` y `.srv` se deben localizar en los directorios `pkg_package/msg` y `pkg_package/srv` respectivamente. Los nombres de los archivos deben empezar estrictamente con mayúsculas y no tener espacios ni separaciones con guiones.

Un mensaje `.msg` sencillo es:

``` cmake
int64 num # <data-type> <name>
```

Para un servicio `.srv` sencillo es:

``` cmake
int64 a # Datos de solicitud del cliente
int64 b
int64 c
--- # Separador
int64 sum # Datos de respuesta del servidor
```

`package.xml`:

``` xml
<buildtool_depend>rosidl_default_generators</buildtool_depend> <!-- Para al momento de compilar -->
<exec_depend>rosidl_default_runtime</exec_depend>  <!-- Para al momento de ejecutar -->
<member_of_group>rosidl_interface_packages</member_of_group>
<!-- <depend> <package-name> </depend> -->
```

`CMake`:

``` CMake
# Para interfaces .msg y .srv personalizables
find_package(rosidl_default_generators REQUIRED)

# set(msg_files # Lista de mensajes a compilar y se les asigna una "variable"
#  "msg/AddressBook.msg"
#)

set(srv_files # Lista de servicios a compilar y se les asigna una "variable"
  "srv/IntegerSum.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  # if there was a dependency: DEPENDENCIES <package> # Add packages that above messages depend on
)

ament_export_dependencies(rosidl_default_runtime)
```

Para verificar que se crearon adecuadamente las interfaces:

``` bash
cd $WS_DEV # Cambiar a ws
source /opt/ros/jazzy/setup.bash # Cargar variables de entorno
colcon build --packages-select <package-where-interface> # Compilar package
source install/setup.bash # Cargar variables de entorno
# Para comprobar
ros2 interface show <path-to-interface-from-pkg-name>
```
