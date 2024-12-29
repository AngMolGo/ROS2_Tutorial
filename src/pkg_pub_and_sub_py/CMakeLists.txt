cmake_minimum_required(VERSION 3.8)
project(pkg_pub_and_sub_py)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(<dependency> REQUIRED)

install(
  PROGRAMS src/simple_subscriber.py # "Agrega el archivo 'src/hola_mundo.cpp' como ejecutable del paquete"
  RENAME py_simple_subscriber # Renómbralo con un alias.
  DESTINATION lib/${PROJECT_NAME} # "Los vas a instalar en este directorio de la instalación del paquete."
)

install(
  PROGRAMS src/simple_publisher.py # "Agrega el archivo 'src/hola_mundo.cpp' como ejecutable del paquete"
  RENAME py_simple_publisher # Renómbralo con un alias.
  DESTINATION lib/${PROJECT_NAME} # "Los vas a instalar en este directorio de la instalación del paquete."
)

# Para interfaces .msg y .srv personalizables
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IntegerAngel.msg"
  # if there was a dependency: DEPENDENCIES <package> # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
