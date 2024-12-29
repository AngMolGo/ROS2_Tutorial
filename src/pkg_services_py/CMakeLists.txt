cmake_minimum_required(VERSION 3.8)
project(pkg_services_py)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(example_interfaces REQUIRED)

install(
  PROGRAMS src/service_member.py # "Agrega el archivo 'src/aservice_member.py' como ejecutable del paquete"
  RENAME server_member # Renómbralo con un alias.
  DESTINATION lib/${PROJECT_NAME} # "Los vas a instalar en este directorio de la instalación del paquete."
)

install(
  PROGRAMS src/client_node.py # "Agrega el archivo 'src/client_node.py' como ejecutable del paquete"
  RENAME server_client # Renómbralo con un alias.
  DESTINATION lib/${PROJECT_NAME} # "Los vas a instalar en este directorio de la instalación del paquete."
)

# Para interfaces .msg y .srv personalizables
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/IntegerSum.srv"
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
