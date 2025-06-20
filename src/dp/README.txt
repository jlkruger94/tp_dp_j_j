#*******************************************************
# Directorio de trabajo
cd ~/ros2_ws

#*******************************************************
# Compilar doble pendulo
colcon build --packages-select dp

#*******************************************************
# Correr la visualización solamente
source install/setup.bash 
ros2 launch dp display.launch.py 

#*******************************************************
# Correr la simulación con el control (ROS2+Gz+RViz2)
source install/setup.bash 
ros2 launch dp dp_sim.launch.py 

#*******************************************************
# Correr la simulación a lazo abierto (ROS2+Gz+RViz2)
# Se usa effort controller para impostar directamente valores de torque en los ejes
# Además el launch levanta el double_pendulum_caos.xacro que no tiene rozamiento y los límites de ejes están para
# muchas vueltas, así como los límites de torque y de velocidad. De esta manera no se limita el comportamiento caótico
source install/setup.bash 
ros2 launch dp dp_sim.launch.py 

#*******************************************************
#Ver el URDF donde se definen los nombres de los joints.
ros2 param get /robot_state_publisher robot_description

#*******************************************************
#Muestra el estado del tópico (tipo de mensaje y cuántos están subscriptos)
ros2 topic info /joint_states
 
#*******************************************************
# Muestra el valor del tópico
ros2 topic echo /joint_states

#*******************************************************
# Mover el robot por terminal (sin control)
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, name: ['joint1', 'joint2'], position: [1.0, 0.5], velocity: [], effort: []}"

#*******************************************************
# Activar/desactivar el control de posición
ros2 control set_controller_state position_controller inactive

o bien usar gui
ros2 run dp toggle_controller

#*******************************************************
# Impone un impulso de torque
ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, 0.0]"

#*******************************************************
# Impone una posicion de referencia al controlador position_controllers (le puedo pasar solo un destino pero no admite PID)
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.57, 0]" --once

#*******************************************************
# Impone una posicion de referencia al controlador joint_trajectory (le puedo pasar una trayectoria suave)
ros2 topic pub /position_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "joint_names: ['joint1', 'joint2']
points:
- positions: [0, 1.57]
  time_from_start: {sec: 1}" --once

#*******************************************************
# Graficador de topics de ROS
ros2 run plotjuggler plotjuggler 

#*******************************************************
# Cambiar ganancias dinámicamente
ros2 param set /position_controller gains.joint1.p 120.0
ros2 param get /position_controller gains

#*******************************************************
# Ajuste por ZN 
#Eje1:
Ku = 120
Tu = 0.150
P = 0.6*Ku 
I = 1.2 * Ku / Tu 
D = 0.075 * Ku * Tu 

# Eje 2
Ku=90
Tu=0.060

