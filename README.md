# NiOSR_projekt
Projekt zaliczeniowy 

Należy w pierwszym oknie terminala w przypadku pierwszej kompilacji środowiska ros2 wywołać komendy:

source /opt/ros/humble/setup.bash

lub w przypadku kolejnej kompilacji 

source install/setup.bash

Następnie 

colcon build

W drugim oknie terminala uruchamiamy turtlesim poleceniem:

ros2 run turtlesim turtlesim_node

Wracamy do poprzedniego okna terminala i uruchamiamy:

ros2 run camera_subscriber camera_node

Uruchomi się okno kontrolera robożółwia turtlesim, który umożliwa poruszanie robotem w dwóch kierunkach w zależności czy użytkownik kliknie w górną część ekranu lub dolną.

Dodatkowe zasoby, które mogą być pomocne:

Dokumentacja ROS2 - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
Dokumentacja turtlesim - https://wiki.ros.org/turtlesim
