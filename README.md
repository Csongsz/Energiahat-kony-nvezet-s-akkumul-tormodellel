Energiahatékony önvezetés akkumulátormodellel

könyvtaram felepitese: ros2_onvezetes/src/

wayp_plan_tools --> itt hoztam letre a nodeomat, CMakeLists.txt-ben modositasok

sim_wayp_plan_tools --> itt csak a mindent egyszerre launch faljban valtoztattam

futtatás:

colcon build

source ~/.bashrc

source install/setup.bash

ign gazebo -v 4 -r ackermann_steering.sdf


colcon build

source ~/.bashrc

source install/setup.bash

ros2 launch sim_wayp_plan_tools all_in_once.launch.py
