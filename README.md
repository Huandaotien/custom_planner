# Requirements:

- Ubuntu 20.04.6 LTS
- ROS Noetic

- Install sbpl library:
    ```
    git clone https://github.com/sbpl/sbpl.git
    mkdir build
    cd build
    cmake ..
    make -j4
    sudo make install
    ```
- Package dependency:
    + vda5050_api: https://github.com/nhamtung/vda5050_api.git

# Service and topic:

- Service file: 
    + file location: ../planner_ws/src/custom_planner/srv/PlanWithOrder.srv
    + type: custom_planner/PlanWithOrder
    + dependency: vda5050_msgs/Order

- Topic name: "/set_plan_with_order"
