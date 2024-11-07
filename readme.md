~~~
sudo apt install ros-foxy-rosidl-generator-dds-idl
pip install empy==3.3.4 numpy==1.23.4 lark
~~~


~~~python
# 1. 需要先开动捕设备
win上打开动捕，开始进入准备状态


# 2 动捕数据传到PC上，只需动捕软件电脑wifi与PCwifi同一个即可
# 如果已经控制过真机了，就断开真机h1与pc的网线，就能运行
# 控制手臂

source install/setup.bash
ros2 run mocap_pkg mocap_control_hand

# 控制手指
source install/setup.bash
ros2 run mocap_pkg mocap_control_finger


# 3 控制真机器， unitree ros支持 接上网线
source install/setup.bash
source source ~/unitree_ros2/setup.sh
# 前面2行只需要运行一次
ros2 run mocap_pkg h1_arm_hand
~~~