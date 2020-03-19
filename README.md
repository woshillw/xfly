1.本功能包使用了[RotorS](https://github.com/ethz-asl/rotors_simulator)的部分Gazebo插件，在使用之前需要安装RotorS,并且参考了[推力矢量可倾转多旋翼Gazebo仿真](https://github.com/LLlkaiwen/vfly)

2.获取功能包和依赖  
`$ cd ~/catkin_ws/src`
`$ git clone https://github.com/woshillw/xfly.git`
`$ git clone https://github.com/woshillw/pid_control.git`

3.启动四旋翼  
`roslaunch xfly mav.launch mav_name:=xfly type:=x`

4.启动pid控制器  
`roslaunch xfly xfly_pid_controller.launch `

5.发送控制指令  
`rostopic pub -1 /xfly/command/pose xfly/xfly_pose "header:
seq: 0
stamp: {secs: 0, nsecs: 0}
frame_id: ''
x: 3.0
y: 3.0
z: 3.0
" `

6.如果不会使用ROS，可以参考[llwrobot](https://github.com/woshillw/llwrobot)
===
