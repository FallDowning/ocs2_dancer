## TODO
### dancer_interface_ros
- rviz文件夹内rviz文件未完成 （已完成）
- src/visualization/LeggedRobotVisualizer.cpp： line141关节未修改（需查看dancer的urdf文件修改） （已完成）
- 所有launch文件的urdf文件地址未修改 （已完成）


### dancer_interface
- config/command/ reference.info 关节初始角度和质心高度未修改 （已完成）
- config/mpc/ task.info 关节初始角度和质心高度未修改 （已完成）
- src/ LeggedRobotinterface.cpp: 加入了一些关于零力矩约束的计算，修改？

### dancer_controllers
launch 文件 的resource 文件地址为dancer_description的dae文件，还未导入 （已完成）

### dancer_raisim
未完成，dancer_controllers包的一个依赖包 (已完成)

### dancer_description
未导入urdf模型及相关的stl文件 （已完成）


## TODO（3.5）
- 可以尝试编译项目+debug了 （在wukong4_control文件夹下）

## Modified（3.9）
- 修改了一些程序bug，导入了新的urdf模型，目前urdf模型还存在一些问题（collision是stl文件的情况下无法加载）（已解决）
- 目前存在这样的报错 “p gains should have the same dimension as the degrees of freedom”

## Modified（3.10）
- 将urdf中的与走路无关的关节设置为fixed，可以解决3.9的问题
- 目前rviz可加载出模型，但走路姿势不正常

## Modified (3.13)
- 在dancer_controllers 加入dancerDummynode.cpp和dancerMpcTestNode.cpp 
- 在dancer_controllers 加入dancer_test.launch 文件
- 目前可以roslaunch dancer_test.launch文件，机器人可以正常运动
## TODO（3.13）
- 目前存在机器人初始脚底板翘起的问题，或许可通过修改urdf文件或加入初始位姿实现
