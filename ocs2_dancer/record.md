## TODO
### dancer_interface_ros
- rviz文件夹内rviz文件未完成 （已完成）
- src/visualization/LeggedRobotVisualizer.cpp： line141关节未修改（需查看dancer的urdf文件修改） （已完成）
- 所有launch文件的urdf文件地址未修改 （已完成）


### dancer_interface
- config/command/ reference.info 关节初始角度和质心高度未修改
- config/mpc/ task.info 关节初始角度和质心高度未修改
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
- 目前存在p gain 不等于自由度的报错
