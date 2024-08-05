- aubo 提供 urdf (不发送)
- aubo_description 提供 kde（aubo 官方指定）
- bringup 提供 launch 文件
- vr_cali 标定动作. in: tf(ref, tracker_random) out: tf(custom, tracker_upright)
- vr_track_tcp 接收 windows 发来的 tracker 坐标. in: tcp(localhost: 3001) out: tf(ref, tracker_random)
- ik_kdl in: load urdf, tf(tracker_upright, custom) | out(tcp: 192.168.38.128:8899 (to vm))

## windows side
- in: vr sdk | out: tcp(wsl:3001)

So:

## full kdl

```
ros2 launch bringup cali.py
# windows: ./opencv-tcp.exe
# open VMWARE
# Calibrate.
ros2 launch bringup kdl.py
```

## Full moveit

```
ros2 launch bringup cali.py
# windows: ./opencv-tcp.exe
# Calibrate. using cli: j.p0, j.px, j.py, j.cali, j.up (or simply j.l to load)
ros2 launch moveit_config demo.launch.py 
colcon build --packages-select ik && ros2 run ik ik_joint_broadcaster_exe
colcon build --packages-select ik bringup && ros2 launch bringup moveit.py
```

## To show robot model

```
colcon build --symlink-install --packages-select aubo aubo_description
jst bs install/setup.bash
ros2 launch aubo launch.py
```

## 神秘科技

- 神秘的在线插补，限制关节速度和角速度
- hand 静止检测
- 延迟抽象，进行位置预测
- 光速碰撞检测

## todo:

- [x] angle interpolation seems wrong.
- [x] 电机发的是两圈的，注意内部处理。
- [x] 机械臂开机怎么开，更新机械臂驱动，加速 macsize 获取
- [x] 仿真模式和真机模式都有很大延迟，奇怪的是，即使仿真时控制 mac 调小，延迟依然小不下来，不及以前。- 速度 / 加速度插值有问题（线性插值加速度导致加速度太小了，估计哪里写挂了）。优化了插值方法以后变好了。
- [ ] 透传函数时不时阻塞一两百毫秒，检查出来透传到实机不管动不动都有 0.2% 几率阻塞 100~200ms，虚拟机没有此问题，换网线没用。
- [ ] 插值过程可能碰撞
- [ ] joint_states 同步到 moveit 场景中，增加一个机械臂，进行碰撞预测。
- [ ] 真前瞻
- [ ] 多种方案比较，论文向
