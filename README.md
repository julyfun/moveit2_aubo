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

## full moveit

```
ros2 launch bringup cali.py
# windows: ./opencv-tcp.exe
# Calibrate.
ros2 launch moveit_config demo.launch.py 
colcon build .. && ros2 launch bringup moveit.py
```

## To show robot model

```
colcon build --symlink-install --packages-select aubo aubo_description
jst bs install/setup.bash
ros2 launch aubo launch.py
```

## todo:

- [x] angle interpolation seems wrong.
- [x] 电机发的是两圈的，注意内部处理。
- [x] 机械臂开机怎么开，更新机械臂驱动，加速 macsize 获取
- [ ] 仿真模式和真机模式都有很大延迟，奇怪的是，即使仿真时控制 mac 调小，延迟依然小不下来，不及以前。
