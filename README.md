# 自律走行で使用するワーキングスペース
## 使用方法
### GPSでcsvファイルでのウェイポイント付近まで行きたいとき
1. 以下のコマンドをターミナルで実行する
```
cd ~/autonav_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch autonav launch_sim.launch.py
```
2. 新しいターミナルを開き
```
source install/setup.bash
ros2 run autonav nav_gate_gps_csv
```
を開くと、近い場所から順にローバーが回っていくようになる

### 2つのArucoマーカの間を通りたいとき
1. 以下のコマンドをターミナルで実行する
```
cd ~/autonav_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch autonav launch_sim.launch.py
```
2. 新しいターミナルを開き
```
source install/setup.bash
ros2 run opencv_ros2 aruco_node_tf --ros-args   -r /image_raw:=/camera/image_raw   -r /camera_info:=/camera/camera_info   -p marker_size:=2.00   -p camera_frame:="camera_link_optical"   -p use_sim_time:=true
```
を実行し、Arucoマーカ検出が始まる. markersizeは状況によって変更する.
3. 更に新しいターミナルを開き
```
source install/setup.bash
ros2 run autonav all_gate_follower
```
を実行すると旋回してマーカを探し、２つ見つかったら中点を通るように動くようになる.