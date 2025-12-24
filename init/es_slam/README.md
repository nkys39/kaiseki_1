# es_slam package
## 1.package description
進化戦略に基づく2次元SLAM
## 2.node
### 2.1.es_slam_node
#### 2.1.1.Subscribed Topics
- ~scan:(sensor_msgs/LaserScan)
- /tf:(tf/tfMessage)
#### 2.1.2.Published Topics
- ~map:(nav_msgs/OccupancyGrid)
- /tf:(tf/tfMessage)
#### 2.1.3.Services
#### 2.1.4.Action
#### 2.1.5.Parameters

- base_frame (str,default="base_footprint")
  - 未使用(オドメトリ融合時に使用)
- odom_frame (str, default="odom")
  - オドメトリ原点座標のTFフレームID
- map_frame (str,default="map")
  - 地図原点座標のTFフレームID
- map_size (int, default=1000)
  - 地図の大きさ(単位：グリッドの数)(自動判定を実装した場合不要)
- err (int, default=-999999999)
- map_rate (double, default=50.0)
  - 地図の解像度(単位：mm)(元の実装がmm単位で行っていたため、mmになっているが、SI単位系のmに直したほうが良い)
- t2 (int, default=500)

#### 2.1.6.Required tf Transforms
[サブスクライブしている/scanトピックのframe_id]→[odom]
#### 2.1.7.Provided tf Transforms
[map] → [odom]


## 3.Operation check method
Noetic版のturtlebo3_simulationを用いた実行方法
### 3.1.install dependencies
```sh
sudo apt install ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3 ros-noetic-turtlebot3-bringup ros-noetic-turtlebot3-slam ros-noetic-turtlebot3-gazebo ros-noetic-navigation
```

### 3.2.Execution command
turtlebo3_simulationの起動
```sh
export TURTLEBOT3_MODEL=burger
roslaunch es_slam turtlebot3_sim.launch
```
es_slamの起動
```sh
roslaunch es_slam es_slam.launch
```
速度指令の配信
```sh
# ゲームパッドの場合(1番ボタンを押しながら左スティック)
roslaunch es_slam joy_teleop.launch
# turtlebo3のキーボードteleopの場合
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
地図の保存
```sh
rosrun map_server map_saver
```