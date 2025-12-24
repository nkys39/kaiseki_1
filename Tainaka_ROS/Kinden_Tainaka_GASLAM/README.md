
"Kinden_Tainaka_GASLAM"から取り出して、２つのワークスペースを"/home/user/"直下にそれぞれ配置してください
（"/home/user/ros1_ws"及び"/home/user/catkin_ws"となるように）
ros1_wsがGASLAM、catkin_wsがカートグラファーです


rosbagデータからGASLAM実行（初期位置推定なし）
ros1_ws/catkin_ws直下にbagファイルがあります

```sh
cd ros1_ws/catkin_ws
catkin_make
source devel/setup.bash

roslaunch gaslam_test rviz_gaslam-MIT.launch

rosbag play basescan_cmdvel_of_dataset1.bag

```

なお、
・"ros1_ws/catkin_ws/src/gaslam_test/src/gaslam_test_node.cpp"の12行目にある"bool applyReset = true;"
・"ros1_ws/catkin_ws/src/gaslam_test/src/slam_class.cpp"の6行目にある"bool apply = true;"
の2箇所をfalseに変更することで、照度測定ロボットの従来手法（2022年度終了時点の状態）に切り替えることができます


cartographerの実行
```sh
cd catkin_ws
catkin_make_isolated
source devel_isolated/cartographer_rviz/setup.bash

roslaunch '/home/user/catkin_ws/src/cartographer_launch/launch/cartographer_2d_kazuki.launch' 

```





