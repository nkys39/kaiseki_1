#include "gaslam_test_node.hpp"
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <stdexcept>
#include <cstring>
#include <jpeglib.h>

#define DEBUG_PRINT(var) std::cout<<#var<<" = "<<var<<std::endl;

bool applyReset = true;

// コンストラクタ
GaSlamNode::GaSlamNode()
{
    //サブスクライバーの定義
    scan_sub = nh.subscribe("/base_scan", 1, &GaSlamNode::scanCallback, this);
    cmdvel_sub = nh.subscribe("/teleop/cmd_vel", 1000, &GaSlamNode::cmdVelCallback, this);//
    //map_sub_ = nh_.subscribe("/map", 10, &GaSlamNode::mapCallback, this);//
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map",1);
}

GaSlamNode::~GaSlamNode()
{
}
//起動時の最初(パラメータ関連)
void GaSlamNode::onInitialize(){
    ROS_INFO("onInitialize");
} 
//起動時1度のみ(初期化用)
void GaSlamNode::onActivate(){
    ROS_INFO("onActivate");

    //事前地図画像読み込み
    cv::Mat image = cv::imread("/home/user/maps/map.jpg", cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        ROS_ERROR("Could not open or find the image");
        return;
    }
    ROS_INFO("premap read");

    ROS_INFO("image size: %d, %d", image.cols, image.rows);

    // slam._map_size_x = image.cols;
    // slam._map_size_y = image.rows;
    // slam._map_center_x = slam._map_size_x * 0.5;
    // slam._map_center_y = slam._map_size_y * 0.5;
    //ROS_INFO("slam: %d, %d", slam._map_size_x, slam._map_size_y);

    //パラメータの登録
    ros::NodeHandle nh_priv("~");		//ノードハンドルの宣言
    nh_priv.param<std::string>("base_frame", m_base_frame, "base_footprint");
    nh_priv.param<std::string>("odom_frame", m_odom_frame, "odom");
    nh_priv.param<std::string>("map_frame", m_map_frame, "map");

    // if(slam._map_size_x > slam._map_size_y){
    //     nh_priv.param<int>("map_size_x", m_map_size, slam._map_size_x);//
    // }
    // else{
    //     nh_priv.param<int>("map_size_y", m_map_size, slam._map_size_y);//
    // }

    //nh_priv.param<int>("map_size_x", m_map_size_x, slam._map_size_x);//
    //nh_priv.param<int>("map_size_y", m_map_size_y, slam._map_size_y);//

    nh_priv.param<int>("err", m_err, -999999999);
    nh_priv.param<double>("map_rate", m_map_rate, 50.0);//
    nh_priv.param<int>("t2", m_t2, 500);
    //ROS_INFO("pre set: %d, %d", slam._map_size_x, slam._map_size_y);
    slam.set_param(image.cols, image.rows, m_err, m_map_rate, m_t2);//
    ROS_INFO("parameter set");
    //ROS_INFO("set: %d, %d", slam._map_size_x, slam._map_size_y);

    //占有格子空間地図を作成
    map1 = malloc_2d(slam._map_size_y, slam._map_size_x);
    map2 = malloc_2d(slam._map_size_y, slam._map_size_x);
    slam.omap_CAD = malloc_2d(slam._map_size_y, slam._map_size_x);
    slam.amap_CAD = malloc_2d(slam._map_size_y, slam._map_size_x);
    ROS_INFO("map malloc_2d done");

    for (int y = 0; y < slam._map_size_y; ++y) {
        for (int x = 0; x < slam._map_size_x; ++x) {
            
            //ROS_INFO("%d", image.at<uchar>(y, x));
            if (image.at<uchar>(y, x) < 200) {
                map1[y][x] = 100; // 黒色の部分を占有度100とする
                map2[y][x] = 100; // 黒色の部分を占有度100とする
                slam.omap_CAD[y][x] = 100; // 黒色の部分を占有度100とする
                slam.amap_CAD[y][x] = 100; // 黒色の部分を占有度100とする
            } else {
                map1[y][x] = 0; // それ以外を占有度0とする
                map2[y][x] = 0; // それ以外を占有度0とする
                slam.omap_CAD[y][x] = 0; // それ以外を占有度0とする
                slam.amap_CAD[y][x] = 0; // それ以外を占有度0とする
            }
        }
    }
    ROS_INFO("occupancy grid map created");

    //check initial occupancy
    // for (int i = 0; i < slam._map_size_y; ++i) {
    //     std::stringstream ss;
    //     for (int j = 0; j < slam._map_size_x; ++j) {
    //         ss << map1[i][j] << " ";
    //     }
    //     ROS_INFO("Row %d: %s", i, ss.str().c_str());
    // }

    // slam推定状態の初期化
    robot_pos_s1 = NULL;
    robot_pos_s1 = (struct robot_position *)malloc(sizeof(struct robot_position));
    slam.initialize_RobotPosition(robot_pos_s1, 0);
}
void info_robot_position(robot_position *robot){
    DEBUG_PRINT(robot->real_x);
    DEBUG_PRINT(robot->real_y);
    DEBUG_PRINT(robot->map_x);
    DEBUG_PRINT(robot->map_y);
    DEBUG_PRINT(robot->rangle);
    DEBUG_PRINT(robot->dangle);
    DEBUG_PRINT(robot->longitude);
    DEBUG_PRINT(robot->latitude);
    DEBUG_PRINT(robot->best_gene);
    DEBUG_PRINT(robot->slam_count);
}
//実行中(メイン処理)
void GaSlamNode::onExecute(){
    ROS_INFO("onExecute");
}
//終了処理(メモリ解放等) 
void GaSlamNode::onDeactivate(){
    ROS_INFO("onDeactivate");
}

// cmd_velトピックのコールバック関数
// cmd_velからx方向の速度とy方向の速度とz軸周りの回転速度を求めてmodel[3]に格納
void GaSlamNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg){

    //ROS_INFO("modelCallBack");

    GaSlamNode::slam.vel[0] = msg->linear.x;
    GaSlamNode::slam.vel[1] = msg->linear.y;
    GaSlamNode::slam.vel[2] = msg->angular.z;
    //ROS_INFO("x_vel: %f, y_vel: %f, z_ang_vel: %f", GaSlamNode::slam.vel[0], GaSlamNode::slam.vel[1], GaSlamNode::slam.vel[2]);

}

// scanトピックのコールバック関数
void GaSlamNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan){	//サブスクライバ―のコールバック関数
    // ROS_INFO("scanCallback");
    // scanデータの整形
    scan_data=*scan;
    tf::StampedTransform ln_odom_to_laser;
    try
    {
        ln.lookupTransform(m_odom_frame.c_str(), scan_data.header.frame_id, ros::Time(0), ln_odom_to_laser);
        double roll, pitch, yaw;
        tf::Matrix3x3(ln_odom_to_laser.getRotation()).getRPY(roll, pitch, yaw);
        double x,y,z;
        x = ln_odom_to_laser.getOrigin().x();
        y = ln_odom_to_laser.getOrigin().y();
        z = ln_odom_to_laser.getOrigin().z();
        // std::cout << "x: " << ln_odom_to_laser.getOrigin().x() << std::endl;
        // std::cout << "y: " << ln_odom_to_laser.getOrigin().y() << std::endl;
        // std::cout << "z: " << ln_odom_to_laser.getOrigin().z() << std::endl;
        // std::cout << "roll: " << roll << std::endl;
        // std::cout << "pitch: " << pitch << std::endl;
        // std::cout << "yaw: " << yaw << std::endl;
        odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0,0,yaw),tf::Vector3(x,y,0.0));
    }
    catch (...)
    {
        ROS_INFO("tf1 error");
    }

    long length_data[(unsigned int)scan_data.ranges.size()];
    // DEBUG_PRINT(scan_data.ranges.size());
    for(int i=0;i<scan_data.ranges.size();i++){
        
        length_data[i] =  scan_data.ranges[i]*1000;//m⇒mm
    }
    slam.set_laser_data(
        length_data,
        (int)scan_data.ranges.size(),
        scan_data.angle_increment,
        scan_data.angle_min,
        scan_data.range_min*1000,
        scan_data.range_max*1000
        );

    //slam.set_model_data();

    ROS_INFO("slam count : %d", robot_pos_s1->slam_count);

    // localization
    //ROS_INFO("slam.slam execute");

    slam.compare_map(map1, map2);
    ROS_INFO("map_match_rate : %f", slam.map_match_rate);

    if (slam.map_match_rate < 0.9988 && applyReset == true) {//////////////////////////////////////////////////////////////////////////

		ROS_INFO("reset environment map");

		for (int i = 0; i < slam._map_size_y; i++) {
			for (int j = 0; j < slam._map_size_x; j++) {

				map1[i][j] = slam.omap_CAD[i][j];
				map2[i][j] = slam.amap_CAD[i][j];

			}
		}
        //robot_pos_s1->slam_count = 0;
	}

    slam.slam(robot_pos_s1,map1,map2);
    // std::cout<<"slam"<<std::endl;
    // info_robot_position(robot_pos_s1);
    // std::cout<<"map_building2"<<std::endl;

    ROS_INFO("best fitness = %f", robot_pos_s1->fit_m[robot_pos_s1->best_gene]);

    // mapping
    //ROS_INFO("slam.map_building2 execute");
    slam.map_building2(robot_pos_s1, map1, map2);

    // for (int i = 0; i < slam._map_size_y; ++i) {
    //     std::stringstream ss;
    //     for (int j = 0; j < slam._map_size_x; ++j) {
    //         ss << map1[i][j] << " ";
    //     }
    //     ROS_INFO("Row %d: %s", i, ss.str().c_str());
    // }

    // publish map
    nav_msgs::OccupancyGrid og;
    og.info.resolution = m_map_rate*0.001;
    og.info.width = slam._map_size_x;
    og.info.height = slam._map_size_y;
    og.info.origin.position.x = -slam._map_center_x*og.info.resolution;
    og.info.origin.position.y = -slam._map_center_y*og.info.resolution;
    og.info.origin.position.z = 0.0;
    og.info.origin.orientation.x = 0.0;
    og.info.origin.orientation.y = 0.0;
    og.info.origin.orientation.z = 0.0;
    og.info.origin.orientation.w = 0.0;
    og.data.resize(og.info.width*og.info.height);
    og.header.frame_id = m_map_frame.c_str();
    int cnt=0;
    for(int i=0;i<slam._map_size_y;i++){
        for(int j=0;j<slam._map_size_x;j++){
            if(map1[slam._map_size_y-j-1][slam._map_size_x-i-1]>0){
                // std::cout<<map1[i][j]<<", ";
                og.data[cnt]=100;
            }else if(map1[slam._map_size_y-j-1][slam._map_size_x-i-1]<0){
                og.data[cnt]=0;
            }else{
                og.data[cnt]=-1;
            }
            cnt++;
        }
    }
    map_pub.publish(og);

    // publish TF
    tf::Transform transform;
    tf::Transform laser_to_map;
    double mpose_x = robot_pos_s1->real_x*0.001;
    double mpose_y = robot_pos_s1->real_y*0.001;
    laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0,0,robot_pos_s1->rangle*M_PI/180),tf::Vector3(mpose_x,mpose_y,0.0));
    transform = (odom_to_laser * laser_to_map.inverse()).inverse();
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), m_map_frame.c_str(), m_odom_frame.c_str()));
}

// 2次元ベクトルを確保する関数
// std::vector<std::vector<long long int>> create2DVector(int rows, int cols) {
//     return std::vector<std::vector<long long int>>(rows, std::vector<long long int>(cols));
// }

// 2次元配列を動的に割り当てる関数
int** malloc_2d(int rows, int cols) {
    int** array = (int**)malloc(rows * sizeof(int*));
    for (int i = 0; i < rows; ++i) {
        array[i] = (int*)malloc(cols * sizeof(int));
    }
    if (array == NULL) {
        ROS_INFO("NULL array");
    }

    return array;
}

// 2次元配列を解放する関数
void free_2d(int** array, size_t rows) {
    for (size_t i = 0; i < rows; ++i) {
        free(array[i]);
    }
    free(array);
}

