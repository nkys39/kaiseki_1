
// ROS
#include <ros/ros.h>//ROSの基本ヘッダーファイル
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


#include "slam_class.hpp"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <stdexcept>
#include <cstring>
#include <jpeglib.h>

//std::vector<std::vector<long long int>> create2DVector(int rows, int cols);
int** malloc_2d(int rows, int cols);
void free_2d(int** array, size_t rows);

class GaSlamNode
{
private:
    ros::NodeHandle nh;			//ノードハンドルの宣言
    ros::Subscriber scan_sub;//Laser_Scan型のサブスクライブ
    ros::Subscriber cmdvel_sub;//cmd_velのサブスクライブ
    //ros::Subscriber map_sub;//map_serverからのmapのサブスクライブ
    ros::Publisher map_pub;
    tf::TransformListener ln;
    tf::TransformBroadcaster br;
    tf::Transform odom_to_laser;

    sensor_msgs::LaserScan scan_data;//最新のscanトピックデータが格納される
    // bool scanUpdated;//scanトピックが更新された時True,処理された時falseになる

    SLAM slam;
    struct robot_position *robot_pos_s1;
    int **map1;
    int **map2;
    //std::vector<std::vector<int>> map1;//
    //std::vector<std::vector<int>> map2;//
private:
    // scanトピックのコールバック関数
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);//
    //void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);//
public:
    GaSlamNode();//コンストラクタ
    ~GaSlamNode();//デストラクタ

    void onInitialize(); //起動時の最初(パラメータ関連)
    void onActivate();   //起動時1度のみ(初期化用)
    void onExecute();    //実行中(メイン処理)
    void onDeactivate(); //終了処理(メモリ解放等) 

public:
    //パラメータ 
    std::string m_base_frame;
    std::string m_odom_frame;
    std::string m_map_frame;
    int m_map_size;
    //int m_map_size_x;//
    //int m_map_size_y;//
    int m_err;
    double m_map_rate;
    int m_t2;
};


