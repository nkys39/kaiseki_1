// File: include/es_slam/es_slam_node.hpp
#ifndef ES_SLAM_TEST_NODE_HPP
#define ES_SLAM_TEST_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <memory>
#include <vector>
#include <numeric>  // std::iota用
#include <algorithm>  // std::sort用
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>

#include "es_slam/slam_class.hpp"
#include "es_slam/scan_point_resampler.hpp"
#include "es_slam/point_types.hpp"
#include "es_slam/global_localization.hpp"
#include "es_slam/multi_resolution_map.hpp"

class EsSlamNode : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr genes_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr transformed_scan_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr resampled_scan_pub;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br;
    tf2::Transform base_to_laser;

    double last_x;
    double last_y;
    double last_yaw;
    rclcpp::Time last_time;

    sensor_msgs::msg::LaserScan scan_data;
    SLAM slam;
    ScanPointResampler scan_resampler;
    struct robot_position *robot_pos_s1;
    std::vector<std::vector<int>> map1;
    std::vector<std::vector<int>> map2;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;  // マップ購読用のサブスクライバを追加
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr current_map;  // 現在のマップを保持
    bool localization_only;   // localization onlyモードのフラグ
    bool received_first_map;  // 最初のマップを受信したかどうかのフラグを追加
    
    // 大域的自己位置推定関連
    std::unique_ptr<es_slam::GlobalLocalizationEngine> global_localization_engine_;
    std::unique_ptr<es_slam::GlobalLocalizationManager> global_localization_manager_;
    std::unique_ptr<es_slam::MultiResolutionMap> multi_resolution_map_;
    
    // 大域的自己位置推定のパブリッシャー
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr global_loc_status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr global_loc_candidates_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr global_loc_confidence_pub_;
    
    // 大域的自己位置推定のサブスクライバー
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    
    // 大域的自己位置推定パラメータ
    bool enable_initial_global_localization_;
    bool global_localization_done_;
    es_slam::GlobalLocalizationEngine::Parameters global_loc_params_;

private:
    int logodds_to_occupancy(double logodds);
    double probability_to_logodds(double prob);
    double logodds_to_probability(double logodds);

    void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan);
    void publishGenesPoseArray(const robot_position* robot);
    void processResampledScan(const sensor_msgs::msg::LaserScan& scan,
                             std::vector<LPoint2D>& resampled_points);
    void publishResampledScan(const std::vector<LPoint2D>& resampled_points,
                             const sensor_msgs::msg::LaserScan& original_scan);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &msg);
    
    // 大域的自己位置推定関連のメソッド
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg);
    void performGlobalLocalization(const sensor_msgs::msg::LaserScan& scan);
    void publishGlobalLocalizationStatus(const std::string& status);
    void publishGlobalLocalizationConfidence(double confidence);
    nav_msgs::msg::OccupancyGrid convertMapToOccupancyGrid();

public:
    EsSlamNode();
    ~EsSlamNode();

public:
    // 既存のパラメータ
    std::string m_base_frame;
    std::string m_odom_frame;
    std::string m_map_frame;
    std::string m_transformed_frame;
    int m_initial_map_size;
    int m_err;
    double m_map_rate;
    int m_t2;
    int m_ganm;

    // 対数オッズパラメータ
    bool use_logodds;
    double logodds_occ;
    double logodds_free;
    double logodds_max;
    double logodds_min;
    double logodds_thresh_occ;
    double logodds_thresh_free;

    // リサンプリングパラメータを追加
    double m_resample_distance_threshold;  // 点の距離間隔[m]
    double m_resample_length_threshold;    // 点の距離閾値[m]
    bool m_use_resampling;                // リサンプリングを使用するかどうか
};

#endif // ES_SLAM_NODE_HPP