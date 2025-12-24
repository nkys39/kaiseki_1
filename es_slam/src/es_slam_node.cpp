#include "es_slam/es_slam_node.hpp"
#include <stdio.h>
#include <iostream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#define DEBUG_PRINT(var) std::cout<<#var<<" = "<<var<<std::endl;

// コンストラクタ
EsSlamNode::EsSlamNode() : Node("es_slam_node") {
    // パラメータの宣言と取得
    this->declare_parameter<std::string>("base_frame", "base_footprint");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("transformed_frame", "transformed_laser");
    this->declare_parameter<int>("initial_map_size", 1);
    this->declare_parameter<int>("err", -999999999);
    this->declare_parameter<double>("map_rate", 50.0);
    this->declare_parameter<int>("t2", 500);
    this->declare_parameter<int>("ganm", 100);
    this->declare_parameter<bool>("use_resampling", true);
    this->declare_parameter<double>("resample_distance_threshold", 0.05);
    this->declare_parameter<double>("resample_length_threshold", 0.25);
    this->declare_parameter<bool>("use_logodds", false);
    this->declare_parameter<double>("logodds_occ", 0.85);
    this->declare_parameter<double>("logodds_free", -0.4);
    this->declare_parameter<double>("logodds_max", 3.5);
    this->declare_parameter<double>("logodds_min", -2.0);
    this->declare_parameter<double>("logodds_thresh_occ", 0.6);
    this->declare_parameter<double>("logodds_thresh_free", -0.3);
    this->declare_parameter<bool>("localization_only", false);
    
    this->get_parameter("base_frame", m_base_frame);
    this->get_parameter("map_frame", m_map_frame);
    this->get_parameter("transformed_frame", m_transformed_frame);
    this->get_parameter("initial_map_size", m_initial_map_size);
    this->get_parameter("err", m_err);
    this->get_parameter("map_rate", m_map_rate);
    this->get_parameter("t2", m_t2);
    this->get_parameter("ganm", m_ganm);
    this->get_parameter("use_resampling", m_use_resampling);
    this->get_parameter("resample_distance_threshold", m_resample_distance_threshold);
    this->get_parameter("resample_length_threshold", m_resample_length_threshold);
    this->get_parameter("use_logodds", use_logodds);
    this->get_parameter("logodds_occ", logodds_occ);
    this->get_parameter("logodds_free", logodds_free);
    this->get_parameter("logodds_max", logodds_max);
    this->get_parameter("logodds_min", logodds_min);
    this->get_parameter("logodds_thresh_occ", logodds_thresh_occ);
    this->get_parameter("logodds_thresh_free", logodds_thresh_free);
    this->get_parameter("localization_only", localization_only);

    // パブリッシャーとサブスクライバーの初期化
    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 1, std::bind(&EsSlamNode::scanCallback, this, std::placeholders::_1));
    map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("slam_odom", 1);
    genes_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("slam_genes", 1);
    transformed_scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("transformed_scan", 1);

    // TFの初期化
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
    br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // SLAMの初期化
    slam.set_param(m_err, m_map_rate, m_t2);
    slam.set_ganm(m_ganm);
    
    if (use_logodds) {
        slam.set_logodds_param(
            logodds_occ,
            logodds_free,
            logodds_max,
            logodds_min,
            logodds_thresh_occ,
            logodds_thresh_free
        );
    }
    slam.enable_logodds(use_logodds);
    slam.set_localization_only(localization_only);

    // リサンプリングの設定
    if (m_use_resampling) {
        scan_resampler.setParameters(m_resample_distance_threshold, m_resample_length_threshold);
        resampled_scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("resampled_scan", 1);
    }

    // localization onlyモードの設定
    if (localization_only) {
        map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "static_map", 1, std::bind(&EsSlamNode::mapCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Running in localization only mode");
        
        // 大域的自己位置推定パラメータの宣言と読み込み
        this->declare_parameter("global_localization.enable_initial_estimation", true);
        this->declare_parameter("global_localization.search_range_xy", 2000.0);
        this->declare_parameter("global_localization.search_range_angle", 90.0);
        this->declare_parameter("global_localization.population_size", 1000);
        this->declare_parameter("global_localization.max_iterations", 100);
        this->declare_parameter("global_localization.fitness_threshold", 0.4);
        this->declare_parameter("global_localization.enable_scale_adjustment", false);
        
        this->get_parameter("global_localization.enable_initial_estimation", enable_initial_global_localization_);
        this->get_parameter("global_localization.search_range_xy", global_loc_params_.search_range_xy);
        this->get_parameter("global_localization.search_range_angle", global_loc_params_.search_range_angle);
        this->get_parameter("global_localization.population_size", global_loc_params_.population_size);
        this->get_parameter("global_localization.max_iterations", global_loc_params_.max_iterations);
        this->get_parameter("global_localization.fitness_threshold", global_loc_params_.fitness_threshold);
        this->get_parameter("global_localization.enable_scale_adjustment", global_loc_params_.enable_scale_adjustment);
        
        // 大域的自己位置推定の初期化
        global_localization_engine_ = std::make_unique<es_slam::GlobalLocalizationEngine>(this->get_logger());
        global_localization_engine_->setParameters(global_loc_params_);
        
        global_localization_manager_ = std::make_unique<es_slam::GlobalLocalizationManager>();
        multi_resolution_map_ = std::make_unique<es_slam::MultiResolutionMap>();
        
        // 大域的自己位置推定のパブリッシャー
        global_loc_status_pub_ = this->create_publisher<std_msgs::msg::String>("global_localization/status", 1);
        global_loc_candidates_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("global_localization/candidates", 1);
        global_loc_confidence_pub_ = this->create_publisher<std_msgs::msg::Float64>("global_localization/confidence", 1);
        
        // 初期位置推定のサブスクライバー
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 1, std::bind(&EsSlamNode::initialPoseCallback, this, std::placeholders::_1));
        
        global_localization_done_ = false;
    }

    // 変数の初期化
    last_x = 0.0;
    last_y = 0.0;
    last_yaw = 0.0;
    last_time = this->get_clock()->now();
    received_first_map = false;

    // ロボット位置の初期化
    robot_pos_s1 = slam.create_robot_position();
    slam.initialize_RobotPosition(robot_pos_s1, 0);

    // マップの初期化
    if (!localization_only) {
        map1 = std::vector<std::vector<int>>(m_initial_map_size, std::vector<int>(m_initial_map_size, 0));
        map2 = std::vector<std::vector<int>>(m_initial_map_size, std::vector<int>(m_initial_map_size, 0));
    }

    RCLCPP_INFO(this->get_logger(), "ES-SLAM Node initialized successfully");
}

EsSlamNode::~EsSlamNode() {
    if (robot_pos_s1) {
        delete robot_pos_s1;  // メモリ解放
        robot_pos_s1 = nullptr;
    }
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


// 対数オッズ関連の補助関数の実装
double EsSlamNode::probability_to_logodds(double prob) {
    if (prob < 0.001) prob = 0.001;
    if (prob > 0.999) prob = 0.999;
    return log(prob / (1.0 - prob));
}

double EsSlamNode::logodds_to_probability(double logodds) {
    return 1.0 - (1.0 / (1.0 + exp(logodds)));
}

int EsSlamNode::logodds_to_occupancy(double logodds) {
    double prob = logodds_to_probability(logodds);
    return static_cast<int>(prob * 100);
}

void EsSlamNode::publishGenesPoseArray(const robot_position* robot) {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = this->get_clock()->now();
    pose_array.header.frame_id = m_map_frame;

    // 現在のロボットの位置と角度を基準にする
    double robot_x = robot->real_x * 0.001; // mmをmに変換
    double robot_y = robot->real_y * 0.001;
    double robot_angle = robot->rangle * M_PI/180.0; // 度をラジアンに変換

    // 各遺伝子の姿勢を追加
    for(int i = 0; i < m_ganm; i++) {
        geometry_msgs::msg::Pose pose;
        
        // 遺伝子の相対位置を計算
        double dx = robot->gen_m[i][0] * 0.001; // mmをmに変換
        double dy = robot->gen_m[i][1] * 0.001;
        double dtheta = robot->gen_m[i][2] * M_PI/180.0; // 度をラジアンに変換

        // 遺伝子の位置をロボットの位置を基準に変換
        double gene_angle = robot_angle + dtheta;
        double gene_x = robot_x + (dx * cos(robot_angle) - dy * sin(robot_angle));
        double gene_y = robot_y + (dx * sin(robot_angle) + dy * cos(robot_angle));

        pose.position.x = gene_x;
        pose.position.y = gene_y;
        pose.position.z = 0.0;

        // 姿勢をクォータニオンに変換
        tf2::Quaternion q;
        q.setRPY(0, 0, gene_angle);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        pose_array.poses.push_back(pose);
    }

    // 最良の遺伝子を異なる色で表示できるように、最後に追加
    geometry_msgs::msg::Pose best_pose;
    double dx = robot->gen_m[robot->best_gene][0] * 0.001;
    double dy = robot->gen_m[robot->best_gene][1] * 0.001;
    double dtheta = robot->gen_m[robot->best_gene][2] * M_PI/180.0;

    double best_angle = robot_angle + dtheta;
    double best_x = robot_x + (dx * cos(robot_angle) - dy * sin(robot_angle));
    double best_y = robot_y + (dx * sin(robot_angle) + dy * cos(robot_angle));

    best_pose.position.x = best_x;
    best_pose.position.y = best_y;
    best_pose.position.z = 0.1; // 最良の遺伝子を少し上に表示

    tf2::Quaternion q;
    q.setRPY(0, 0, best_angle);
    best_pose.orientation.x = q.x();
    best_pose.orientation.y = q.y();
    best_pose.orientation.z = q.z();
    best_pose.orientation.w = q.w();

    pose_array.poses.push_back(best_pose);

    genes_pub->publish(pose_array);
}

void EsSlamNode::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan) {
    try {
        // localization onlyモードでマップが必要な場合のチェック
        if (localization_only && !received_first_map) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for map data...");
            return;
        }

        scan_data = *scan;
        rclcpp::Time current_time = this->get_clock()->now();
        
        // 大域的自己位置推定の初回実行
        if (localization_only && enable_initial_global_localization_ && 
            !global_localization_done_ && received_first_map &&
            global_localization_manager_ && global_localization_manager_->isReferenceMapInitialized()) {
            
            RCLCPP_INFO(this->get_logger(), "Performing initial global localization");
            performGlobalLocalization(scan_data);
        }

        // 変換されたLaserScanメッセージを作成
        sensor_msgs::msg::LaserScan transformed_scan = *scan;
        transformed_scan.header.frame_id = m_transformed_frame;
        transformed_scan.header.stamp = current_time;
        transformed_scan_pub->publish(transformed_scan);
        
        // レーザーのマウント位置を取得
        double mountX = 0.0;
        double mountY = 0.0;
        double mountYaw = 0.0;
        
        try {
            geometry_msgs::msg::TransformStamped transformStamped = 
                tf_buffer->lookupTransform(m_base_frame, scan_data.header.frame_id, tf2::TimePointZero);
                
            mountX = transformStamped.transform.translation.x;
            mountY = transformStamped.transform.translation.y;
            
            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);
            double roll, pitch;
            tf2::Matrix3x3(q).getRPY(roll, pitch, mountYaw);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF2 error: %s", ex.what());
            return;
        }

        if (m_use_resampling) {
            // スキャンデータをカートesian座標系に変換
            Scan2D scan2d;
            std::vector<double> x_points;
            std::vector<double> y_points;
            
            for (size_t i = 0; i < scan_data.ranges.size(); i++) {
                float range = scan_data.ranges[i];
                if (range >= scan_data.range_min && range <= scan_data.range_max) {
                    double angle = scan_data.angle_min + i * scan_data.angle_increment;
                    
                    // レーザー座標系での点の位置
                    double x = range * std::cos(angle);
                    double y = range * std::sin(angle);
                    
                    scan2d.lps.emplace_back(i, x, y);
                }
            }

            // リサンプリング実行
            scan_resampler.resamplePoints(&scan2d);

            // リサンプリングされた点群をSLAM用のデータに変換
            x_points.reserve(scan2d.lps.size());
            y_points.reserve(scan2d.lps.size());

            for (const auto& point : scan2d.lps) {
                x_points.push_back(point.x * 1000.0);  // mをmmに変換
                y_points.push_back(point.y * 1000.0);  // mをmmに変換
            }

            // SLAMにデータを渡す
            slam.set_laser_data_xy(
                x_points,
                y_points,
                scan_data.range_min * 1000.0,  // mをmmに変換
                scan_data.range_max * 1000.0,  // mをmmに変換
                mountX * 1000.0,               // mをmmに変換
                mountY * 1000.0,               // mをmmに変換
                mountYaw
            );

            // リサンプリングされた点群の可視化
            publishResampledScan(scan2d.lps, scan_data);

        } else {
            // リサンプリングなしの場合も、xy座標で渡す
            std::vector<double> x_points;
            std::vector<double> y_points;
            x_points.reserve(scan_data.ranges.size());
            y_points.reserve(scan_data.ranges.size());

            for(size_t i = 0; i < scan_data.ranges.size(); i++) {
                float range = scan_data.ranges[i];
                double angle = scan_data.angle_min + i * scan_data.angle_increment;
                
                x_points.push_back(range * std::cos(angle) * 1000.0);  // mをmmに変換
                y_points.push_back(range * std::sin(angle) * 1000.0);  // mをmmに変換
            }
            
            slam.set_laser_data_xy(
                x_points,
                y_points,
                scan_data.range_min * 1000.0,
                scan_data.range_max * 1000.0,
                mountX * 1000.0,
                mountY * 1000.0,
                mountYaw
            );
        }


        // SLAMの実行前にマップの状態を確認
        int unknown_count1 = 0, free_count1 = 0, occupied_count1 = 0;
        int unknown_count2 = 0, free_count2 = 0, occupied_count2 = 0;

        if (!map1.empty() && !map1[0].empty()) {
            for(size_t i = 0; i < map1.size(); i++) {
                for(size_t j = 0; j < map1[i].size(); j++) {
                    // map1の統計
                    if (map1[i][j] == 0) {
                        unknown_count1++;
                    } else if (map1[i][j] < 0) {
                        free_count1++;
                    } else {
                        occupied_count1++;
                    }
                    
                    // map2の統計
                    if (i < map2.size() && j < map2[i].size()) {
                        if (map2[i][j] == 0) {
                            unknown_count2++;
                        } else if (map2[i][j] < 0) {
                            free_count2++;
                        } else {
                            occupied_count2++;
                        }
                    }
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Before SLAM - Map1: Unknown=%d, Free=%d, Occupied=%d", 
                unknown_count1, free_count1, occupied_count1);
        RCLCPP_INFO(this->get_logger(), "Before SLAM - Map2: Unknown=%d, Free=%d, Occupied=%d", 
                unknown_count2, free_count2, occupied_count2);

        // robot_pos_s1がnullptrでないことを確認
        if (!robot_pos_s1) {
            RCLCPP_ERROR(this->get_logger(), "robot_pos_s1 is null!");
            return;
        }
        
        // 条件分岐を修正
        slam.slam(robot_pos_s1, map1, map2);
        
        // 遺伝子配列の公開
        publishGenesPoseArray(robot_pos_s1);
        
        if (!localization_only) {
            slam.map_building2(robot_pos_s1, map1, map2);

            // マップの公開
            nav_msgs::msg::OccupancyGrid og;
            og.info.resolution = m_map_rate * 0.001;  // mmをmに変換
            og.info.width = slam.get_current_map_width();
            og.info.height = slam.get_current_map_height();
            
            // 原点位置の計算
            double origin_x = -slam.get_map_center_x() * og.info.resolution;
            double origin_y = -slam.get_map_center_y() * og.info.resolution;
            
            og.info.origin.position.x = origin_x;
            og.info.origin.position.y = origin_y;
            og.info.origin.position.z = 0.0;
            og.info.origin.orientation.x = 0.0;
            og.info.origin.orientation.y = 0.0;
            og.info.origin.orientation.z = 0.0;
            og.info.origin.orientation.w = 1.0;
            
            og.data.resize(og.info.width * og.info.height);
            og.header.frame_id = m_map_frame;
            og.header.stamp = current_time;
            
            // マップデータの変換
            size_t cnt = 0;
            for(size_t j = 0; j < og.info.height; j++) {
                for(size_t i = 0; i < og.info.width; i++) {
                    int value = 0;
                    if (i < map1.size() && j < map1[i].size()) {
                        value = map1[i][j];
                    }
                    
                    if (use_logodds) {
                        if (value == 0) {
                            og.data[cnt] = -1;  // 未知領域
                        } else {
                            // 対数オッズ処理
                            double normalized = value / 100.0;
                            double prob = (normalized + 1.0) / 2.0;
                            double logodds = probability_to_logodds(prob);
                            og.data[cnt] = logodds_to_occupancy(logodds);
                        }
                    } else {
                        // 従来のロジック
                        if (value == 0) {
                            og.data[cnt] = -1;  // 未知領域
                        } else if (value > 0) {
                            og.data[cnt] = 100;  // 占有領域
                        } else {
                            og.data[cnt] = 0;    // 自由領域
                        }
                    }
                    cnt++;
                }
            }
            map_pub->publish(og);
        }

        // マップの公開（localization onlyモードでも公開）
        if (current_map) {
            nav_msgs::msg::OccupancyGrid og = *current_map;
            og.header.stamp = this->get_clock()->now();
            map_pub->publish(og);
        }

        // 現在の位置を計算
        double current_x = robot_pos_s1->real_x * 0.001;  // mmをmに変換
        double current_y = robot_pos_s1->real_y * 0.001;
        double current_yaw = robot_pos_s1->rangle * M_PI/180.0;  // 度をラジアンに変換

        // 速度を計算
        double dt = (current_time - last_time).seconds();
        double vx = (current_x - last_x) / dt;
        double vy = (current_y - last_y) / dt;
        double vyaw = (current_yaw - last_yaw) / dt;

        // Odometryメッセージの作成
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = m_map_frame;
        odom.child_frame_id = m_base_frame;

        // 位置をセット
        odom.pose.pose.position.x = current_x;
        odom.pose.pose.position.y = current_y;
        odom.pose.pose.position.z = 0.0;

        // 向きをクォータニオンでセット
        tf2::Quaternion q;
        q.setRPY(0, 0, current_yaw);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // 速度をセット
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vyaw;

        // 共分散行列をセット
        for(int i = 0; i < 36; i++) {
            odom.pose.covariance[i] = 0;
            odom.twist.covariance[i] = 0;
        }
        odom.pose.covariance[0] = 0.01;   // x位置の分散
        odom.pose.covariance[7] = 0.01;   // y位置の分散
        odom.pose.covariance[35] = 0.01;  // yaw角の分散
        odom.twist.covariance[0] = 0.01;  // x速度の分散
        odom.twist.covariance[7] = 0.01;  // y速度の分散
        odom.twist.covariance[35] = 0.01; // 角速度の分散

        // Odometryをパブリッシュ
        odom_pub->publish(odom);

        // TFをパブリッシュ
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = current_time;
        transformStamped.header.frame_id = m_map_frame;
        transformStamped.child_frame_id = m_base_frame;
        
        transformStamped.transform.translation.x = current_x;
        transformStamped.transform.translation.y = current_y;
        transformStamped.transform.translation.z = 0.0;
        
        transformStamped.transform.rotation = odom.pose.pose.orientation;
        
        br->sendTransform(transformStamped);

        // 現在の値を保存
        last_x = current_x;
        last_y = current_y;
        last_yaw = current_yaw;
        last_time = current_time;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in scanCallback: %s", e.what());
    }
}

void EsSlamNode::processResampledScan(const sensor_msgs::msg::LaserScan& scan,
                                    std::vector<LPoint2D>& resampled_points) {
    // スキャンデータをScan2D形式に変換
    Scan2D scan2d;
    
    for (size_t i = 0; i < scan.ranges.size(); i++) {
        if (scan.ranges[i] >= scan.range_min && scan.ranges[i] <= scan.range_max) {
            double angle = scan.angle_min + i * scan.angle_increment;
            double x = scan.ranges[i] * std::cos(angle);
            double y = scan.ranges[i] * std::sin(angle);
            scan2d.lps.emplace_back(i, x, y);
        }
    }

    // リサンプリング実行
    scan_resampler.resamplePoints(&scan2d);
    resampled_points = scan2d.lps;
}

// リサンプリングされたスキャンの可視化も修正
void EsSlamNode::publishResampledScan(const std::vector<LPoint2D>& resampled_points,
                                    const sensor_msgs::msg::LaserScan& original_scan) {
    sensor_msgs::msg::LaserScan resampled_scan = original_scan;
    resampled_scan.header.stamp = this->get_clock()->now();
    resampled_scan.header.frame_id = m_transformed_frame;
    
    // 点群を極座標系に変換
    std::vector<float> ranges(original_scan.ranges.size(), std::numeric_limits<float>::infinity());
    
    for (const auto& point : resampled_points) {
        // カートesian座標から極座標に変換
        double range = std::sqrt(point.x * point.x + point.y * point.y);
        double angle = std::atan2(point.y, point.x);
        
        // 角度インデックスを計算
        double normalized_angle = angle - original_scan.angle_min;
        if (normalized_angle < 0) normalized_angle += 2 * M_PI;
        if (normalized_angle > 2 * M_PI) normalized_angle -= 2 * M_PI;
        
        int index = static_cast<int>(normalized_angle / original_scan.angle_increment);
        if (index >= 0 && index < static_cast<int>(ranges.size())) {
            ranges[index] = static_cast<float>(range);
        }
    }
    
    resampled_scan.ranges = ranges;
    resampled_scan_pub->publish(resampled_scan);
}

void EsSlamNode::mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg) {
    try {
        current_map = msg;
        
        // マップのサイズチェック
        if (msg->info.width == 0 || msg->info.height == 0) {
            RCLCPP_WARN(this->get_logger(), "Received empty map");
            return;
        }

        // 変換前のmap1とmap2の統計を取る
        int unknown_count1 = 0, free_count1 = 0, occupied_count1 = 0;
        int unknown_count2 = 0, free_count2 = 0, occupied_count2 = 0;

        if (!map1.empty() && !map1[0].empty()) {
            for(size_t i = 0; i < map1.size(); i++) {
                for(size_t j = 0; j < map1[i].size(); j++) {
                    // map1の統計
                    if (map1[i][j] == 0) {
                        unknown_count1++;
                    } else if (map1[i][j] < 0) {
                        free_count1++;
                    } else {
                        occupied_count1++;
                    }
                    
                    // map2の統計
                    if (i < map2.size() && j < map2[i].size()) {
                        if (map2[i][j] == 0) {
                            unknown_count2++;
                        } else if (map2[i][j] < 0) {
                            free_count2++;
                        } else {
                            occupied_count2++;
                        }
                    }
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Map1 before conversion - Unknown: %d, Free: %d, Occupied: %d", 
                 unknown_count1, free_count1, occupied_count1);
        RCLCPP_INFO(this->get_logger(), "Map2 before conversion - Unknown: %d, Free: %d, Occupied: %d", 
                 unknown_count2, free_count2, occupied_count2);

        // 以下、既存の変換処理
        map1.clear();
        map2.clear();
        map1.resize(msg->info.width, std::vector<int>(msg->info.height, 0));
        map2.resize(msg->info.width, std::vector<int>(msg->info.height, 0));
        
        size_t cnt = 0;
        for(size_t j = 0; j < msg->info.height; j++) {
            for(size_t i = 0; i < msg->info.width; i++) {
                if (cnt < msg->data.size()) {
                    if (msg->data[cnt] == -1) {
                        map1[i][j] = 0;      // 未知領域
                        map2[i][j] = 0;
                    } else if (msg->data[cnt] == 0) {
                        map1[i][j] = -100;     // 自由領域
                        map2[i][j] = -100;
                    } else {
                        map1[i][j] = 100;      // 占有領域
                        map2[i][j] = 100;
                    }
                    cnt++;
                }
            }
        }
        
        received_first_map = true;
        
        // 大域的自己位置推定のための地図設定
        if (localization_only && global_localization_manager_) {
            global_localization_manager_->setReferenceMap(*msg);
            
            // マルチ解像度地図を作成
            if (multi_resolution_map_->createFromOccupancyGrid(*msg)) {
                RCLCPP_INFO(this->get_logger(), "Multi-resolution map created successfully");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to create multi-resolution map");
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Map received and converted successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in mapCallback: %s", e.what());
    }
}

// 大域的自己位置推定関連のメソッド実装
void EsSlamNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg) {
    if (!localization_only || !global_localization_manager_ || !global_localization_manager_->isReferenceMapInitialized()) {
        RCLCPP_WARN(this->get_logger(), "Cannot perform global localization: not in localization mode or no reference map");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Received initial pose, starting global localization");
    
    // 現在のレーザースキャンを使用して大域的自己位置推定を実行
    if (!scan_data.ranges.empty()) {
        performGlobalLocalization(scan_data);
    } else {
        RCLCPP_WARN(this->get_logger(), "No laser scan data available for global localization");
    }
}

void EsSlamNode::performGlobalLocalization(const sensor_msgs::msg::LaserScan& scan) {
    if (!global_localization_engine_ || !global_localization_manager_) {
        RCLCPP_ERROR(this->get_logger(), "Global localization engine not initialized");
        return;
    }
    
    publishGlobalLocalizationStatus("Starting global localization...");
    
    // 初期推定値を設定（マップ中心付近）
    geometry_msgs::msg::Pose2D initial_guess;
    initial_guess.x = 0.0;
    initial_guess.y = 0.0;
    initial_guess.theta = 0.0;
    
    // 現在のSLAM地図を取得
    nav_msgs::msg::OccupancyGrid current_slam_map = convertMapToOccupancyGrid();
    
    // 大域的自己位置推定を実行
    auto result = global_localization_engine_->performGlobalLocalization(
        es_slam::GlobalLocalizationEngine::MatchingMode::SCAN_TO_MAP,
        scan,
        global_localization_manager_->getReferenceMap(),
        current_slam_map,
        initial_guess
    );
    
    if (result.success) {
        RCLCPP_INFO(this->get_logger(), "Global localization succeeded with confidence %.3f", result.confidence);
        
        // SLAMシステムに推定位置を設定
        double estimated_x = result.estimated_pose.pose.pose.position.x;
        double estimated_y = result.estimated_pose.pose.pose.position.y;
        
        // クォータニオンからyaw角を計算
        tf2::Quaternion q(
            result.estimated_pose.pose.pose.orientation.x,
            result.estimated_pose.pose.pose.orientation.y,
            result.estimated_pose.pose.pose.orientation.z,
            result.estimated_pose.pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // ロボット位置を更新
        robot_pos_s1->real_x = estimated_x;
        robot_pos_s1->real_y = estimated_y;
        robot_pos_s1->rangle = yaw;
        
        publishGlobalLocalizationStatus("Global localization completed successfully");
        publishGlobalLocalizationConfidence(result.confidence);
        
        global_localization_done_ = true;
        
    } else {
        RCLCPP_WARN(this->get_logger(), "Global localization failed: %s", result.status_message.c_str());
        publishGlobalLocalizationStatus("Global localization failed");
        publishGlobalLocalizationConfidence(0.0);
    }
    
    // 候補位置を可視化用に公開
    auto candidates = global_localization_engine_->getCurrentPopulation();
    global_loc_candidates_pub_->publish(candidates);
}

void EsSlamNode::publishGlobalLocalizationStatus(const std::string& status) {
    std_msgs::msg::String msg;
    msg.data = status;
    global_loc_status_pub_->publish(msg);
}

void EsSlamNode::publishGlobalLocalizationConfidence(double confidence) {
    std_msgs::msg::Float64 msg;
    msg.data = confidence;
    global_loc_confidence_pub_->publish(msg);
}

nav_msgs::msg::OccupancyGrid EsSlamNode::convertMapToOccupancyGrid() {
    nav_msgs::msg::OccupancyGrid grid;
    
    if (map1.empty() || map2.empty()) {
        return grid;
    }
    
    // ヘッダー設定
    grid.header.stamp = this->get_clock()->now();
    grid.header.frame_id = m_map_frame;
    
    // 地図情報設定
    grid.info.resolution = m_map_rate / 1000.0;  // mm to m
    grid.info.width = map1.size();
    grid.info.height = map1[0].size();
    grid.info.origin.position.x = -(grid.info.width * grid.info.resolution) / 2.0;
    grid.info.origin.position.y = -(grid.info.height * grid.info.resolution) / 2.0;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    
    // データ変換
    grid.data.resize(grid.info.width * grid.info.height);
    
    for (size_t x = 0; x < grid.info.width; ++x) {
        for (size_t y = 0; y < grid.info.height; ++y) {
            int index = y * grid.info.width + x;
            
            if (use_logodds) {
                // 対数オッズから占有確率へ変換
                double prob = logodds_to_probability(map1[x][y]);
                grid.data[index] = static_cast<int8_t>(prob * 100);
            } else {
                // 直接変換
                if (map1[x][y] == 0) {
                    grid.data[index] = -1;  // 未知
                } else if (map1[x][y] < 0) {
                    grid.data[index] = 0;   // 自由
                } else {
                    grid.data[index] = 100; // 占有
                }
            }
        }
    }
    
    return grid;
}