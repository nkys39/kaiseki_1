// slam_class.hpp
#ifndef SLAM_CLASS_HPP
#define SLAM_CLASS_HPP
#include "random_class.hpp"
#include "scan_point_resampler.hpp"

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <vector>

#define GALM 3

struct robot_position {
    int map_x;
    int map_y;
    double real_x;
    double real_y;
    double rangle;
    double dangle;
    double longitude;
    double latitude;
    
    std::vector<std::vector<double>> gen_m;
    std::vector<double> fit_m;
    int best_gene;
    int slam_count;

    // コンストラクタを追加
    robot_position(int ganm) {
        gen_m.resize(ganm + 1, std::vector<double>(GALM + 1, 0.0));
        fit_m.resize(ganm + 1, 0.0);
        best_gene = 0;
        slam_count = 0;
    }
};

class SLAM
{
private:
    int _err;
    double _map_rate;
    double _map_center_x;
    double _map_center_y;
    int _t2;
    int _ganm;
    int _map_expansion_size;
    int _current_map_width;
    int _current_map_height;

    std::vector<float> laser_x;
    std::vector<float> laser_y;
    Random random;

    bool _use_logodds;         // 対数オッズを使用するかどうか
    double _l_occ;             // 占有時の対数オッズ増分
    double _l_free;            // 非占有時の対数オッズ増分
    double _l_max;             // 最大対数オッズ値
    double _l_min;             // 最小対数オッズ値
    double _l_thresh_occ;      // 占有判定の閾値
    double _l_thresh_free;     // 非占有判定の閾値

    ScanPointResampler resampler;  // リサンプラーを追加
    bool _localization_only;  // localization onlyモードのフラグを追加
    
private:
    void transformation_Rmatrix(struct robot_position *sa1, struct robot_position *sa2, double R[][3], double tv[]);
    double fitcal_m(int i, struct robot_position *robot, std::vector<std::vector<int>> &omap, std::vector<std::vector<int>> &amap);
    int g_max(struct robot_position *robot, double *best_fitness);
    int g_min(struct robot_position *robot, double *worst_fitness);
    void mga_init(struct robot_position *robot, std::vector<std::vector<int>> &omap, std::vector<std::vector<int>> &amap);
    void map_mating(struct robot_position *robot, std::vector<std::vector<int>> &omap, std::vector<std::vector<int>> &amap);
    // ヘルパー関数
    double probability_to_logodds(double prob);
    double logodds_to_probability(double logodds);
    void calc_mapvalue(int k, int l, int pt_x, int pt_y, int sflag, std::vector<std::vector<int>> &omap, std::vector<std::vector<int>> &amap);
    // マップ拡張用の関数を追加
    void expand_map_if_needed(int required_x, int required_y, 
                            std::vector<std::vector<int>> &omap, 
                            std::vector<std::vector<int>> &amap);
    void expand_map(int new_width, int new_height,
                   std::vector<std::vector<int>> &omap, 
                   std::vector<std::vector<int>> &amap);
public:
    struct robot_position* create_robot_position();
    SLAM();
    ~SLAM();
    void set_param(int err, double map_rate, int t2);
    void set_ganm(int ganm); // GANMを設定する新しいメソッド
    int get_ganm() const { return _ganm; } // GANMを取得するメソッド
    void set_laser_data(long *lrf_range, int data_num, double angle_increment, double angle_min, double range_min, double range_max, double mountX, double mountY, double mountYaw);
    // 新しいxy点群用の関数を追加
    void set_laser_data_xy(
        const std::vector<double>& x_points,  // x座標[mm]
        const std::vector<double>& y_points,  // y座標[mm]
        double range_min,                     // 最小距離[mm]
        double range_max,                     // 最大距離[mm]
        double mountX,                        // マウント位置X[mm]
        double mountY,                        // マウント位置Y[mm]
        double mountYaw                       // マウント角度[rad]
    );
    void slam(struct robot_position *robot, std::vector<std::vector<int>> &omap, std::vector<std::vector<int>> &amap);
    void map_building2(struct robot_position *robot, std::vector<std::vector<int>> &omap, std::vector<std::vector<int>> &amap);
    double transformation_angle(double theta1);
    void initialize_RobotPosition(struct robot_position *robot, int dflag);

    void set_logodds_param(
        double l_occ,
        double l_free,
        double l_max,
        double l_min,
        double thresh_occ,
        double thresh_free
    );
    void enable_logodds(bool enable);

    // マップの拡張サイズを設定する関数を追加
    void set_map_expansion_size(int size) { _map_expansion_size = size; }
    // マップの現在のサイズを取得
    int get_current_map_width() const { return _current_map_width; }
    int get_current_map_height() const { return _current_map_height; }
    double get_map_center_x() const {
        return _map_center_x;
    }

    double get_map_center_y() const {
        return _map_center_y;
    }

    void set_laser_data_with_resampling(
        long* lrf_range, 
        int data_num,
        double angle_increment, 
        double angle_min,
        double range_min, 
        double range_max,
        double mountX, 
        double mountY, 
        double mountYaw
    );
    
    // リサンプラーのパラメータを設定するメソッドを追加
    void set_resampler_parameters(double distanceThreshold, double lengthThreshold);

    void set_localization_only(bool enable) { _localization_only = enable; }
    bool is_localization_only() const { return _localization_only; }
};

#endif