#ifndef SLAM_CLASS_HPP
#define SLAM_CLASS_HPP
#include "random_class.hpp"

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>

#include <vector>
#include <opencv2/opencv.hpp>

// #define MAPSIZE 1000 
// #define ERR -999999999
// #define MAPRATE 50.0	
// #define CENTER MAPSIZE/2
#define GALM 3
#define GANM 100

struct robot_position{
    int map_x;
    int map_y;
    double real_x;
    double real_y;
    double rangle;
    double dangle;
    
    double longitude;
    double latitude;
    
    double gen_m[GANM+1][GALM+1];
    double fit_m[GANM+1];
    int best_gene;
    int slam_count;
};

class SLAM
{
private:
    int _map_size;
    int _err;
    double _map_rate;
    int _map_center;
    int _t2;
    std::vector<float> laser_x;
    std::vector<float> laser_y;
    Random random;
    // std::vector<double> in_laser_data;
    bool modelmode = true;
private:
    // void transformation_Rmatrix(struct robot_position *sa1, struct robot_position *sa2, double R[][3], double tv[]);
    // // Caluculate localization of the robot by using SSGA. Created by Yuichiro Toda.
    // double fitcal_m(int i, struct robot_position *robot,int** &omap, int** &amap);
    // double fitcal_MIX(int i, struct robot_position* robot, int** omap, int** amap);

    // int g_max(struct robot_position *robot, double *best_fitness);
    // int g_min(struct robot_position *robot, double *worst_fitness);
    // void mga_init(struct robot_position *robot, int** &omap, int** &amap);
    // void map_mating(struct robot_position *robot, int** &omap, int** &amap);
    // void calc_mapvalue(int k, int l, int pt_x, int pt_y, int sflag, int** &omap, int** &amap);
    //void calcRobotModel(double model[3]);

public:
    SLAM();//コンストラクタ
    ~SLAM();//デストラクタ
    void transformation_Rmatrix(struct robot_position *sa1, struct robot_position *sa2, double R[][3], double tv[]);
    // Caluculate localization of the robot by using SSGA. Created by Yuichiro Toda.
    double fitcal_m(int i, struct robot_position *robot,int** &omap, int** &amap);

    double fitcal_MIX(int i, struct robot_position* robot, int** &omap, int** &amap);
    void compare_map(int** omap, int** amap);

    int g_max(struct robot_position *robot, double *best_fitness);
    int g_min(struct robot_position *robot, double *worst_fitness);
    void mga_init(struct robot_position *robot, int** &omap, int** &amap);
    void map_mating(struct robot_position *robot, int** &omap, int** &amap);
    void calc_mapvalue(int k, int l, int pt_x, int pt_y, int sflag, int** &omap, int** &amap);
    //void calcRobotModel(double model[3])

    int **omap_CAD;
    int **amap_CAD;
    double map_match_rate = 1;

    int _map_size_x;//
    int _map_size_y;//
    int _map_center_x;//
    int _map_center_y;//
    double vel[3] = {0.0, 0.0, 0.0}; //cmd_velトピックのデータを格納
    double model[3] = {0.0, 0.0, 0.0};//運動学モデルの移動量を格納
    void set_param(int map_size_x, int map_size_y, int err, double map_rate,int t2);
    void set_laser_data(long *lrf_range,int data_num,double angle_increment,double angle_min,double range_min,double range_max);
    void slam(struct robot_position *robot, int** &omap, int** &amap);
    void map_building2(struct robot_position *robot, int** &omap, int** &amap);
    double transformation_angle(double theta1);
    void initialize_RobotPosition(struct robot_position *robot, int dflag);

public:
    
    
};

#endif