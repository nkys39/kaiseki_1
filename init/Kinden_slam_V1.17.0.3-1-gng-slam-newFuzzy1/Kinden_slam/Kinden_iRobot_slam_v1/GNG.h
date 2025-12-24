//
//  GNG.hpp
//  3DGNG
//
//  Created by KubotaLab on R 5/08/22.
//

#ifndef GNG_h
#define GNG_h

#include <iostream>
#include <stdio.h>
#include <vector>
#include <unordered_map>
#include <opencv2/opencv.hpp>

#define LEARNRATE_S1    0.08 // 0.08
#define LEARNRATE_S2    0.008 // 0.008
#define BETA            0.005
#define ALFA            0.5

#define MAX_NODE_NUM    4000     // 最大ノード数
#define NUMOFVALUE      2       // ベクトルの次元数
#define EDGELIFE        13
#define LAMBLA_VALUE    11
#define lAMBDA          30     // 1周期のデータ使用数 300
#define NUMOFUSINGDATA  5000
#define MAXAGE          88
#define ERROR_THRESHOLD 4 //10

class gngNode {
public:
    int id = -1;            // ノードid
    float x;
    float y;
    float error = 1.0;      // 積算誤差
    int age = 0;            // ノードの年齢（）
    float occupancy = 0.5f;  // 占有度

    struct edge {
        int age = 0;            // Edgeの年齢
        float distance = 0.0;   // Edgeの長さ

        edge() {};
        edge(int _age, float _d) : age(_age), distance(_d) {};
    };

    std::unordered_map<int, edge> edges; // Edgeの集合 key:隣接ノードid, value:edge

protected:
public:
    gngNode() {};
    gngNode(float _x, float _y) : x(_x), y(_y) {};
    ~gngNode() {};
};

class GNG {
public:
    gngNode nodes[MAX_NODE_NUM];
    int num_of_node = 0;
private:
    int num_trial = 0;  // 試行回数
    int learning_count = 0;
    std::vector<std::string> linesname; //描画用
    float total_error = 0;
    float update_value = 1e6;

public:
    GNG() {
        int f_idx = add_node(500.0, 200.0);
        int s_idx = add_node(600.0, 300.0);
        nodes[f_idx].id = f_idx;
        nodes[s_idx].id = s_idx;
        connect(f_idx, s_idx);
    }
    ~GNG() {}

    void LearningWithOccupancy(float _x, float _y, float _occupancy);           /* 学習 */
    void drawGNG(cv::Mat draw_Image);                                           /* 描画 */
    void ExtractionOfMovingArea();                                              /* 占有度の高いノードを削除 */

private:
    /* ノードを削除する関数 */
    void delete_node(int idx);
    /* ノードを追加する関数 */
    int add_node(float x, float y);
    /* 占有度を引き継いでノードを追加する関数 */
    int add_node_withOcc(float x, float y, float occ);
    /* エッジを切断する関数 */
    void disconnect(int idx1, int idx2);
    /* ノード同士を接続する関数 */
    void connect(int idx1, int idx2);
    /* エッジ数が0のノードを削除する関数 */
    void check_delete_no_edge();
    /* 積算誤差でノードを追加する関数 */
    void AdditionPricessUsingNodeError(int node_idx);

public:
    int getNearestNodeIndex(float _x, float _y);

};


#endif /* GNG_hpp */
