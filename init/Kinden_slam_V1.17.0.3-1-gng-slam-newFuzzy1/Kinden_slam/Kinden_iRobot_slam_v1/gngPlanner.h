/*
*  gngPlanner.h
*  Astar pathplanning based on gng
*
*  Created by Mahiro Watanabe on 24/02/09.
*
*/

#ifndef Astar_hpp
#define Astar_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>

#include "GNG.h"

struct Node {

    bool exist;                      //存在するか

    float pos_wide;
    float pos_hight;
    std::unordered_map<int, float> adjacent_map; //key:隣接ノードid, value:ノード間距離

    float occupancy = 0.5;
    float danger = 0;
    float accessibility = 1.0;

    Node() {
        pos_wide = 0;
        pos_hight = 0;
        exist = false;
    }

    Node(float wide, float hight) {
        pos_wide = wide;
        pos_hight = hight;
        exist = true;
    }

    Node(float wide, float hight, double occ) {
        pos_wide = wide;
        pos_hight = hight;
        exist = true;
        occupancy = occ;
    }

};

enum NodeType { NEW, OPEN, CLOSE };

struct CalcNode {
    int node_index;
    double fs = 0.0;
    double gs = 0.0;
    NodeType nodetype = NodeType::NEW;
    int parent_node_index = 0;

    // 他の必要なメンバ変数や関数
    CalcNode() {};

    CalcNode(int nodeID) {
        node_index = nodeID;
    }

};

class FastAstar {
public:
    Node nodes[MAX_NODE_NUM];
    //path
    std::vector<int> result_path; //ノードID
    bool draw_flag = false;       //経路探索結果描画用フラグ
    int current_target_index = 0;

private:
    //Pos
    int start_pos[2];
    int end_pos[2];
    int start_index;
    int end_index;
    float goal_distance;
    float danger_pos[2];
    float path_length = 0;

    //Neurons
    std::vector<int> active_neurons;  // 有効なノードのID
    int loop_count = 0;

    //計算
    // target>>candidateの移動にかかるコスト
    float _cost(const CalcNode& target, const CalcNode& candidate) {
        return nodes[target.node_index].adjacent_map[candidate.node_index] + nodes[candidate.node_index].danger * 1000 + (1.0 - this->nodes[candidate.node_index].accessibility) * 1000;
    }
    // 推定コスト
    float _hs(const CalcNode& node) const {
        return std::sqrt(std::pow(nodes[node.node_index].pos_wide - nodes[end_index].pos_wide, 2) + std::pow(nodes[node.node_index].pos_hight - nodes[end_index].pos_hight, 2));
    }


public:

    FastAstar(GNG* gng);
    ~FastAstar() {};

    //calc
    int FindNearestNode(int wide, int hight);
    void AddNodeDanger(std::vector<std::pair<int, double>> nodes_dangerous);
    void AddNodeAccessibility(std::vector<std::pair<int, double>> nodes_accessibility);

    //A*
    void Intialization(int start_x, int start_y, int goal_x, int goal_y);
    void AstarPathPlanningProcess();

    //draw
    void drawPathPlanningResult(IplImage* draw_Image);

private:

    float CalcEdgeLength(int index1, int index2);
};



#endif /* Astar_hpp */
