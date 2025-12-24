//
//  GNG.cpp
//  3DGNG
//
//  Created by KubotaLab on R 5/08/22.
//

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <vector>

#include "GNG.h"

#pragma mark -
#pragma mark GNG Processing

void GNG::LearningWithOccupancy(float _x, float _y, float _occupancy) {
    // Step.1 入力データv[]をp(v)を使ってランダムに取得する>>input_vector
    // Step.2 入力データvに対する勝者ノードs1と第二勝者ノードs2を取り出す
    float min_dis1 = 1e6, min_dis2 = 1e6;  // 距離の仮保存
    int s1_idx = -1, s2_idx = -1;     // 第一勝者ノード, 第二勝者ノード
    for (int i = 0; i < MAX_NODE_NUM; i++) {
        if (nodes[i].id == -1) continue;
        float dis = (_x - nodes[i].x) * (_x - nodes[i].x) + (_y - nodes[i].y) * (_y - nodes[i].y);
        if (dis < min_dis1) {
            min_dis2 = min_dis1; s2_idx = s1_idx;
            min_dis1 = dis; s1_idx = i;
        }
        else if (dis < min_dis2) {
            min_dis2 = dis; s2_idx = i;
        }
    }
    min_dis1 = sqrt(min_dis1);
    min_dis2 = sqrt(min_dis2);

    // Step.3 勝者ノードs1について入力データvとの事情誤差を積算誤差E_sに加算する
    nodes[s1_idx].error += min_dis1;
    total_error += min_dis1;

    // Step.4 ノードs1およびノードs2と結合関係あるノードと参照ベクトルを更新する, 占有度ベクトルを更新する
    nodes[s1_idx].x += LEARNRATE_S1 * (_x - nodes[s1_idx].x);
    nodes[s1_idx].y += LEARNRATE_S1 * (_y - nodes[s1_idx].y);
    nodes[s1_idx].occupancy += LEARNRATE_S1 * (_occupancy - nodes[s1_idx].occupancy);
    nodes[s2_idx].x += LEARNRATE_S2 * (_x - nodes[s2_idx].x);
    nodes[s2_idx].y += LEARNRATE_S2 * (_y - nodes[s2_idx].y);
    for (auto itr = nodes[s1_idx].edges.begin(); itr != nodes[s1_idx].edges.end(); ++itr) {
        int adjIdx = itr->first;
        nodes[adjIdx].x += LEARNRATE_S2 * (_x - nodes[adjIdx].x);
        nodes[adjIdx].y += LEARNRATE_S2 * (_y - nodes[adjIdx].y);
        // ついでにエッジの距離計算
        //        float d = (nodes[s1_idx].x - nodes[adjIdx].x)*(nodes[s1_idx].x - nodes[adjIdx].x)+(nodes[s1_idx].y - nodes[adjIdx].y)*(nodes[s1_idx].y - nodes[adjIdx].y);
        //        d = sqrt(d);
        //        nodes[s1_idx].edges[adjIdx].distance = d;
        //        nodes[adjIdx].edges[s1_idx].distance = d;
    }


    // Step.5 エッジの年齢を0にリセットする。またノードs1とs2の間にエッジが存在しなければ新たにエッジを作成する
    connect(s1_idx, s2_idx);

    // Step.6 ノードs1と結合関係のあるすべてのエッジの年齢をインクリメントする
    std::vector<int> delete_edges;
    for (auto itr = nodes[s1_idx].edges.begin(); itr != nodes[s1_idx].edges.end(); ++itr) {
        int adjIdx = itr->first;
        nodes[s1_idx].edges[adjIdx].age++;
        nodes[adjIdx].edges[s1_idx].age++;
        // Edgeの年齢による切断
        if (nodes[s1_idx].edges[adjIdx].age > MAXAGE) {
            delete_edges.push_back(adjIdx);
        }
    }
    // Step.7 事前に設定したしきい値a_maxを超える年齢のエッジを削除する
    for (int i = 0; i < (int)delete_edges.size(); i++) {
        disconnect(s1_idx, delete_edges[i]);
        if (nodes[delete_edges[i]].edges.size() == 0) { //孤立ノードの削除
            delete_node(delete_edges[i]);
        }
    }

    // Step.8 Step8 GNGへのデータ入力がlambda回ごとに次の操作をおこなう
    if (num_trial == lAMBDA) {
        num_trial = 0; //reset trial num
        // ノードが最大数でなければ、追加操作を行う
        if (num_of_node != MAX_NODE_NUM && total_error / lAMBDA > ERROR_THRESHOLD) {
            float max_err = 0.0;  // 最大積算誤差の値
            int max_err_index = -1;     // 最大積算誤差のノードID
            // 最大積算誤差の探索
            for (int i = 0; i < MAX_NODE_NUM; i++) {
                if (nodes[i].id == -1) continue;
                if (max_err < nodes[i].error) {
                    max_err = nodes[i].error;
                    max_err_index = i;
                }
            }
            // ノード追加
            AdditionPricessUsingNodeError(max_err_index);
        }
        update_value = total_error / lAMBDA;
        total_error = 0;
    }

    // Step.9 すべてのノードの積算誤差を減らす
    for (int i = 0; i < MAX_NODE_NUM; i++) {
        if (nodes[i].id == -1) continue;
        nodes[i].error -= BETA * nodes[i].error;
    }

    num_trial++;
    learning_count++;

}

#pragma mark -
#pragma mark GNG calculation

/* ノードを削除する関数 */
void GNG::delete_node(int idx) {
    nodes[idx].id = -1;
    num_of_node--;
}
/* ノードを追加する関数 */
int GNG::add_node(float x, float y) {
    gngNode new_node(x, y);
    // 空きノード探索
    for (int i = 0; i < MAX_NODE_NUM; i++) {
        if (nodes[i].id == -1) {
            new_node.id = i;
            nodes[i] = new_node;
            num_of_node++;
            return i;
        }
    }
    return -1; //空きがない場合
}
/* 占有度を引き継いでノードを追加する関数 */
int GNG::add_node_withOcc(float x, float y, float occ) {
    gngNode new_node(x, y);
    new_node.occupancy = occ;
    // 空きノード探索
    for (int i = 0; i < MAX_NODE_NUM; i++) {
        if (nodes[i].id == -1) {
            new_node.id = i;
            nodes[i] = new_node;
            num_of_node++;
            return i;
        }
    }
    return -1; //空きがない場合
}
/* エッジを切断する関数 */
void GNG::disconnect(int idx1, int idx2) {
    nodes[idx1].edges.erase(idx2);
    nodes[idx2].edges.erase(idx1);
}
/* ノード同士を接続する関数 */
void GNG::connect(int idx1, int idx2) {
    auto itr = nodes[idx1].edges.find(idx2);
    if (itr != nodes[idx1].edges.end()) { // すでに接続されている場合
        nodes[idx1].edges[idx2].age = 0;
        nodes[idx2].edges[idx1].age = 0;
    }
    else { //新しくエッジを作成
//        float d = (nodes[idx1].x - nodes[idx2].x)*(nodes[idx1].x - nodes[idx2].x) + (nodes[idx1].y - nodes[idx2].y)*(nodes[idx1].y - nodes[idx2].y);
//        d = sqrt(d);
//        gngNode::edge new_edge(0, d);
        gngNode::edge new_edge(0, 0);
        nodes[idx1].edges[idx2] = new_edge;
        nodes[idx2].edges[idx1] = new_edge;
    }
}
/* エッジ数が0のノードを削除する関数 */
void GNG::check_delete_no_edge() {
    for (int i = 0; i < MAX_NODE_NUM; i++) {
        if (nodes[i].id == -1) continue;
        if (nodes[i].edges.size() == 0)
            delete_node(i);
    }
}

#pragma mark -
#pragma mark Node Addition Processing

void GNG::AdditionPricessUsingNodeError(int node_idx) {
    // (i) node_idx: 積算誤差最大のノードq << node_idx

    // (ii) ノードqに接続されているノードで最も長いEdgeを持つノードfの探索: max_err_f
    int max_err_f = -1;
    float max_err = 0.0;
    for (auto itr = nodes[node_idx].edges.begin(); itr != nodes[node_idx].edges.end(); ++itr) {
        int adjIdx = itr->first;
        if (max_err < nodes[adjIdx].error) {
            max_err = nodes[adjIdx].error;
            max_err_f = adjIdx;
        }
    }

    // (iii) q-fを２分するノードrを追加: new_node_idx
    int new_node_idx = add_node_withOcc((float)((nodes[node_idx].x + nodes[max_err_f].x) * 0.5f), (float)((nodes[node_idx].y + nodes[max_err_f].y) * 0.5f), (float)((nodes[node_idx].occupancy + nodes[max_err_f].occupancy) * 0.5f));
    disconnect(node_idx, max_err_f);    // q-f切断
    connect(node_idx, new_node_idx);    // q-r接続
    connect(max_err_f, new_node_idx);   // r-f接続

    // (iv) ノードq,fの積算誤差更新
    nodes[node_idx].error -= ALFA * nodes[node_idx].error;
    nodes[max_err_f].error -= ALFA * nodes[max_err_f].error;

    // (v) ノードrの積算誤差更新
    nodes[new_node_idx].error = (nodes[node_idx].error + nodes[max_err_f].error) / 2.0;
}

#pragma mark -
#pragma mark Map fix

void GNG::ExtractionOfMovingArea() {
    //占有度が高いノードの削除
    for (int i = 0; i < MAX_NODE_NUM; i++) {
        if (nodes[i].id == -1) continue;
        if (nodes[i].occupancy > 0.38f) {
            this->delete_node(i);
        }
    }
}

#pragma mark -
#pragma mark draw

void GNG::drawGNG(cv::Mat draw_Image) {
    //draw node & edge
    cv::Point point, point2;
    for (int i = 0; i < MAX_NODE_NUM; i++) {
        if (nodes[i].id == -1) continue;
        point.x = nodes[i].x;
        point.y = nodes[i].y;
        double rgb_node = 0.7;
        double rgb_line = 0.8;
        //node
        cv::circle(draw_Image, point, 2, cv::Scalar((int)(255.0 * rgb_node), (int)(255.0 * rgb_node), (int)(255.0 * rgb_node)), -1, 8, 0);
        // text
        /*char txt_node;
        sprintf_s(&txt_node, "%f", nodes[i].occupancy);*/
        
        /*std::stringstream ss;
        ss << nodes[i].occupancy;
        std::string str = ss.str();
        cv::putText(draw_Image, str, cv::Point(point.x,point.y+1), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0));*/
        //edge
        for (auto itr = nodes[i].edges.begin(); itr != nodes[i].edges.end(); ++itr) {
            int adjIdx = itr->first;
            if (nodes[adjIdx].id == -1) continue;
            if (i > adjIdx) continue;
            point2.x = nodes[adjIdx].x;
            point2.y = nodes[adjIdx].y;
            cv::line(draw_Image, point, point2, cv::Scalar((int)(255.0 * rgb_line), (int)(255.0 * rgb_line), (int)(255.0 * rgb_line)), 1, 8, 0);
        }
    }

    //// text
    //char txt_loop;
    //sprintf(&txt_loop, "Learning time: %d", this->learning_count);
    //std::string str(&txt_loop);
    //cv::putText(draw_Image, str, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

    //char txt_neurons;
    //sprintf(&txt_neurons, "Number of nodes: %d", this->num_of_node);
    //std::string str1(&txt_neurons);
    //cv::putText(draw_Image, str1, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
}

#pragma mark -
#pragma mark public func

int GNG::getNearestNodeIndex(float _x, float _y) {
    int nodeIdx = 0;
    float distance = 1e6;
    for (int i = 0; i < MAX_NODE_NUM; i++) {
        if (nodes[i].id == -1) continue;
        float dis = (_x - nodes[i].x) * (_x - nodes[i].x) + (_y - nodes[i].y) * (_y - nodes[i].y);
        if (dis < distance) {
            distance = dis;
            nodeIdx = i;
        }
    }
    return nodeIdx;
}
