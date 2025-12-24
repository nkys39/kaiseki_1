/*
*  gngPlanner.cpp
*  Astar pathplanning based on gng
*
*  Created by Mahiro Watanabe on 24/02/09.
*
*/

#include "gngPlanner.h"

FastAstar::FastAstar(GNG* gng) {
    // ノード配列の初期化
    Node init_node = Node();
    for (int i = 0; i < MAX_NODE_NUM; i++) {
        this->nodes[i] = init_node;
    }

    // gng位相構造の取得
    float wide, hight, occ;
    for (int i = 0; i < MAX_NODE_NUM; i++) {
        if (gng->nodes[i].id == -1) continue;
        wide = gng->nodes[i].x;
        hight = gng->nodes[i].y;
        occ = gng->nodes[i].occupancy;
        Node init_node = Node(wide, hight, occ);
        this->nodes[i] = init_node;
        this->active_neurons.push_back(i);
    }
    for (int i = 0; i < (int)this->active_neurons.size(); i++) {
        int index = this->active_neurons[i];
        // エッジの長さを保存
        for (auto itr = gng->nodes[index].edges.begin(); itr != gng->nodes[index].edges.end(); ++itr) {
            int adjIdx = itr->first;
            if (gng->nodes[adjIdx].id == -1) continue;
            if (index > adjIdx) continue;
            float dis = this->CalcEdgeLength(index, adjIdx);
            this->nodes[index].adjacent_map[adjIdx] = dis;
            this->nodes[adjIdx].adjacent_map[index] = dis;
        }
    }
}

void FastAstar::Intialization(int start_x, int start_y, int goal_x, int goal_y) {
    this->start_pos[0] = start_x;
    this->start_pos[1] = start_y;
    this->end_pos[0] = goal_x;
    this->end_pos[1] = goal_y;

    this->goal_distance = (this->end_pos[0] - this->start_pos[0]) * (this->end_pos[0] - this->start_pos[0]) + (this->end_pos[1] - this->start_pos[1]) * (this->end_pos[1] - this->start_pos[1]);
    this->goal_distance = sqrt(this->goal_distance);

    this->start_index = this->FindNearestNode(this->start_pos[0], this->start_pos[1]);
    this->end_index = this->FindNearestNode(this->end_pos[0], this->end_pos[1]);

    printf("Start: (%d,%d), End: (%d,%d)\n", start_x, start_y, goal_x, goal_y);
    printf("Start node: %d, End node: %d\n", this->start_index, this->end_index);
    result_path.clear();
}

int FastAstar::FindNearestNode(int wide, int hight) {
    int nearest_nodeId = 0;
    float min_dis = 1e6;
    float n_w, n_h, dis;
    for (int i = 0; i < (int)this->active_neurons.size(); i++) {
        int index = this->active_neurons[i];
        n_w = this->nodes[index].pos_wide;
        n_h = this->nodes[index].pos_hight;
        dis = (n_w - wide) * (n_w - wide) + (n_h - hight) * (n_h - hight);
        if (min_dis > dis) {
            min_dis = dis;
            nearest_nodeId = index;
        }
    }
    return nearest_nodeId;
}

float FastAstar::CalcEdgeLength(int index1, int index2) {
    return sqrt((this->nodes[index1].pos_wide - this->nodes[index2].pos_wide) * (this->nodes[index1].pos_wide - this->nodes[index2].pos_wide) + (this->nodes[index1].pos_hight - this->nodes[index2].pos_hight) * (this->nodes[index1].pos_hight - this->nodes[index2].pos_hight));
}

void FastAstar::AddNodeDanger(std::vector<std::pair<int, double>> nodes_dangerous) {
    for (int i = 0; i < (int)nodes_dangerous.size(); i++) {
        this->nodes[nodes_dangerous[i].first].danger = nodes_dangerous[i].second;
    }
}

void FastAstar::AddNodeAccessibility(std::vector<std::pair<int, double>> nodes_accessibility) {
    for (int i = 0; i < (int)nodes_accessibility.size(); i++) {
        this->nodes[nodes_accessibility[i].first].accessibility = nodes_accessibility[i].second;
    }
}

//A*
void FastAstar::AstarPathPlanningProcess() {

    std::unordered_map<int, CalcNode> table; //key:ノードID, value:対象ノードの評価値
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<>> OPENList; //OPEN

    CalcNode targetNode;

    //step3
    CalcNode startCalcNode = CalcNode(start_index);
    startCalcNode.fs = _hs(startCalcNode); // f(s)=h(s)
    table[startCalcNode.node_index] = startCalcNode;
    OPENList.push({ startCalcNode.fs, startCalcNode.node_index });

    bool end_state = true;
    while (end_state) {
        //step4
        if (OPENList.empty()) {
            std::cout << "No route found" << std::endl;
            end_state = false;
            break;
        }

        //step5
        targetNode = table[OPENList.top().second];

        OPENList.pop();

        if (targetNode.nodetype == CLOSE) {
            continue;
        }

        //step6
        if (targetNode.node_index == end_index) {
            end_state = false;
            break;
        }

        targetNode.nodetype = CLOSE;
        targetNode.gs = targetNode.fs - _hs(targetNode);
        table[targetNode.node_index] = targetNode;  //table内のtarget更新

        //step7 隣接ノードを次のパス候補として計算
        for (auto itr = this->nodes[targetNode.node_index].adjacent_map.begin(); itr != this->nodes[targetNode.node_index].adjacent_map.end(); ++itr) {
            int adjIdx = itr->first;
            CalcNode candidate;
            auto it = table.find(adjIdx);
            if (it != table.end()) {
                candidate = table[adjIdx];
            }
            else {
                candidate = CalcNode(adjIdx);
            }
            float fsd = targetNode.gs + _cost(targetNode, candidate) + _hs(candidate);

            if (candidate.nodetype == NEW) {//tableにない
                candidate.fs = fsd;
                candidate.parent_node_index = targetNode.node_index;
                candidate.nodetype = OPEN;
                table[candidate.node_index] = candidate;
                OPENList.push({ candidate.fs, candidate.node_index });
            }
            else {//tableにある
                if (fsd < candidate.fs) {
                    candidate.fs = fsd;
                    candidate.parent_node_index = targetNode.node_index;
                    candidate.nodetype = OPEN;
                    table[candidate.node_index] = candidate;
                    OPENList.push({ candidate.fs, candidate.node_index });
                }
            }
        }
    }

    for (CalcNode* ptr = &targetNode; ptr != nullptr; ptr = &table[ptr->parent_node_index]) {
        result_path.push_back(ptr->node_index);
        if (ptr->node_index == this->start_index) break;
    }

    std::reverse(result_path.begin(), result_path.end());

    for (int i = 0; i < (int)result_path.size() - 1; i++) {
        this->path_length += this->nodes[result_path[i]].adjacent_map[result_path[i + 1]];
    }
    printf("path length: %f\n", this->path_length);
    draw_flag = true;
}

#pragma mark -
#pragma mark draw

void FastAstar::drawPathPlanningResult(IplImage* draw_Image) {
    //start, end
    cv::Point pt;
    pt.x = this->start_pos[0];
    pt.y = this->start_pos[1];
    cvCircle(draw_Image, cvPoint(pt.x, pt.y), 5, CV_RGB(200, 0, 0), -1, 8, 0);
    pt.x = this->end_pos[0];
    pt.y = this->end_pos[1];
    cvCircle(draw_Image, cvPoint(pt.x, pt.y), 5, CV_RGB(200, 0, 0), -1, 8, 0);

    //draw node & edge
    cv::Point point, point2;
    for (int i = 0; i < (int)result_path.size() - 1; i++) {

        point.x = this->nodes[result_path[i]].pos_wide;
        point.y = this->nodes[result_path[i]].pos_hight;
        //node
        cvCircle(draw_Image, cvPoint(point.x, point.y), 4, CV_RGB(0, 0, 200), -1, 8, 0);
        //edge
        point2.x = this->nodes[result_path[i + 1]].pos_wide;
        point2.y = this->nodes[result_path[i + 1]].pos_hight;
        cvLine(draw_Image, cvPoint(point.x, point.y), cvPoint(point2.x, point2.y), CV_RGB(0, 0, 150), 1, 8, 0);
    }

    point.x = this->nodes[result_path.back()].pos_wide;
    point.y = this->nodes[result_path.back()].pos_hight;
    //node
    cvCircle(draw_Image, cvPoint(point.x, point.y), 4, CV_RGB(0, 0, 100), -1, 8, 0);

    point.x = this->nodes[result_path[current_target_index]].pos_wide;
    point.y = this->nodes[result_path[current_target_index]].pos_hight;
    //node
    cvCircle(draw_Image, cvPoint(point.x, point.y), 4, CV_RGB(200, 200, 0), -1, 8, 0);

}
