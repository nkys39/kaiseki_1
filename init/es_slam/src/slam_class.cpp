#include "es_slam/slam_class.hpp"

// slam_class.cpp の変更部分
SLAM::SLAM() {
    _map_expansion_size = 1;  // デフォルトの拡張サイズ
    _current_map_width = 1;  // 初期サイズ
    _current_map_height = 1;
    _map_center_x = _current_map_width / 2.0;
    _map_center_y = _current_map_height / 2.0;
    _err = -999999999;
    _map_rate = 50.0;
    _t2 = 500;
    _ganm = 100;
    
    _use_logodds = false;
    _l_occ = 0.2;
    _l_free = -0.1;
    _l_max = 5.0;
    _l_min = -5.0;
    _l_thresh_occ = 0.5;
    _l_thresh_free = -0.5;
    _localization_only = false;
}
SLAM::~SLAM(){

}
// robot_position作成用の新しいメソッド
struct robot_position* SLAM::create_robot_position() {
    return new robot_position(_ganm);
}
// private
void SLAM::transformation_Rmatrix(struct robot_position *sa1, struct robot_position *sa2, double R[][3], double tv[])
{
    double roll = sa1->rangle/180.0*M_PI;
    double pitch = 0.0;
    double yaw = sa1->rangle/180.0*M_PI;
    
    
    tv[0] = sa1->real_x*0.001;
    tv[1] = sa2->real_x*0.001;
    tv[2] = sa1->real_y*0.001;
    
    R[0][0] = cos(roll)*cos(pitch), R[0][1] = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*sin(yaw), R[0][2] = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
    R[1][0] = sin(roll)*cos(pitch), R[1][1] = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw), R[1][2] = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
    R[2][0] = -sin(pitch),          R[2][1] = cos(pitch)*sin(yaw),                                R[2][2] = cos(pitch)*cos(yaw);
    
}

// Caluculate localization of the robot by using SSGA. Created by Yuichiro Toda.
// fitcal_m関数の修正
double SLAM::fitcal_m(int i, struct robot_position *robot, std::vector<std::vector<int>> &omap, std::vector<std::vector<int>> &amap) {
    if (!robot) return 0.0;

    int hit = 0, err = 0;
    int hit2 = 0, err2 = 0, unce = 0;
    double p = 0.0, p2 = 0.0, p4 = 0.0;
    int num = 0;

    // std::cout << "Evaluating fitness for gene " << i << std::endl;

    // 評価対象の姿勢を計算
    double eval_angle = robot->dangle + robot->gen_m[i][2];
    double angle_rad = eval_angle * M_PI / 180.0;
    double s1 = sin(angle_rad);
    double c1 = cos(angle_rad);

    // std::cout << "Pose: angle=" << eval_angle << ", sin=" << s1 << ", cos=" << c1 << std::endl;

    // 評価位置の計算
    double x_c2 = robot->real_x;
    double y_c2 = robot->real_y;
    x_c2 += robot->gen_m[i][0] * c1 - robot->gen_m[i][1] * s1;
    y_c2 += robot->gen_m[i][1] * c1 + robot->gen_m[i][0] * s1;

    // std::cout << "Evaluation position: x=" << x_c2 << ", y=" << y_c2 << std::endl;

    // レーザーデータの評価
    for(size_t k = 0; k < laser_x.size(); k += 2) {
        if(laser_x[k] != _err && laser_y[k] != _err) {
            // レーザー点をグローバル座標系に変換
            double x_3 = (double)laser_x[k] * c1 - (double)laser_y[k] * s1 + x_c2;
            double y_3 = (double)laser_x[k] * s1 + (double)laser_y[k] * c1 + y_c2;

            // グローバル座標を地図座標系に変換
            int pt1_x = (int)(_map_center_x + x_3 / _map_rate);
            int pt1_y = (int)(_map_center_y + y_3 / _map_rate);

            // 地図範囲チェック
            if(pt1_x < 0 || pt1_x >= omap.size() || pt1_y < 0 || pt1_y >= omap[0].size()) {
                continue;
            }

            // 占有値の評価
            double h = omap[pt1_x][pt1_y];
            double h2 = amap[pt1_x][pt1_y];

            if(h2 > 0) {
                hit++;
            } else {
                err++;
            }

            // 確率値に基づく評価
            double prob = random.cal_probability(h);
            if(prob > 0.3) {
                hit2++;
            } else if(prob < -0.3) {
                err2++;
            } else {
                unce++;
            }

            p4 += prob;
            num++;
        }
    }

    // スコアの計算
    if(num == 0) {
        // std::cout << "No valid points for evaluation" << std::endl;
        return 0.0;
    }

    // 累積マップとの一致度
    p2 = (hit == 0) ? 0.0 : (double)hit / (double)(hit + err);
    // std::cout << "Cumulative map match: hit=" << hit << ", err=" << err << ", score=" << p2 << std::endl;

    // 占有格子との一致度 
    p = (hit2 == 0) ? 0.0 : (double)hit2 / (double)(hit2 + err2 + unce);
    // std::cout << "Occupancy grid match: hit2=" << hit2 << ", err2=" << err2 << ", unce=" << unce << ", score=" << p << std::endl;

    // 最終スコア
    p4 = p * p2;
    // std::cout << "Final fitness score: " << p4 << std::endl;

    return p4;
}

int SLAM::g_max(struct robot_position *robot, double *best_fitness)
{
	int j;
	double max;
	for(int i=0;i<_ganm;i++){
		if(i == 0){
			max = robot->fit_m[i];
			j = i;
		}else if(max < robot->fit_m[i]){
			max = robot->fit_m[i];
			j = i;
		}
	}
	*best_fitness = max;
	return j;
}

int SLAM::g_min(struct robot_position *robot, double *worst_fitness)
{
	int j;
	double min;
	for(int i=0;i<_ganm;i++){
		if(i == 0){
			min = robot->fit_m[i];
			j = i;
		}else if(min > robot->fit_m[i]){
			min = robot->fit_m[i];
			j = i;
		}
	}
	*worst_fitness = min;
	return j;
}

void SLAM::mga_init(struct robot_position *robot, std::vector<std::vector<int>> &omap, std::vector<std::vector<int>> &amap) {
    if (!robot) return;

    for (int i = 0; i < _ganm; i++) {
        if (i == 0) {
            robot->gen_m[i][0] = robot->gen_m[robot->best_gene][0];
            robot->gen_m[i][1] = robot->gen_m[robot->best_gene][1];
            robot->gen_m[i][2] = robot->gen_m[robot->best_gene][2];
            robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
            continue;
        }
        else if (i == 1) {
            robot->gen_m[i][0] = 0;
            robot->gen_m[i][1] = 0;
            robot->gen_m[i][2] = 0;
            robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
            continue;
        }
        else if (i == 2) {
            robot->gen_m[i][0] = -robot->gen_m[robot->best_gene][0];
            robot->gen_m[i][1] = -robot->gen_m[robot->best_gene][1];
            robot->gen_m[i][2] = -robot->gen_m[robot->best_gene][2];
            robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
            continue;
        }
        else {
            robot->gen_m[i][0] = 40.0*random.rndn();
            robot->gen_m[i][1] = 40.0*random.rndn();
            robot->gen_m[i][2] = random.rndn();
        }
        robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
    }
}


void SLAM::map_mating(struct robot_position *robot, std::vector<std::vector<int>> &omap, std::vector<std::vector<int>> &amap) {
    int t = 0;  // ループカウンタの初期化
    int g, g2;  // 遺伝子インデックス
    int worst_gene;  // 最悪遺伝子のインデックス
    double best_fitness;  // 最良の適応度
    double worst_fitness;  // 最悪の適応度
    
    mga_init(robot, omap, amap);
    
    while (t < _t2) {
        double r, r2;
        robot->best_gene = g_max(robot, &best_fitness);
        worst_gene = g_min(robot, &worst_fitness);
        
        g = (int)(_ganm * random.rnd());
        while(g == robot->best_gene) {
            g = (int)(_ganm * random.rnd());
        }
        
        // スコアの差に基づく探索範囲の調整
        r2 = (10.0 * (best_fitness - robot->fit_m[g])) / (best_fitness - worst_fitness + 0.01);
        
        g2 = robot->best_gene;
        for (int j = 0; j < GALM; j++) {
            r = random.rnd();
            // クロスオーバー
            if (r < 0.7) {
                robot->gen_m[worst_gene][j] = robot->gen_m[g2][j];
            } else {
                robot->gen_m[worst_gene][j] = (robot->gen_m[g][j] + robot->gen_m[g2][j]) * random.rnd();
            }

            // 突然変異
            if (random.rnd() < 0.7) {
                if (j != 2) {  // x, y座標の変異
                    robot->gen_m[worst_gene][j] += 10.0 * random.rndn_10() * 
                        (0.01 + r2 * (double)(_t2 - t) / (double)_t2);
                } else {      // 角度の変異
                    robot->gen_m[worst_gene][j] += random.rndn() * 
                        (0.01 + r2 * (double)(_t2 - t) / (double)_t2);
                }
            }
        }
        
        robot->fit_m[worst_gene] = fitcal_m(worst_gene, robot, omap, amap);
        t++;
    }
}
double SLAM::probability_to_logodds(double prob) {
    return log(prob / (1.0 - prob));
}

double SLAM::logodds_to_probability(double logodds) {
    return 1.0 - (1.0 / (1.0 + exp(logodds)));
}
void SLAM::calc_mapvalue(int k, int l, int pt_x, int pt_y, int sflag, 
                        std::vector<std::vector<int>> &omap, 
                        std::vector<std::vector<int>> &amap) {
    // 範囲チェック
    if (k < 0 || k >= omap.size() || l < 0 || l >= omap[0].size() ||
        pt_x < 0 || pt_x >= omap.size() || pt_y < 0 || pt_y >= omap[0].size()) {
        return;
    }

    if (_use_logodds) {
        // 対数オッズベースの更新
        double current_logodds = probability_to_logodds((double)(omap[k][l] + 100) / 200.0);
        
        bool is_occupied = false;
        if (abs(k - pt_x) <= 1 && abs(l - pt_y) <= 1) {
            current_logodds += _l_occ;
            is_occupied = true;
        } else {
            switch(sflag) {
                case 1: is_occupied = (k >= pt_x); break;
                case 2: is_occupied = (k <= pt_x); break;
                case 3: is_occupied = (l >= pt_y); break;
                case 4: is_occupied = (l <= pt_y); break;
            }
            current_logodds += is_occupied ? _l_occ : _l_free;
        }
        
        current_logodds = std::min(_l_max, std::max(_l_min, current_logodds));
        double prob = logodds_to_probability(current_logodds);
        omap[k][l] = (int)((prob * 200.0) - 100);  // [-100, 100]の範囲に変換
        
        if (is_occupied) {
            amap[k][l]++;
        }
    } else {
        // 従来のロジック（値の範囲を確認）
        switch(sflag) {
            case 1:
                if(k >= pt_x) omap[k][l] = std::min(omap[k][l] + 2, 100);
                else omap[k][l] = std::max(omap[k][l] - 1, -100);
                break;
            case 2:
                if(k <= pt_x) omap[k][l] = std::min(omap[k][l] + 2, 100);
                else omap[k][l] = std::max(omap[k][l] - 1, -100);
                break;
            case 3:
                if(l >= pt_y) omap[k][l] = std::min(omap[k][l] + 2, 100);
                else omap[k][l] = std::max(omap[k][l] - 1, -100);
                break;
            case 4:
                if(l <= pt_y) omap[k][l] = std::min(omap[k][l] + 2, 100);
                else omap[k][l] = std::max(omap[k][l] - 1, -100);
                break;
        }
        
        if(random.cal_probability(amap[k][l]) > 0.85) {
            if(omap[k][l] < 0 && amap[k][l] + omap[k][l] > 0) {
                omap[k][l] = std::min(omap[k][l] + amap[k][l], 100);
            }
        }
    }
}

void SLAM::expand_map_if_needed(int required_x, int required_y, 
                               std::vector<std::vector<int>> &omap, 
                               std::vector<std::vector<int>> &amap) {
    // マップが空の場合の初期化
    if(omap.empty() || omap[0].empty()) {
        omap.resize(1, std::vector<int>(1, 0));
        amap.resize(1, std::vector<int>(1, 0));
        _current_map_width = 1;
        _current_map_height = 1;
        _map_center_x = 0;
        _map_center_y = 0;
    }

    int current_width = omap.size();
    int current_height = omap[0].size();
    int new_width = current_width;
    int new_height = current_height;
    bool need_expansion = false;
    int offset_x = 0;
    int offset_y = 0;

    // 必要な拡張サイズの計算
    if (required_x < 0) {
        int extension = abs(required_x) + _map_expansion_size;
        new_width += extension;
        offset_x = extension;
        need_expansion = true;
    } else if (required_x >= current_width) {
        new_width = required_x + _map_expansion_size + 1;  // +1 for safety
        need_expansion = true;
    }

    if (required_y < 0) {
        int extension = abs(required_y) + _map_expansion_size;
        new_height += extension;
        offset_y = extension;
        need_expansion = true;
    } else if (required_y >= current_height) {
        new_height = required_y + _map_expansion_size + 1;  // +1 for safety
        need_expansion = true;
    }

    // 拡張が必要な場合の処理
    if (need_expansion) {
        try {
            // 新しいマップの作成
            std::vector<std::vector<int>> new_omap(new_width, std::vector<int>(new_height, 0));
            std::vector<std::vector<int>> new_amap(new_width, std::vector<int>(new_height, 0));

            // 既存データのコピー
            for (int x = 0; x < current_width; x++) {
                for (int y = 0; y < current_height; y++) {
                    if (x + offset_x < new_width && y + offset_y < new_height) {
                        new_omap[x + offset_x][y + offset_y] = omap[x][y];
                        new_amap[x + offset_x][y + offset_y] = amap[x][y];
                    }
                }
            }

            // マップの更新
            omap = std::move(new_omap);
            amap = std::move(new_amap);
            
            // 中心座標の更新
            _map_center_x += offset_x;
            _map_center_y += offset_y;
            
            // サイズの更新
            _current_map_width = new_width;
            _current_map_height = new_height;
        } catch (const std::exception& e) {
            // メモリ確保失敗などの例外処理
            printf("Map expansion failed: %s", e.what());
            return;
        }
    }
}

void SLAM::expand_map(int new_width, int new_height,
                     std::vector<std::vector<int>> &omap, 
                     std::vector<std::vector<int>> &amap) {
    // 現在のマップサイズを保存
    int old_width = omap.size();
    int old_height = omap[0].size();

    // 新しいマップを作成
    std::vector<std::vector<int>> new_omap(new_width, std::vector<int>(new_height, 0));
    std::vector<std::vector<int>> new_amap(new_width, std::vector<int>(new_height, 0));

    // オフセットを計算（現在のマップの中心を維持）
    int offset_x = (new_width - old_width) / 2;
    int offset_y = (new_height - old_height) / 2;

    // 既存のデータをコピー
    for (int x = 0; x < old_width; x++) {
        for (int y = 0; y < old_height; y++) {
            int new_x = x + offset_x;
            int new_y = y + offset_y;
            if (new_x >= 0 && new_x < new_width && new_y >= 0 && new_y < new_height) {
                new_omap[new_x][new_y] = omap[x][y];
                new_amap[new_x][new_y] = amap[x][y];
            }
        }
    }

    // マップを更新
    omap = std::move(new_omap);
    amap = std::move(new_amap);
    
    // 新しいサイズを保存
    _current_map_width = new_width;
    _current_map_height = new_height;
}


// public
// パラメータをセットする関数
void SLAM::set_param(int err, double map_rate, int t2) {
    _err = err;
    _map_rate = map_rate;
    _t2 = t2;
}
void SLAM::set_ganm(int ganm) {
    _ganm = ganm;
}
// laser_dataをセットする関数
void SLAM::set_laser_data(long *lrf_range,int data_num,double angle_increment,double angle_min,double range_min,double range_max, double mountX, double mountY, double mountYaw){

	
    
    laser_x.clear();
    laser_y.clear();
	
    double rad = angle_min+ mountYaw;
    for(int i=0;i<data_num;i++){
        if(lrf_range[i] < range_min || lrf_range[i] > range_max){
            laser_x.push_back(_err);
            laser_y.push_back(_err);
        }else{
            laser_x.push_back(lrf_range[i]*cos(rad)+ mountX);
            laser_y.push_back(lrf_range[i]*sin(rad)+mountY);
        }
        rad += angle_increment;
    }
}
void SLAM::set_laser_data_xy(
    const std::vector<double>& x_points,
    const std::vector<double>& y_points,
    double range_min,
    double range_max,
    double mountX,
    double mountY,
    double mountYaw
) {
    if (x_points.size() != y_points.size()) {
        throw std::invalid_argument("x_points and y_points must have the same size");
    }

    // レーザーデータをクリア
    laser_x.clear();
    laser_y.clear();

    // 回転行列のパラメータを計算
    double cos_yaw = std::cos(mountYaw);
    double sin_yaw = std::sin(mountYaw);

    // 各点を処理
    for (size_t i = 0; i < x_points.size(); i++) {
        double x = x_points[i];
        double y = y_points[i];

        // 点までの距離を計算
        double range = std::sqrt(x*x + y*y);

        // 距離が有効範囲内かチェック
        if (range >= range_min && range <= range_max) {
            // マウント位置を考慮した座標変換
            double transformed_x = x * cos_yaw - y * sin_yaw + mountX;
            double transformed_y = x * sin_yaw + y * cos_yaw + mountY;

            laser_x.push_back(static_cast<float>(transformed_x));
            laser_y.push_back(static_cast<float>(transformed_y));
        } else {
            // 無効な点の場合
            laser_x.push_back(_err);
            laser_y.push_back(_err);
        }
    }
}

// 自己位置追跡関数(tracking module)
void SLAM::slam(struct robot_position *robot, std::vector<std::vector<int>> &omap, std::vector<std::vector<int>> &amap){
    static double pre_angle;

    pre_angle = robot->dangle;
    int i,j;
    if(robot->slam_count==0){        
        for(i=0;i<_ganm+1;i++){
            robot->fit_m[i] = 0;
            for(j=0;j<GALM+1;j++)
                robot->gen_m[i][j] = 0;
        }
        robot->best_gene = 0;
    }
    
    if(robot->slam_count > 1) {
        map_mating(robot, omap, amap);
        
        // 位置の更新（座標系を考慮）
        double s_x = robot->gen_m[robot->best_gene][0];
        double s_y = robot->gen_m[robot->best_gene][1];
        double s_r = robot->gen_m[robot->best_gene][2];
        
        robot->real_x += s_x * cos(robot->dangle * M_PI / 180.0) - 
                        s_y * sin(robot->dangle * M_PI / 180.0);
        robot->real_y += s_y * cos(robot->dangle * M_PI / 180.0) + 
                        s_x * sin(robot->dangle * M_PI / 180.0);
        
        robot->dangle += s_r;
        robot->rangle = robot->dangle;  // 絶対角度の更新
        
        // 地図座標の更新
        robot->map_x = (int)(_map_center_x + robot->real_x / _map_rate);
        robot->map_y = (int)(_map_center_y + robot->real_y / _map_rate);
        
        // 角度の正規化
        if(fabs(robot->rangle) > 180.0) {
            double err_a = fabs(robot->rangle) - 180.0;
            if(robot->rangle > 0) {
                robot->rangle = -180.0 + err_a;
            } else {
                robot->rangle = 180.0 - err_a;
            }
        }
    }
    
    robot->slam_count++;
}
void SLAM::map_building2(struct robot_position *robot, std::vector<std::vector<int>> &omap, std::vector<std::vector<int>> &amap) {
    // 無効なデータのチェック
    if (!robot) {
        printf("Invalid robot pointer in map_building2");
        return;
    }

    // マップの有効性チェック
    if (omap.empty() || omap[0].empty() || amap.empty() || amap[0].empty()) {
        printf("Empty maps in map_building2");
        omap.resize(1, std::vector<int>(1, 0));
        amap.resize(1, std::vector<int>(1, 0));
        _current_map_width = 1;
        _current_map_height = 1;
        _map_center_x = 0;
        _map_center_y = 0;
    }

    try {
        double angle_rad = (double)robot->dangle * M_PI / 180.0;
        double s1 = sin(angle_rad);
        double c1 = cos(angle_rad);

        // ロボットの現在位置を地図座標系に変換
        int ptc_x = (int)(_map_center_x + robot->real_x / _map_rate);
        int ptc_y = (int)(_map_center_y + robot->real_y / _map_rate);

        // ロボット位置に対する地図拡張
        expand_map_if_needed(ptc_x, ptc_y, omap, amap);

        // 現在のマップサイズを取得
        int current_width = omap.size();
        int current_height = omap[0].size();

        // レーザースキャンデータの処理
        for(size_t j = 0; j < laser_x.size(); j++) {
            if(laser_x[j] == _err || laser_y[j] == _err) {
                continue;
            }

            // レーザー点をグローバル座標系に変換
            double x_3 = (double)laser_x[j] * c1 - (double)laser_y[j] * s1 + (double)robot->real_x;
            double y_3 = (double)laser_x[j] * s1 + (double)laser_y[j] * c1 + (double)robot->real_y;

            // グローバル座標を地図座標系に変換
            int pt_x = (int)(_map_center_x + x_3 / _map_rate);
            int pt_y = (int)(_map_center_y + y_3 / _map_rate);

            // レーザー点に対する地図拡張
            expand_map_if_needed(pt_x, pt_y, omap, amap);

            // 地図サイズの更新（拡張後）
            current_width = omap.size();
            current_height = omap[0].size();

            // 座標の有効性チェック
            if (pt_x < 0 || pt_x >= current_width || pt_y < 0 || pt_y >= current_height ||
                ptc_x < 0 || ptc_x >= current_width || ptc_y < 0 || ptc_y >= current_height) {
                continue;
            }

            // レーザー終点の占有をインクリメント
            amap[pt_x][pt_y]++;

            // ブレゼンハムのレイトレース
            int dx = pt_x - ptc_x;
            int dy = pt_y - ptc_y;
            int k, l;
            int e = 0;  // ブレゼンハムのエラー項

            // 始点と終点が同じ場合
            if (dx == 0 && dy == 0) {
                calc_mapvalue(ptc_x, ptc_y, pt_x, pt_y, 1, omap, amap);
                continue;
            }

            // 第1象限（dx≥0, dy≥0）
            if (dx >= 0 && dy >= 0) {
                if (dx > dy) {
                    for (l = ptc_y, k = ptc_x; k <= pt_x; k++) {
                        e += dy;
                        if (e > dx) {
                            e -= dx;
                            l++;
                        }
                        calc_mapvalue(k, l, pt_x, pt_y, 1, omap, amap);
                    }
                } else {
                    for (l = ptc_y, k = ptc_x; l <= pt_y; l++) {
                        e += dx;
                        if (e > dy) {
                            e -= dy;
                            k++;
                        }
                        calc_mapvalue(k, l, pt_x, pt_y, 3, omap, amap);
                    }
                }
            }
            // 第4象限（dx≥0, dy<0）
            else if (dx >= 0 && dy < 0) {
                dy = -dy;
                if (dx > dy) {
                    for (l = ptc_y, k = ptc_x; k <= pt_x; k++) {
                        e += dy;
                        if (e > dx) {
                            e -= dx;
                            l--;
                        }
                        calc_mapvalue(k, l, pt_x, pt_y, 1, omap, amap);
                    }
                } else {
                    for (l = ptc_y, k = ptc_x; l >= pt_y; l--) {
                        e += dx;
                        if (e > dy) {
                            e -= dy;
                            k++;
                        }
                        calc_mapvalue(k, l, pt_x, pt_y, 4, omap, amap);
                    }
                }
            }
            // 第2象限（dx<0, dy≥0）
            else if (dx < 0 && dy >= 0) {
                dx = -dx;
                if (dx > dy) {
                    for (l = ptc_y, k = ptc_x; k >= pt_x; k--) {
                        e += dy;
                        if (e > dx) {
                            e -= dx;
                            l++;
                        }
                        calc_mapvalue(k, l, pt_x, pt_y, 2, omap, amap);
                    }
                } else {
                    for (l = ptc_y, k = ptc_x; l <= pt_y; l++) {
                        e += dx;
                        if (e > dy) {
                            e -= dy;
                            k--;
                        }
                        calc_mapvalue(k, l, pt_x, pt_y, 3, omap, amap);
                    }
                }
            }
            // 第3象限（dx<0, dy<0）
            else {
                dx = -dx;
                dy = -dy;
                if (dx > dy) {
                    for (l = ptc_y, k = ptc_x; k >= pt_x; k--) {
                        e += dy;
                        if (e > dx) {
                            e -= dx;
                            l--;
                        }
                        calc_mapvalue(k, l, pt_x, pt_y, 2, omap, amap);
                    }
                } else {
                    for (l = ptc_y, k = ptc_x; l >= pt_y; l--) {
                        e += dx;
                        if (e > dy) {
                            e -= dy;
                            k--;
                        }
                        calc_mapvalue(k, l, pt_x, pt_y, 4, omap, amap);
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        printf("Error in map_building2: %s", e.what());
    }
}

// Initialize the input robot position. Created by Yuichiro Toda on 12/04/19.
void SLAM::initialize_RobotPosition(struct robot_position *robot, int dflag) {
    if (!robot) return;

    robot->gen_m.resize(_ganm + 1, std::vector<double>(GALM + 1, 0.0));
    robot->fit_m.resize(_ganm + 1, 0.0);

    if(dflag == 0) {
        robot->real_x = 0;
        robot->real_y = 0;
        robot->rangle = 0;
        robot->dangle = 0;
        robot->longitude = 0.0;
        robot->latitude = 0.0;
        robot->map_x = _map_center_x;  // _map_centerから_map_center_xに変更
        robot->map_y = _map_center_y;  // _map_centerから_map_center_yに変更
        robot->slam_count = 0;
        robot->best_gene = 0;
    } else {
        robot->real_x = _err;
        robot->real_y = _err;
        robot->rangle = 0;
        robot->dangle = 0;
        robot->map_x = 0;
        robot->map_y = 0;
        robot->longitude = 0.0;
        robot->latitude = 0.0;
        robot->slam_count = 0;
        robot->best_gene = 0;
    }
}

void SLAM::set_logodds_param(
    double l_occ,
    double l_free,
    double l_max,
    double l_min,
    double thresh_occ,
    double thresh_free
) {
    _l_occ = l_occ;
    _l_free = l_free;
    _l_max = l_max;
    _l_min = l_min;
    _l_thresh_occ = thresh_occ;
    _l_thresh_free = thresh_free;
}

void SLAM::enable_logodds(bool enable) {
    _use_logodds = enable;
}

void SLAM::set_resampler_parameters(double distanceThreshold, double lengthThreshold) {
    resampler.setParameters(distanceThreshold, lengthThreshold);
}

void SLAM::set_laser_data_with_resampling(
    long* lrf_range,
    int data_num,
    double angle_increment,
    double angle_min,
    double range_min,
    double range_max,
    double mountX,
    double mountY,
    double mountYaw
) {
    // First, convert laser data to Scan2D format
	Scan2D scan;
	double rad = angle_min + mountYaw;
	
	for (int i = 0; i < data_num; i++) {
		if (lrf_range[i] >= range_min && lrf_range[i] <= range_max) {
			double x = lrf_range[i] * cos(rad) + mountX;
			double y = lrf_range[i] * sin(rad) + mountY;
			scan.lps.emplace_back(LPoint2D(i, x, y));
		}
		rad += angle_increment;
	}
	
	// Apply resampling
	resampler.resamplePoints(&scan);
	
	// Convert resampled points back to laser format
	laser_x.clear();
	laser_y.clear();
	
	for (const auto& point : scan.lps) {
		laser_x.push_back(point.x);
		laser_y.push_back(point.y);
	}
}

