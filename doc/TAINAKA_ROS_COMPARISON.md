# gng-slam-newFuzzy1 vs Tainaka_ROS 比較

## 概要

| 項目 | gng-slam-newFuzzy1 | Tainaka_ROS |
|------|-------------------|-------------|
| プラットフォーム | Windows (独自アプリ) | ROS1 (catkin) |
| 言語 | C++ | C++ |
| センサー | UTM-30LX / RS-LiDAR-16 | /base_scan トピック |
| SLAMアルゴリズム | ハイブリッドGA-SLAM | ハイブリッドGA-SLAM（同一） |

---

## 1. SLAM・自己位置推定の比較

### 1.1 コア関数の比較

両バージョンで**同一のハイブリッドマッチング**が実装されています：

| 関数 | gng-slam-newFuzzy1 | Tainaka_ROS | 説明 |
|------|-------------------|-------------|------|
| `fitcal_m()` | ✅ | ✅ | 単一マップマッチング |
| `fitcal_MIX()` | ✅ | ✅ | ハイブリッドマッチング |
| `compare_map()` | ✅ | ✅ | マップ一致率計算 |
| `map_mating()` | ✅ (`mga()`) | ✅ | GA交叉・突然変異 |
| `map_building2()` | ✅ | ✅ | マップ構築 |

### 1.2 fitcal_MIX() の実装比較

**gng-slam-newFuzzy1 (slam.cpp:163-189)**
```cpp
double fitcal_MIX(int i, struct robot_position* robot, int** omap, int** amap) {
    fit_env = fitcal_m(i, robot, omap, amap);
    fit_cad = fitcal_m(i, robot, omap_CAD, amap_CAD);

    dis_m = sqrt((robot->model[0]^2) + (robot->model[1]^2));
    dis_g = sqrt((robot->gen_m[i][0]^2) + (robot->gen_m[i][1]^2));
    model_ratio = abs(dis_m - dis_g) / dis_m;

    weight = map_match_rate * (1 - model_ratio);
    fit_mix = weight * fit_env + (1 - weight) * fit_cad;
    return fit_mix;
}
```

**Tainaka_ROS (slam_class.cpp:108-141)**
```cpp
double SLAM::fitcal_MIX(int i, struct robot_position* robot, int** &omap, int** &amap) {
    fit_env = fitcal_m(i, robot, omap, amap);
    fit_cad = fitcal_m(i, robot, omap_CAD, amap_CAD);

    dis_m = sqrt((model[0] * model[0]) + (model[1] * model[1]));
    dis_g = sqrt((robot->gen_m[i][0]^2) + (robot->gen_m[i][1]^2));

    // ゼロ除算対策が追加
    if(dis_m == 0){
        model_ratio = 0;
    } else {
        model_ratio = abs(dis_m - dis_g) / dis_m;
    }

    weight = map_match_rate * (1 - model_ratio);
    fit_mix = weight * fit_env + (1 - weight) * fit_cad;
    return fit_mix;
}
```

**差異**: Tainaka_ROSでは`dis_m == 0`のゼロ除算対策が追加されている

---

## 2. マップリセット閾値の比較

| バージョン | 閾値 | 場所 |
|-----------|------|------|
| gng-slam-newFuzzy1 | `< 0.98` | slam.cpp:342 |
| Tainaka_ROS (クラス) | `< 0.90` | slam_class.cpp:248 |
| Tainaka_ROS (ノード) | `< 0.9988` | gaslam_test_node.cpp:202 |

**Tainaka_ROS**では、ROSノード側でより厳しい閾値 (0.9988) を使用しています。

---

## 3. 運動モデルの違い

### gng-slam-newFuzzy1
```cpp
// 固定の制御出力から計算
foutp[0] = outp[0];
foutp[1] = outp[1];
```

### Tainaka_ROS
```cpp
// cmd_velトピックから速度情報を取得
void GaSlamNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg){
    slam.vel[0] = msg->linear.x;   // x方向速度
    slam.vel[1] = msg->linear.y;   // y方向速度
    slam.vel[2] = msg->angular.z;  // 回転速度
}

// mga_init()でモデル計算
void SLAM::mga_init(struct robot_position *robot, int** &omap, int** &amap) {
    double dt = 0.05154639;  // base_scanの周期 (19.4Hz)
    model[0] = vel[0] * dt;
    model[1] = vel[1] * dt;
    model[2] = vel[2] * dt;
    ...
}
```

---

## 4. CADマップ更新機能（Tainaka_ROS独自）

**Tainaka_ROS**には、環境の変化をCADマップに反映する機能が追加されています：

```cpp
// slam_class.cpp:564-577 - map_building2()内
// 環境マップで新たに検出された障害物をCADマップに追加
if (apply) {
    for (int i = 0; i < _map_size_y; i++) {
        for (int j = 0; j < _map_size_x; j++) {
            // CADマップにない障害物が環境マップで検出された場合
            if (random.cal_probability(omap_CAD[i][j]) < 0.8 &&
                random.cal_probability(omap[i][j]) >= 0.8 &&
                amap[i][j] > 100) {
                // CADマップを更新
                omap_CAD[i][j] = omap[i][j];
                amap_CAD[i][j] = amap[i][j];
            }
        }
    }
}
```

この機能により、CADマップに存在しない新しい障害物（家具の移動など）を学習できます。

---

## 5. ROS統合（Tainaka_ROS独自）

### 5.1 トピック構成

| トピック | 方向 | 説明 |
|---------|------|------|
| `/base_scan` | Subscribe | LiDARスキャンデータ |
| `/teleop/cmd_vel` | Subscribe | 速度コマンド |
| `/map` | Publish | 占有格子地図 |

### 5.2 TF変換

```cpp
// gaslam_test_node.cpp:267-273
tf::Transform laser_to_map;
laser_to_map = tf::Transform(
    tf::createQuaternionFromRPY(0, 0, robot_pos_s1->rangle*M_PI/180),
    tf::Vector3(mpose_x, mpose_y, 0.0)
);
transform = (odom_to_laser * laser_to_map.inverse()).inverse();
br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
    m_map_frame.c_str(), m_odom_frame.c_str()));
```

### 5.3 フレーム構成

```
map_frame
    └── odom_frame
         └── base_footprint
              └── base_scan
```

---

## 6. クラス設計の違い

### gng-slam-newFuzzy1
- グローバル変数とスタンドアロン関数
- Windowsアプリケーション統合

### Tainaka_ROS
- `SLAM`クラスにカプセル化
- `Random`クラスを別途用意
- ROSノードクラス `GaSlamNode` でラップ

```
┌──────────────────────────────────────────────────────────────┐
│                     GaSlamNode (ROS)                         │
├──────────────────────────────────────────────────────────────┤
│  - scan_sub: /base_scan サブスクライバ                       │
│  - cmdvel_sub: /teleop/cmd_vel サブスクライバ                │
│  - map_pub: /map パブリッシャ                                │
│  - slam: SLAMクラスインスタンス                              │
│  - robot_pos_s1: ロボット位置構造体                          │
│  - map1, map2: 占有格子マップ                                │
├──────────────────────────────────────────────────────────────┤
│                         SLAM Class                           │
├──────────────────────────────────────────────────────────────┤
│  - omap_CAD, amap_CAD: CADマップ                             │
│  - map_match_rate: マップ一致率                              │
│  - vel[3]: 速度データ                                        │
│  - model[3]: 運動モデル                                      │
│  - fitcal_m(), fitcal_MIX()                                 │
│  - compare_map(), map_mating()                              │
│  - slam(), map_building2()                                   │
└──────────────────────────────────────────────────────────────┘
```

---

## 7. マップサイズ処理の違い

### gng-slam-newFuzzy1
```cpp
// 固定サイズ定義
#define MAPSIZE 3000
int map_size_width = 3000;
int map_size_height = 3000;
```

### Tainaka_ROS
```cpp
// 画像サイズから動的に設定
cv::Mat image = cv::imread("/home/user/maps/map.jpg", cv::IMREAD_GRAYSCALE);
slam.set_param(image.cols, image.rows, m_err, m_map_rate, m_t2);
```

---

## 8. 処理フローの比較

### gng-slam-newFuzzy1
```
┌─────────────────────────────────────────────────┐
│ main() ループ                                   │
├─────────────────────────────────────────────────┤
│  1. LRFデータ取得 (urg_get_distance)            │
│  2. slam() / slam2() 呼び出し                   │
│  3. map_building2() 呼び出し                    │
│  4. controller (fuzzy) 呼び出し                 │
│  5. モータ制御出力                              │
│  6. 画面描画                                    │
└─────────────────────────────────────────────────┘
```

### Tainaka_ROS
```
┌─────────────────────────────────────────────────┐
│ scanCallback() (トピック受信時)                  │
├─────────────────────────────────────────────────┤
│  1. TF変換取得 (odom → laser)                   │
│  2. LRFデータ変換 (set_laser_data)              │
│  3. compare_map() でマップ比較                  │
│  4. マップリセット判定 (0.9988閾値)             │
│  5. slam() 呼び出し                             │
│  6. map_building2() 呼び出し                    │
│  7. OccupancyGrid パブリッシュ                  │
│  8. TF パブリッシュ (map → odom)                │
└─────────────────────────────────────────────────┘
```

---

## 9. まとめ

### 共通点
1. **ハイブリッドマッチング**: 両方とも`fitcal_MIX()`によるCAD+SLAM統合
2. **GAアルゴリズム**: 同一の遺伝的アルゴリズム (GANM=100個体)
3. **マップ一致率計算**: `compare_map()`による動的評価
4. **マップリセット機能**: 一致率低下時のCADマップへのリセット

### Tainaka_ROSの改良点
1. **ゼロ除算対策**: `model_ratio`計算時の安全対策
2. **CADマップ更新**: 新規障害物のCADマップへの学習
3. **ROS統合**: トピック・TF・サービスによる標準化
4. **動的マップサイズ**: 画像から自動設定
5. **より厳しいリセット閾値**: 0.9988（精度向上）

### gng-slam-newFuzzy1の特徴
1. **GNGパスプランニング**: Growing Neural Gasによる経路計画
2. **ファジィ制御**: 目標追従・障害物回避の統合制御
3. **多目的行動調停**: 複数行動の重み付け制御
