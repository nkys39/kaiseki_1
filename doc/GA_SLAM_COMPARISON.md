# GA-SLAM アルゴリズム比較

## 概要

4つのバージョンのGA-SLAM（遺伝的アルゴリズムベースのSLAM）実装を比較します。

| 項目 | backup | newFuzzy1 | Tainaka_ROS | es_slam |
|------|--------|-----------|-------------|---------|
| 個体数 (GANM) | 100 | 100 | 100 | 100 (可変) |
| 遺伝子数 (GALM) | 3 | 3 | 3 | 3 |
| 世代数 (T2) | 500 | 500 | 500 | 500 |
| 運動モデル | 速度指令値ベース | 速度指令値ベース | cmd_vel | なし |
| 運動モデルON/OFF | `modelMode` | `modelMode` | なし（常にON） | なし |
| r2計算 | 固定 (0.5) | 固定 (0.5) | 適応的 | 適応的 |
| ハイブリッドマッチング | なし | あり | あり | なし |

---

## 1. 初期化関数 `mga_init()` の比較

### 1.1 backup / newFuzzy1

```cpp
void mga_init(struct robot_position *robot, int **omap, int **amap)
{
    // 速度指令値から運動モデル計算（エンコーダではない）
    double model[3];
    calcRobotModel(model);  // foutp（モータ出力値）を使用
    robot->model[0] = model[0];
    robot->model[1] = model[1];
    robot->model[2] = model[2];

    for (int i = 0; i < GANM; i++)
    {
        if (ga_search_range == 0) {
            if (i == 0) {
                // 前回のベスト個体を保持
                robot->gen_m[i][0] = robot->gen_m[robot->best_gene][0];
                ...
            }
            else if (i == 1) {
                // ゼロ個体
                robot->gen_m[i][0] = 0;
                ...
            }
            else if (i == 2) {
                // 反転個体
                robot->gen_m[i][0] = -robot->gen_m[robot->best_gene][0];
                ...
            }
            else {
                if (modelMode == 0) {
                    // ランダム初期化
                    robot->gen_m[i][0] = 40.0 * rndn();
                    robot->gen_m[i][1] = 40.0 * rndn();
                    robot->gen_m[i][2] = 1.0 * rndn();
                }
                else {
                    // 運動モデル周辺に初期化
                    robot->gen_m[i][0] = model[0] + 40.0 * rndn();
                    robot->gen_m[i][1] = model[1] + 40.0 * rndn();
                    robot->gen_m[i][2] = model[2] + 1.0 * rndn();
                }
            }
        }
    }
}
```

**特徴**:
- `calcRobotModel()` による速度指令値ベースの運動モデル使用（※エンコーダではない）
- `modelMode` パラメータで初期化方式を選択（ON/OFF切替可能）
  - `modelMode=0`: ランダム初期化（運動モデル不使用）
  - `modelMode=1`: 運動モデル周辺に初期化
- `ga_search_range` パラメータで探索範囲を制御
- 最初の3個体を特殊個体として配置

#### calcRobotModel()の実装詳細

```cpp
// slam.cpp:1038-1039
wL = alpha[0] * foutp[1];  // 左車輪の速度指令値 × 変換係数
wR = alpha[1] * foutp[0];  // 右車輪の速度指令値 × 変換係数

vL = (r / 2.0) * wL;       // 車輪半径から移動速度へ
vR = (r / 2.0) * wR;

v = (vL + vR) / 2.0;       // 並進速度
w = (vR - vL) / d;         // 回転角速度（d=車輪間隔）
```

**注意**: `foutp`は`decision_making()`から出力されるモータ制御指令値であり、エンコーダからのフィードバック値ではない。

### 1.2 Tainaka_ROS

```cpp
void SLAM::mga_init(struct robot_position *robot, int** &omap, int** &amap)
{
    // cmd_velトピックから運動モデル計算
    double dt = 0.05154639;  // base_scanの周期 (19.4Hz)
    model[0] = vel[0] * dt;
    model[1] = vel[1] * dt;
    model[2] = vel[2] * dt;

    for (int i = 0; i < GANM; i++)
    {
        if (i == 0) {
            robot->gen_m[i][0] = robot->gen_m[robot->best_gene][0];
            ...
        }
        else if (i == 1) {
            robot->gen_m[i][0] = 0;
            ...
        }
        else if (i == 2) {
            robot->gen_m[i][0] = -robot->gen_m[robot->best_gene][0];
            ...
        }
        else {
            // 運動モデル周辺に初期化
            robot->gen_m[i][0] = model[0] + 40.0*random.rndn();
            robot->gen_m[i][1] = model[1] + 40.0*random.rndn();
            robot->gen_m[i][2] = model[2] + 40.0*random.rndn();  // 角度も40倍
        }
    }
}
```

**特徴**:
- ROSの`cmd_vel`トピックから速度取得
- 固定タイムステップ (dt = 0.05154639秒)
- `modelMode`パラメータなし（常に運動モデル使用）
- 角度の初期化範囲が大きい（40.0倍）

### 1.3 es_slam

```cpp
void SLAM::mga_init(struct robot_position *robot, std::vector<std::vector<int>> &omap, std::vector<std::vector<int>> &amap)
{
    // 運動モデルなし

    for (int i = 0; i < _ganm; i++) {
        if (i == 0) {
            robot->gen_m[i][0] = robot->gen_m[robot->best_gene][0];
            ...
        }
        else if (i == 1) {
            robot->gen_m[i][0] = 0;
            ...
        }
        else if (i == 2) {
            robot->gen_m[i][0] = -robot->gen_m[robot->best_gene][0];
            ...
        }
        else {
            // 完全ランダム初期化
            robot->gen_m[i][0] = 40.0*random.rndn();
            robot->gen_m[i][1] = 40.0*random.rndn();
            robot->gen_m[i][2] = random.rndn();
        }
    }
}
```

**特徴**:
- **運動モデルなし**（純粋なスキャンマッチングのみ）
- `_ganm`パラメータで個体数を可変設定可能
- `std::vector`による動的メモリ管理
- 最もシンプルな実装

---

## 2. 進化関数 `map_mating()` の比較

### 2.1 r2（適応係数）の計算

| バージョン | r2の計算方法 |
|-----------|-------------|
| backup | `r2 = 0.5;` （固定値） |
| newFuzzy1 | `r2 = 0.5;` （固定値） |
| Tainaka_ROS | `r2 = (10.0*(best_fitness - robot->fit_m[g])) / (best_fitness - worst_fitness + 0.01);` |
| es_slam | `r2 = (10.0*(best_fitness - robot->fit_m[g])) / (best_fitness - worst_fitness + 0.01);` |

**解説**:
- **固定値 (0.5)**: 探索範囲が一定で、収束が安定するが局所解に陥りやすい
- **適応的計算**: 適応度の差に基づいて探索範囲を動的調整、より効率的な探索が可能

### 2.2 交叉・突然変異

全バージョンで共通のロジック:

```cpp
// 交叉 (確率 70%)
if (r < 0.7)
    robot->gen_m[worst_gene][j] = robot->gen_m[g2][j];  // ベスト個体をコピー
else
    robot->gen_m[worst_gene][j] = (robot->gen_m[g][j] + robot->gen_m[g2][j])*rnd();  // 平均

// 突然変異 (確率 70%)
if (rnd() < 0.7) {
    if (j != 2)  // x, y座標
        robot->gen_m[worst_gene][j] += 10 * rndn_10() * (0.01 + r2*(T2 - t) / T2);
    else         // 角度
        robot->gen_m[worst_gene][j] += rndn() * (0.01 + r2*(T2 - t) / T2);
}
else {
    // より大きな突然変異
    ...
}
```

**突然変異の特徴**:
- 世代が進むにつれて突然変異量が減少（アニーリング効果）
- x, y座標は10倍のスケール
- 角度は小さなスケール

---

## 3. 適応度関数の比較

### 3.1 fitcal_m() - 単一マップマッチング

全バージョンで共通の基本構造:

```cpp
double fitcal_m(int i, struct robot_position *robot, int** &omap, int** &amap)
{
    // レーザー点を候補姿勢で変換
    for(k=0; k<laser_size; k+=2) {
        // グローバル座標に変換
        x_3 = laser_x[k]*c1 - laser_y[k]*s1 + x_c2;
        y_3 = laser_x[k]*s1 + laser_y[k]*c1 + y_c2;

        // マップ座標に変換
        pt1_x = _map_center_x - x_3/_map_rate;
        pt1_y = _map_center_y - y_3/_map_rate;

        // 一致度を計算
        if(amap[pt1_x][pt1_y] > 0) hit++;
        else err++;

        if(cal_probability(omap[pt1_x][pt1_y]) > 0.3) hit2++;
        else if(cal_probability(omap[pt1_x][pt1_y]) < -0.3) err2++;
        else unce++;
    }

    // スコア計算
    p2 = hit / (hit + err);
    p = hit2 / (hit2 + err2 + unce);
    return p * p2;
}
```

### 3.2 fitcal_MIX() - ハイブリッドマッチング (newFuzzy1 / Tainaka_ROS)

```cpp
double fitcal_MIX(int i, struct robot_position* robot, int** &omap, int** &amap)
{
    // 環境マップとのマッチング
    fit_env = fitcal_m(i, robot, omap, amap);

    // CADマップとのマッチング
    fit_cad = fitcal_m(i, robot, omap_CAD, amap_CAD);

    // 運動モデルとの乖離度
    dis_m = sqrt(model[0]^2 + model[1]^2);
    dis_g = sqrt(robot->gen_m[i][0]^2 + robot->gen_m[i][1]^2);
    model_ratio = abs(dis_m - dis_g) / dis_m;

    // 重み付け融合
    weight = map_match_rate * (1 - model_ratio);
    fit_mix = weight * fit_env + (1 - weight) * fit_cad;

    return fit_mix;
}
```

**ハイブリッドマッチングの効果**:
- 環境変化時: CADマップの重みが増加 → 安定した位置推定
- 運動モデルと乖離: CADマップの重みが増加 → 誤推定を防止

---

## 4. マップリセット機能の比較

| バージョン | リセット閾値 | 場所 |
|-----------|-------------|------|
| backup | なし | - |
| newFuzzy1 | < 0.98 | map_mating() |
| Tainaka_ROS (クラス) | < 0.90 | map_mating() |
| Tainaka_ROS (ノード) | < 0.9988 | scanCallback() |
| es_slam | なし | - |

**リセット処理**:
```cpp
if (map_match_rate < threshold) {
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            omap[i][j] = omap_CAD[i][j];
            amap[i][j] = amap_CAD[i][j];
        }
    }
}
```

---

## 5. その他の差異

### 5.1 データ構造

| バージョン | マップ型 | メモリ管理 |
|-----------|---------|-----------|
| backup | `int**` | malloc/free |
| newFuzzy1 | `int**` | malloc/free |
| Tainaka_ROS | `int**` | malloc/free |
| es_slam | `std::vector<std::vector<int>>` | 自動管理 |

### 5.2 乱数生成

| バージョン | 乱数クラス |
|-----------|-----------|
| backup | グローバル関数 (rnd, rndn, rndn_10) |
| newFuzzy1 | グローバル関数 (rnd, rndn, rndn_10) |
| Tainaka_ROS | Randomクラス |
| es_slam | Randomクラス |

### 5.3 es_slam独自機能

1. **動的マップ拡張**: `expand_map_if_needed()`
2. **Log-odds占有格子**: `_use_logodds`オプション
3. **スキャンリサンプリング**: `ScanPointResampler`クラス
4. **可変個体数**: `set_ganm(int ganm)`

---

## 6. まとめ

### 推奨用途

| バージョン | 推奨シーン |
|-----------|-----------|
| backup | 固定環境、ホイールオドメトリ利用可能 |
| newFuzzy1 | 環境変化あり、CAD地図利用可能 |
| Tainaka_ROS | ROS環境、環境変化あり |
| es_slam | ROS環境、未知環境、CAD地図なし |

### GA-SLAMの進化

```
backup (基本形)
    │
    ├─→ newFuzzy1 (ハイブリッドマッチング追加)
    │       │
    │       └─→ Tainaka_ROS (ROS化 + 適応的r2)
    │
    └─→ es_slam (シンプル化 + 動的拡張)
```
