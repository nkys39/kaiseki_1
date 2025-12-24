# バージョン比較: V1.17.0.3-1-backup vs gng-slam-newFuzzy1

## 概要

新バージョン `gng-slam-newFuzzy1` は、旧バージョン `backup` に対して以下の主要な変更が加えられています：

1. **SLAM処理の改良**: CADマップとリアルタイムSLAMマップの**ハイブリッドマッチング**
2. **GNG (Growing Neural Gas)**: 経路計画のためのニューラルネットワーク追加
3. **ファジィ制御の刷新**: 目標追従・障害物回避の新設計

---

## 1. SLAM処理の変更（最重要）

### 1.1 新しいハイブリッドマッチング

**旧バージョン**: LRFスキャンを単一のマップ（SLAM or CAD）とマッチング

**新バージョン**: LRFスキャンを**両方のマップ**とマッチングし、重み付けで統合

```cpp
// slam.cpp: 新関数 fitcal_MIX()
double fitcal_MIX(int i, struct robot_position* robot, int** omap, int** amap) {

    // 環境マップ（リアルタイムSLAM）とマッチング
    fit_env = fitcal_m(i, robot, omap, amap);

    // CADマップとマッチング
    fit_cad = fitcal_m(i, robot, omap_CAD, amap_CAD);

    // モデル優先度による重み計算
    dis_m = sqrt((robot->model[0]^2) + (robot->model[1]^2));
    dis_g = sqrt((robot->gen_m[i][0]^2) + (robot->gen_m[i][1]^2));
    model_ratio = abs(dis_m - dis_g) / dis_m;

    // 重み付け統合
    weight = map_match_rate * (1 - model_ratio);
    fit_mix = weight * fit_env + (1 - weight) * fit_cad;

    return fit_mix;
}
```

### 1.2 マップ一致率の計算

```cpp
// slam.cpp: 新関数 compare_map()
void compare_map(int** omap, int** amap) {
    // 環境マップとCADマップの一致率を計算
    for (各ピクセル) {
        p_env = cal_probability(omap[i][j]);
        p_cad = cal_probability(omap_CAD[i][j]);

        if (p_env >= 0.8) {
            total++;
            if (p_cad >= 0.8) {
                match++;
            }
        }
    }
    map_match_rate = match / total;
}
```

### 1.3 マップリセット機能

```cpp
// slam.cpp: マップ一致率が低下した場合のリセット
if (map_match_rate < 0.98) {
    // 環境マップをCADマップにリセット
    for (全ピクセル) {
        omap[i][j] = omap_CAD[i][j];
        amap[i][j] = amap_CAD[i][j];
    }
    mapUpdate = true;
}
```

### 1.4 マッチング処理の図解

```
┌─────────────────────────────────────────────────────────────────────┐
│                    新バージョンのマッチング処理                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  現在のLRFスキャン (x[], y[])                                        │
│         │                                                            │
│         ├────────────────┬────────────────┐                         │
│         │                │                │                         │
│         ▼                ▼                ▼                         │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐                │
│  │ fitcal_m()   │ │ fitcal_m()   │ │ compare_map()│                │
│  │ 環境マップ用  │ │ CADマップ用  │ │ 一致率計算   │                │
│  │ (omap/amap)  │ │(omap_CAD/   │ │              │                │
│  │              │ │ amap_CAD)   │ │              │                │
│  └──────┬───────┘ └──────┬───────┘ └──────┬───────┘                │
│         │                │                │                         │
│         ▼                ▼                ▼                         │
│      fit_env          fit_cad       map_match_rate                  │
│         │                │                │                         │
│         └────────┬───────┘                │                         │
│                  │                        │                         │
│                  ▼                        │                         │
│  ┌───────────────────────────────────────┐│                         │
│  │ weight = map_match_rate*(1-model_ratio)│                        │
│  │ fit_mix = weight*fit_env              ││                         │
│  │         + (1-weight)*fit_cad          ││                         │
│  └───────────────────────────────────────┘│                         │
│                  │                        │                         │
│                  ▼                        ▼                         │
│           最終適合度              マップリセット判定                  │
│                                  (rate < 0.98でリセット)            │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 2. GNG (Growing Neural Gas) の追加

### 2.1 新規追加ファイル

| ファイル | 説明 |
|----------|------|
| `GNG.cpp/h` | Growing Neural Gasアルゴリズム実装 |
| `gngPlanner.cpp/h` | GNGベースのA*経路計画 |

### 2.2 GNGクラスの主要機能

```cpp
class GNG {
public:
    gngNode nodes[MAX_NODE_NUM];  // 最大4000ノード
    int num_of_node = 0;

    // 占有度を考慮した学習
    void LearningWithOccupancy(float _x, float _y, float _occupancy);

    // GNG描画
    void drawGNG(cv::Mat draw_Image);

    // 占有度の高いノードを削除
    void ExtractionOfMovingArea();
};
```

### 2.3 A*経路計画クラス

```cpp
class FastAstar {
public:
    Node nodes[MAX_NODE_NUM];
    std::vector<int> result_path;  // 経路結果

    // A*経路探索
    void AstarPathPlanningProcess();

    // 危険度・アクセシビリティ設定
    void AddNodeDanger(std::vector<std::pair<int, double>> nodes_dangerous);
    void AddNodeAccessibility(std::vector<std::pair<int, double>> nodes_accessibility);
};
```

---

## 3. ファジィ制御の刷新

### 3.1 新しい設計

| 項目 | 旧バージョン | 新バージョン |
|------|-------------|-------------|
| 目標追従入力数 | MEMNUM1 | INPUT_NUM_TT = 1 |
| 目標追従ルール数 | RULENUM1 | RULE_NUM_TT = 5 |
| 障害物回避入力数 | MEMNUM | INPUT_NUM_CA = 13 |
| 障害物回避ルール数 | RULENUM | RULE_NUM_CA = 13 |
| 行動数 | - | BEHVNUM = 2 |
| 出力数 | - | OUTPUT_NUM = 2 |

### 3.2 新しい関数構成

```cpp
// controller.cpp 新関数
// 目標追従ファジィ (Target Trace)
double membershipTT(int rule_num, double angle);
void fuzzyTT(double angle, double output[2]);

// 障害物回避ファジィ (Collision Avoidance)
double membershipCA(int rule_num, double* min_len);
void fuzzyCA(double* min_len, double* output);

// 多目的行動調停
void init_mobc();
void Update_wgt(double si[BEHVNUM]);
void cal_outp(double outp[OUTPUT_NUM], double toutp[BEHVNUM][OUTPUT_NUM]);
```

### 3.3 GNG経路計画連携

```cpp
// controller.cpp
int gng_path_idx = 0;  // GNGプランニングの中継点ID
int gng_path[MAX_MEASURE_POINT][2];  // 中継点配列
```

---

## 4. main.cpp の変更

### 4.1 新規インクルード

```cpp
#include "GNG.h"
#include "gngPlanner.h"
```

### 4.2 新規グローバル変数

```cpp
GNG *gng;
FastAstar *pp;
```

### 4.3 設定の変更

```cpp
// 新バージョンではマクロで定義
#define MAPFILENAME "1F01_new.png"
#define PATHFILENAME "C:\\kinden\\map\\path2-Building1-1to2mtg.txt"
```

---

## 5. ファイル構成の変更

### 5.1 新規追加ファイル

| ファイル | 説明 |
|----------|------|
| `GNG.cpp` | Growing Neural Gas実装 |
| `GNG.h` | GNGヘッダー |
| `gngPlanner.cpp` | A*経路計画実装 |
| `gngPlanner.h` | 経路計画ヘッダー |
| `slam_org.cpp` | 旧SLAMバックアップ |
| `rnd.cpp/h` | 乱数生成器 |

### 5.2 コード量の変化

| ファイル | 旧バージョン | 新バージョン | 差分 |
|----------|-------------|-------------|------|
| slam.cpp | 1068行 | 1301行 | +233行 |
| controller.cpp | 大幅変更 | 完全刷新 | - |

---

## 6. まとめ

### 主な改良点

1. **SLAM精度向上**: CADマップとSLAMマップの両方を活用したハイブリッドマッチング
2. **経路計画強化**: GNGによる適応的な経路ネットワーク構築
3. **制御刷新**: より洗練されたファジィ制御と多目的行動調停

### 技術的意義

- **map_match_rate** によりSLAMマップの信頼度を動的に評価
- 信頼度が低下した場合にCADマップへリセットする自動復帰機能
- GNGによる環境学習と動的経路計画の統合

### 注意点

- 新バージョンでは `apply` フラグでハイブリッドマッチングのON/OFFが可能
- `omap_CAD`, `amap_CAD` の初期化が必要
