# モジュール詳細解析

## 1. SLAMモジュール (slam.cpp)

### 1.1 概要
遺伝的アルゴリズム（GA）を用いた逐次自己位置推定モジュール。

### 1.2 アルゴリズムフロー

```
1. 初期化 (slam_count == 0)
   └── 遺伝子初期化、角度テーブル作成

2. LRFデータ前処理
   └── 距離100mm〜40000mmのデータのみ使用
   └── 極座標→直交座標変換

3. 遺伝的操作 (map_mating)
   ├── mga_init: 個体群初期化
   │   ├── 個体0: 前回最良解保持
   │   ├── 個体1: 原点(0,0,0)
   │   ├── 個体2: 前回最良解の反転
   │   └── 個体3〜29: ロボットモデル+ランダム
   │
   └── 世代ループ (600回)
       ├── 最良・最悪個体選択
       ├── 交叉 (70%: 最良コピー, 30%: ブレンド)
       ├── 突然変異 (70%: 小変動, 30%: 大変動)
       └── 適合度評価 → 最悪個体置換

4. 位置更新
   └── 最良個体の遺伝子を位置に反映
```

### 1.3 適合度関数

```cpp
double fitcal_m(int i, robot_position *robot, int **omap, int **amap)
{
    // 遺伝子iの座標変換
    x_c2 = robot->real_x + gen[i][0]*c1 - gen[i][1]*s1;
    y_c2 = robot->real_y + gen[i][1]*c1 + gen[i][0]*s1;

    for (各LRFポイント) {
        // ワールド座標変換
        x_3 = x * c1 - y * s1 + x_c2;
        y_3 = x * s1 + y * c1 + y_c2;

        // マップ座標変換
        pt_x = map_center - x_3 / map_rate;
        pt_y = map_center - y_3 / map_rate;

        // amap (累積マップ) でヒット判定
        if (amap[pt_x][pt_y] > 0) hit++;

        // omap (確率マップ) で確率判定
        if (cal_probability(omap) > 0.3) hit2++;
    }

    // 複合適合度
    return (hit2 / total) * (hit / total);
}
```

### 1.4 ロボットモデル

差動二輪駆動モデル:

```
入力: outp[0] (右輪速度), outp[1] (左輪速度)

計算:
  wL = alpha[0] * outp[1]  // 左輪角速度
  wR = alpha[1] * outp[0]  // 右輪角速度

  vL = (r/2) * wL          // 左輪速度
  vR = (r/2) * wR          // 右輪速度

  v = (vL + vR) / 2        // 並進速度
  w = (vR - vL) / d        // 角速度

  if (|vR - vL| < ε):      // 直進
    dx = 0, dy = v*dt, dθ = 0
  else:                     // 旋回
    l = v / w              // 旋回半径
    dx = l*cos(w*dt) - l + h*sin(w*dt)
    dy = l*sin(w*dt) + h*cos(w*dt)
    dθ = w*dt * 180/π
```

---

## 2. コントローラモジュール (controller.cpp)

### 2.1 ファジィ推論ルール

#### 目標追従ルール
```
frth1[8][2] = {
    { 0.4, -0.2 },  // Rule 0: N & S (後方遠い) → 右回転
    { 0,    0   },  // Rule 2: N & M (後方中間) → 停止
    { 0.6,  0.2 },  // Rule 3: N & ML → 左回転
    { 1.0,  0.6 },  // Rule 4: N & L (後方近い) → 前進左回転
    {-0.2,  0.4 },  // Rule 5: P & S → 左回転
    { 0,    0   },  // Rule 7: P & M → 停止
    { 0.2,  0.6 },  // Rule 8: P & ML → 右回転
    { 0.6,  1.0 }   // Rule 9: P & L (前方近い) → 前進右回転
};
```

#### 障害物回避ルール
```
frth[16][2] = {
    { 0.8,  0.8 },  // Rule 1: 障害物なし → 前進
    { 0.8,  0.2 },  // Rule 2: 左障害物 → 右回転
    { 0.8, -0.8 },  // Rule 3: 左前障害物 → 強右回転
    {-0.8, -0.8 },  // Rule 4: 正面障害物 → 後退(ランダム方向)
    {-0.8,  0.8 },  // Rule 5: 右前障害物 → 強左回転
    { 0.2,  0.8 },  // Rule 6: 右障害物 → 左回転
    ...
};
```

### 2.2 多目的行動調停

```cpp
// 環境知覚情報
si[0] = exp(-(si[0] - 1.0)^2 * 4.0)  // 障害物回避の重要度
si[1] = 1.0 - si[0]                   // 目標追従の重要度

// 重み更新行列
dw = {
    { 0.9, 0.1 },  // 障害物優先
    { 0.1, 0.9 }   // 目標優先
};

// 重み更新
for (ib = 0..1):
    wgt[ib] += Σ(dw[ib][is] * si[is])

// 正規化
wgt[ib] /= Σwgt

// 最終出力
outp = Σ(wgt * toutp) / Σwgt
outp *= (min_dis / MAX_DIS) * maxSpeed  // 距離に応じた減速
```

### 2.3 ループ検出

```cpp
// 目標1m以内でループ検出開始
if (det < 1000) {
    dist_robot = 現在位置 - 前回位置
    angle = 3点の角度計算
    sumAngle += angle

    if (sumAngle >= 360 || 経過時間 > 60秒) {
        loop_detect = 1  // ループ検出
        次の目標へ移動
    }
}
```

---

## 3. 自己位置推定モジュール (localization.cpp)

### 3.1 進化戦略 (ES)

```
個体数: EGAN = 1000
遺伝子: (x, y, θ)

初期化:
  個体0: 指定位置
  個体1〜999: 指定位置 ± (area_length / 2) * random

評価:
  fitcal_p(): 多解像度マップとの一致度

世代更新:
  if (適合度 < 0.3):
    再初期化
  else:
    遺伝的操作で子個体生成
    最悪個体と置換

終了条件:
  適合度 > 0.4 AND 世代 > 10 AND カウント > 35
```

### 3.2 多解像度探索

```
解像度8マップ → 粗い探索
    ↓ 収束
解像度4マップ → 中間探索
    ↓ 収束
解像度2マップ → 詳細探索
    ↓ 収束
解像度1マップ → 最終推定
```

### 3.3 スケール調整

```cpp
// LRFスケール最適化
void addjust_scale_multi(...) {
    // GAでスケール係数を探索
    gen_s[i][0] = 1.0 ± 0.5 * random

    // 評価: スケール係数を適用してLRFデータ変換
    x = x_lrf * scale
    y = y_lrf * scale

    // マップとの一致度で評価
}
```

---

## 4. 経路計画モジュール (pathplanning.cpp)

### 4.1 ポテンシャル場法

```cpp
int path_planning(x_s, y_s, x_g, y_g, path, multi_Map) {
    while (true) {
        min_E = MAX;

        // 8近傍探索
        for (i = -1..1, j = -1..1) {
            dis = (x_g - curr_x+i)^2 + (y_g - curr_y+j)^2
            e = norm[curr_x+i][curr_y+j] / (dis + 0.001)

            if (e < min_E) {
                min_E = e
                dir = (i, j)
            }
        }

        // 移動
        curr += dir
        path[ct++] = curr

        if (curr == goal) break
        if (振動検出) break  // ループ回避
    }
}
```

### 4.2 GA経路最適化

```cpp
// 評価関数
fitness = ALPHA_DIS * 目標距離
        + 経路長
        + ALPHA_DANGER * 危険領域通過量
        + ALPHA_UNC * 不確実領域通過量

// 遺伝子: サブゴール列
gen[i] = [サブゴール数, x1, y1, x2, y2, ...]

// 交叉・突然変異でサブゴール最適化
```

---

## 5. センサモジュール (sensing.cpp)

### 5.1 Arduinoプロトコル

```
送信フォーマット: なし（受信のみ）

受信フォーマット (18バイト):
  [0]    : 0x54 ('T')        ヘッダ1
  [1]    : 0x43 ('C')        ヘッダ2
  [2]    : 0x52 ('R')        ヘッダ3
  [3]    : バンパー1 (ASCII)
  [4]    : バンパー2 (ASCII)
  [5]    : バンパー3 (ASCII)
  [6]    : 予約
  [7-8]  : 落下センサ1 (ASCII 2桁)
  [9-10] : 落下センサ2 (ASCII 2桁)
  [11]   : 予約
  [12]   : シャットダウン信号
  [13-14]: 予約
  [15]   : SUMチェック
  [16-17]: 予約
```

### 5.2 シャットダウン処理

```cpp
if (recievedData[12] == 0x00) {
    // 権限取得
    EnablePrivileges(SE_SHUTDOWN_NAME, TRUE)
    // Windows終了
    ExitWindowsEx(EWX_SHUTDOWN | EWX_FORCE, 0)
}
```

---

## 6. RS-LiDAR-16モジュール (myRSLidar.cpp)

### 6.1 パケット構造

```
UDPパケット (1248バイト):
  ├── Header (42バイト): イーサネット/IP/UDPヘッダ
  ├── FiringData (1200バイト): 距離・角度データ
  │   └── 12ブロック × 100バイト
  │       └── 16チャンネル × 3バイト (距離2B + 強度1B)
  └── Tail (6バイト): タイムスタンプ等
```

### 6.2 データ変換

```cpp
// 距離変換
distance = ((byte[1] << 8) | byte[0]) * 0.005  // [m]

// 角度変換
azimuth = ((byte[3] << 8) | byte[2]) / 100.0   // [degree]

// 直交座標変換
x = distance * sin(azimuth)
y = distance * cos(azimuth)
```

### 6.3 チャンネル角度補正

```cpp
const float Calibration_Angle[16] = {
    -15.0161, -13.0276, -11.0066, -9.0274,
    -7.0263,  -5.0078,  -3.0124,  -1.0097,
    14.9693,  12.98,    10.9928,   8.9785,
    6.9769,   4.9935,   2.9981,    0.9954
};
```

---

## 7. 描画モジュール (draw.cpp)

### 7.1 ロボット描画

```cpp
void draw_robot(robot_position *robot, double robot_size, ...) {
    // 現在位置
    rpt = (robot->map_x, robot->map_y)

    // 方向表示（V字形状）
    rpt2 = 回転行列適用(-size, size)
    rpt3 = 回転行列適用(+size, size)

    cvLine(rpt → rpt2)  // 左線
    cvLine(rpt → rpt3)  // 右線
}
```

### 7.2 GA結果可視化 (miniMap)

```
┌────────────────────────────────────┐
│ t=600          [凡例]              │
│ gen=(x,y)      □ Pos_best(t-1)    │
│ fitness=0.85   □ Pos_cand(t)      │
│ output=(L,R)   □ Pos_best(t)      │
│                □ Pos_worst(t)     │
│     ┌────────────────────┐        │
│     │   グリッド表示      │        │
│     │   矢印: 個体位置   │        │
│     └────────────────────┘        │
└────────────────────────────────────┘
```

---

## 8. エッジ抽出モジュール (regiongrowing.cpp)

### 8.1 処理フロー

```
1. CAD画像読み込み
2. マウスで初期位置選択
3. 領域拡張法でエッジ抽出
   - 4近傍探索（下→右→上→左）
   - 閾値: THcolor = 200
4. エッジ画像保存
```

### 8.2 領域拡張アルゴリズム

```cpp
void searchRoom(int px, int py) {
    stack<Point> stk;
    stk.push(初期位置);

    while (!stk.empty()) {
        p = stk.top(); stk.pop();

        for (4近傍) {
            if (画素値 > THcolor && 未訪問) {
                青色マーク
                stk.push(隣接点)
            }
        }
    }
}
```
