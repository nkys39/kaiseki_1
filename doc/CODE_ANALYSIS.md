# Kinden SLAM V1.17.0.3-1 コード解析レポート

## 1. プロジェクト概要

### 1.1 システム名称
**Kinden iRobot SLAM v1** - 照度測定ロボットシステム

### 1.2 開発元
- 首都大学東京 (Tokyo Metropolitan University)
- 開発者: Yuichiro Toda, Naoyuki Kubota, Wei Hong Chin, Kohei Oshio

### 1.3 システム目的
建設現場等における自律移動ロボットを用いた照度測定システム。SLAM（Simultaneous Localization and Mapping）技術を用いてロボットの自己位置を推定しながら、指定された測定ポイントを巡回して照度を計測する。

---

## 2. ファイル構成

```
Kinden_slam/Kinden_iRobot_slam_v1/
├── main.cpp / main.h         # メインエントリーポイント
├── Def.h                     # 定数・構造体定義
├── slam.cpp / slam.h         # SLAMアルゴリズム
├── controller.cpp / controller.h  # ロボット制御
├── localization.cpp / localization.h  # 自己位置推定
├── pathplanning.cpp / pathplanning.h  # 経路計画
├── sensing.cpp / sensing.h   # センサデータ処理
├── draw.cpp / draw.h         # 可視化処理
├── multi_resolution.cpp / multi_resolution.h  # 多解像度マップ
├── regiongrowing.cpp / regiongrowing.h  # エッジ抽出
├── myRSLidar.cpp / myRSLidar.h  # RS-LiDAR-16対応
├── random.cpp / random.h     # 乱数生成
├── malloc.cpp / malloc.h     # メモリ管理
├── illuminance.cpp / illuminance.h  # 照度計通信
├── Serial.cpp / Serial.h     # シリアル通信
├── Pipe_Comm.cpp / Pipe_Comm.h  # 名前付きパイプ通信
├── quickmail.c / quickmail.h # メール送信
├── smtpsocket.c / smtpsocket.h  # SMTP通信
├── rnd.cpp / rnd.h           # 乱数ユーティリティ
├── Setting.ini               # システム設定
├── Parameter.ini             # ユーザー設定
└── Version.rc                # バージョン情報
```

---

## 3. モジュール詳細解析

### 3.1 main.cpp - メインエントリーポイント

#### 主要機能
- システム初期化
- 設定ファイル読み込み（`Setting.ini`, `Parameter.ini`）
- LRFセンサ接続
- 各種スレッド起動
- SLAMメインループ制御

#### 主要関数
| 関数名 | 説明 |
|--------|------|
| `ini_Read()` | Setting.iniから設定読み込み |
| `readParameter()` | Parameter.iniから設定読み込み |
| `sendMail()` | 通知メール送信 |
| `illuminance_save()` | 照度データ保存 |
| `SLAM()` | SLAMメイン処理 |

#### グローバル変数
```cpp
double TR = 54.0;          // mm/pixel (地図解像度)
int width, height;         // マップサイズ
struct org_map baseMap;    // ベースマップ
struct robot_position* robot_pos;  // ロボット位置情報
```

---

### 3.2 slam.cpp - SLAMアルゴリズム

#### アルゴリズム概要
遺伝的アルゴリズム（GA）ベースの自己位置推定を実装。

#### 主要パラメータ
- `GANM = 30`: 個体数
- `GALM = 3`: 遺伝子長（x, y, θ）
- `T2 = 600`: 世代数

#### 適合度関数 `fitcal_m()`
```cpp
// LRFデータと地図の一致度を評価
// 戻り値: 0.0 ~ 1.0 (高いほど良好)
p4 = (hit率) × (確率的一致率)
```

#### 遺伝的操作
1. **交叉**: 確率70%で最良個体の遺伝子コピー
2. **突然変異**: 確率70%で微小変位を加算
3. **選択**: 適合度最低個体を置換

#### ロボットモデル `calcRobotModel()`
差動二輪駆動モデルによる移動量予測:
```cpp
dx = l * cos(w*dt) - l + h * sin(w*dt)
dy = l * sin(w*dt) + h * cos(w*dt)
dtheta = w * dt
```

---

### 3.3 controller.cpp - ロボット制御

#### 制御方式
ファジィ推論による多目的行動制御

#### 行動モジュール
1. **目標追従 (Target Tracking)**
   - 入力: 目標距離、目標角度
   - 出力: 左右車輪速度

2. **障害物回避 (Collision Avoidance)**
   - 入力: 5方向の最小距離（左, 左前, 中央, 右前, 右）
   - 出力: 左右車輪速度

#### ファジィルール
```
目標追従: 8ルール
  例) IF (angle=Negative AND dist=L) THEN (右輪=1.0, 左輪=0.6)

障害物回避: 16ルール
  例) IF (中央=近い) THEN (右輪=-0.8, 左輪=-0.8) // 後退
```

#### 多目的行動調停
```cpp
// 重み更新式
wgt[ib] = wgt[ib] + Σ(dw[ib][is] * si[is])

// 最終出力
outp = Σ(wgt[ib] * toutp[ib]) / Σ(wgt[ib])
```

---

### 3.4 localization.cpp - 初期位置推定

#### アルゴリズム
進化戦略（ES）ベースの大域的自己位置推定

#### 主要パラメータ
- `EGAN = 1000`: 個体数
- 探索範囲: `area_length` (mm)
- 角度探索: 指定角度±45度

#### 多解像度探索
粗い解像度から細かい解像度へ段階的に推定精度を向上:
```
解像度8 → 解像度4 → 解像度2 → 解像度1
```

---

### 3.5 pathplanning.cpp - 経路計画

#### アルゴリズム
ポテンシャル場法 + 遺伝的アルゴリズム

#### 評価関数
```cpp
fitness = ALPHA_DIS * 目標距離 + 経路長 + ALPHA_DANGER * 危険度 + ALPHA_UNC * 不確実性
```

#### 経路探索
1. サブゴールを設定（GAで最適化）
2. 各サブゴール間をポテンシャル場法で接続
3. ループ検出・回避機能

---

### 3.6 sensing.cpp - センサ処理

#### 対応センサ
1. **Arduino センサボード**
   - バンパーセンサ（3個）
   - 落下防止センサ（2個）
   - シャットダウン信号

#### 通信プロトコル
```
ヘッダ: 0x54 0x43 0x52 (T, C, R)
データ: バンパー(3) + 落下(4) + シャットダウン(1)
SUM: チェックサム
```

---

### 3.7 myRSLidar.cpp - RS-LiDAR-16対応

#### 仕様
- パケットサイズ: 1248バイト
- ヘッダ: 42バイト
- データ: 1200バイト（100点×12ブロック）
- 角度分解能: 0.2度
- 距離分解能: 0.5cm または 1cm

#### データ処理
```cpp
// 距離・角度変換
distance = (byte[1] << 8 | byte[0]) * 0.5  // [cm]
azimuth = (byte[3] << 8 | byte[2]) / 100.0 // [degree]
```

---

## 4. データフロー

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   LRF/LiDAR │────▶│    SLAM     │────▶│   Controller │
│   センサ    │     │  自己位置推定 │     │   行動決定  │
└─────────────┘     └─────────────┘     └──────┬──────┘
                           │                    │
                           ▼                    ▼
                    ┌─────────────┐     ┌─────────────┐
                    │  地図更新   │     │   モータ    │
                    │ (map_building)│     │   制御     │
                    └─────────────┘     └─────────────┘
                           │
                           ▼
                    ┌─────────────┐
                    │  経路計画   │
                    │(pathplanning)│
                    └─────────────┘
```

---

## 5. 構造体定義

### 5.1 robot_position
```cpp
struct robot_position {
    int map_x, map_y;        // マップ座標[pixel]
    double real_x, real_y;   // 実座標[mm]
    double rangle, dangle;   // 角度[degree]
    double model[3];         // ロボットモデル出力
    double gen_m[31][4];     // GA個体群
    double fit_m[31];        // 適合度
    int best_gene;           // 最良個体インデックス
    int slam_count;          // SLAMステップ数
};
```

### 5.2 multi_map
```cpp
struct multi_map {
    int **mmap;              // 占有マップ
    int **amap;              // 累積マップ
    double **norm;           // 正規化マップ
    double **anorm;          // 正規化累積マップ
    int reso;                // 解像度
    double rate;             // スケール率
    struct multi_map *parent, *child;  // 階層リンク
};
```

### 5.3 org_map
```cpp
struct org_map {
    int **map, **map2;       // SLAMマップ
    int **cad_map, **cad_map2;   // CAD図面マップ
    int **edge_map, **edge_map2; // エッジマップ
    int width, height;       // マップサイズ
};
```

---

## 6. 設定パラメータ

### 6.1 Setting.ini
| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| MAXSPEED | 500 | 最大速度[mm/s] |
| CONSTANTSPEED | 250 | 定速[mm/s] |
| INITTIME | 90 | 初期位置推定タイムアウト[sec] |
| SEARCHRANGE | 3000 | 初期位置検索範囲[mm] |
| WHEELDIA | 125.0 | 車輪直径[mm] |
| WHEELWIDTH | 330.0 | 車輪幅[mm] |
| RS16_ROTATION_SPEED | 300 | LiDAR回転速度[rpm] |

### 6.2 Parameter.ini
| パラメータ | 説明 |
|-----------|------|
| CONSTRUCTION_SITE | 現場名 |
| EDGE_CAD | エッジCAD使用フラグ |
| FALL_SENSOR | 落下センサ有効フラグ |
| RUN_FINISH_NOTICE | 走行完了通知 |
| MAIL_1-5 | 通知先メールアドレス |

---

## 7. 通信インターフェース

### 7.1 シリアル通信
| ポート | デバイス | ボーレート |
|--------|---------|-----------|
| COM3 | モータArduino | 115200 |
| COM4 | センサArduino | 38400 |
| COM5/6 | 照度計(T-10A) | 4800 |
| COM7 | LRF(UTM-30LX) | 115200 |

### 7.2 ネットワーク通信
| プロトコル | ポート | 用途 |
|-----------|--------|------|
| UDP | 6699 | RS-LiDAR-16 |
| TCP | 5000 | iPadコマンド受信 |
| SMTP | 587 | メール通知 |

### 7.3 名前付きパイプ
```
\\.\pipe\KINDEN_ILLM  - iPadとの通信用
```

---

## 8. ビルド環境

- **IDE**: Visual Studio 2019
- **プラットフォーム**: Windows x64
- **依存ライブラリ**:
  - OpenCV 2.x
  - URG Library (HOKUYO)
  - pthread (POSIX Threads)
  - Winsock2
  - WinInet

---

## 9. バージョン履歴

- **V1.17.0.3**: RS-LiDAR-16対応、ロボットモデル改良
- 主要改変点:
  - henkoModel: ロボット運動モデル適用
  - henkoIP/henkoIPz: LRF領域分割対応
  - henko2: エッジ抽出・CAD連携

---

## 10. 注意事項

### 10.1 ハードウェア要件
- 十分なUSBポート（センサ接続用）
- ネットワークインターフェース（LiDAR、メール用）
- USBメモリ（データ保存用）

### 10.2 動作条件
- ログディレクトリ: `C:\kinden\log\`
- コマンドディレクトリ: `C:\kinden\cmd\`
- 上記ディレクトリは事前に作成が必要

### 10.3 セキュリティ
- Parameter.iniにSMTPパスワードが平文保存されている点に注意
