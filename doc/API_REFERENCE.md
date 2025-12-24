# API リファレンス

## 1. SLAMモジュール (slam.h)

### 1.1 slam()
メインSLAM処理関数（UTM-30LX用）

```cpp
void slam(struct robot_position *robot,
          long *lrf_data,
          int **omap,
          int **amap,
          double *outp);
```

**パラメータ:**
| 名前 | 型 | 説明 |
|------|-----|------|
| robot | robot_position* | ロボット位置構造体 |
| lrf_data | long* | LRFデータ配列 (SCAN_MARKS要素) |
| omap | int** | 占有マップ |
| amap | int** | 累積マップ |
| outp | double* | モータ出力値 [右, 左] |

**処理:**
1. LRFデータを直交座標に変換
2. 遺伝的アルゴリズムで自己位置推定
3. ロボット位置・角度を更新
4. 適合度ログ出力

---

### 1.2 slam2()
RS-LiDAR-16用SLAM処理関数

```cpp
void slam2(struct robot_position* robot,
           long* lrf_dataD,
           float* lrf_dataA,
           int** omap,
           int** amap,
           double* outp);
```

**パラメータ:**
| 名前 | 型 | 説明 |
|------|-----|------|
| lrf_dataD | long* | 距離データ配列 |
| lrf_dataA | float* | 角度データ配列 |

---

### 1.3 map_building2()
地図更新関数

```cpp
void map_building2(struct robot_position *robot,
                   int **omap,
                   int **amap,
                   IplImage *omap_Image,
                   IplImage *amap_Image);
```

**処理:**
- レイトレーシングによる占有確率更新
- Bresenhamアルゴリズムでグリッド走査
- 障害物検出点で確率増加、空きセルで確率減少

---

### 1.4 initialize_RobotPosition()
ロボット位置初期化

```cpp
void initialize_RobotPosition(struct robot_position *robot, int dflag);
```

**パラメータ:**
| dflag | 動作 |
|-------|------|
| 0 | 座標を原点に設定 |
| 1 | 座標をERR値に設定 |

---

### 1.5 transformation_angle()
角度正規化関数

```cpp
double transformation_angle(double theta1);
```

**戻り値:** -180° ~ +180° に正規化された角度

---

### 1.6 calcRobotModel()
ロボット運動モデル計算

```cpp
void calcRobotModel(double* m);
```

**出力:**
- m[0]: Δx [mm]
- m[1]: Δy [mm]
- m[2]: Δθ [degree]

---

## 2. コントローラモジュール (controller.h)

### 2.1 decision_making()
行動決定関数（UTM-30LX用）

```cpp
int decision_making(struct robot_position *robot,
                    double *foutp,
                    long *lrfdata,
                    int path[][2],
                    int *sflag,
                    int pct,
                    int mode);
```

**パラメータ:**
| 名前 | 型 | 説明 |
|------|-----|------|
| foutp | double* | 出力速度 [右, 左, 0, 0] |
| lrfdata | long* | LRFデータ |
| path | int[][2] | 経路点配列 |
| sflag | int* | 測定完了フラグ |
| pct | int | 経路点総数 |
| mode | int | 動作モード |

**戻り値:**
| 値 | 意味 |
|----|------|
| 9 | 全経路完了 |
| mode | 継続 |

---

### 2.2 decision_making2()
行動決定関数（RS-LiDAR-16用）

```cpp
int decision_making2(struct robot_position* robot,
                     double* foutp,
                     long* lrf_dataD,
                     float* lrf_dataA,
                     int path[][2],
                     int* sflag,
                     int pct,
                     int mode);
```

---

## 3. 自己位置推定モジュール (localization.h)

### 3.1 initial_Localization()
初期位置推定関数（UTM-30LX用）

```cpp
struct multi_map *initial_Localization(
    struct es_gene *gene,
    struct robot_position *robot,
    long *lrf_data,
    int **omap,
    struct multi_map *multi_Map,
    IplImage *mImage,
    int *end_flag);
```

**パラメータ:**
| 名前 | 型 | 説明 |
|------|-----|------|
| gene | es_gene* | ES用遺伝子構造体 |
| multi_Map | multi_map* | 現在の解像度マップ |
| mImage | IplImage* | 可視化画像（NULLで無効） |
| end_flag | int* | 終了フラグ（1で推定完了） |

**戻り値:** 次の解像度マップポインタ

---

### 3.2 initial_Localization2()
初期位置推定関数（RS-LiDAR-16用）

```cpp
struct multi_map* initial_Localization2(
    struct es_gene* gene,
    struct robot_position* robot,
    long* lrf_dataD,
    float* lrf_dataA,
    int** omap,
    struct multi_map* multi_Map,
    IplImage* mImage,
    int* end_flag);
```

---

### 3.3 addjust_scale_multi()
スケール調整関数

```cpp
void addjust_scale_multi(struct robot_position *robot,
                         long *lrf_data,
                         struct multi_map *multi_Map);
```

**処理:**
- LRFデータのスケール係数を最適化
- GAで0.5〜1.5の範囲で探索

---

## 4. 経路計画モジュール (pathplanning.h)

### 4.1 path_main()
経路計画メイン関数

```cpp
int path_main(int x_s, int y_s,
              int x_g, int y_g,
              struct multi_map *multi_Map,
              int fpath[][2]);
```

**パラメータ:**
| 名前 | 型 | 説明 |
|------|-----|------|
| x_s, y_s | int | 開始座標 [pixel] |
| x_g, y_g | int | 目標座標 [pixel] |
| fpath | int[][2] | 経路点出力配列 |

**戻り値:** 経路点数

---

## 5. センサモジュール (sensing.h)

### 5.1 sensorComm_Init()
センサ通信初期化

```cpp
int sensorComm_Init(void);
```

**戻り値:**
| 値 | 意味 |
|----|------|
| 0 | 成功 |
| -1 | エラー |

---

### 5.2 SensorDataReceive()
センサデータ受信スレッド

```cpp
void SensorDataReceive(void);
```

**更新するグローバル変数:**
- bumper[3]: バンパーセンサ
- falling[2]: 落下センサ

---

### 5.3 Shutdown()
システムシャットダウン

```cpp
int Shutdown(void);
```

**戻り値:**
| 値 | 意味 |
|----|------|
| 0 | 成功 |
| 1 | 失敗 |

---

## 6. 描画モジュール (draw.h)

### 6.1 draw_robot()
ロボット描画

```cpp
void draw_robot(struct robot_position *robot,
                double robot_size,
                IplImage *draw_Image,
                int R, int G, int B);
```

---

### 6.2 draw_LRFdata()
LRFデータ描画（UTM-30LX用）

```cpp
void draw_LRFdata(int mode,
                  struct robot_position *robot,
                  long *lrf_data,
                  IplImage *drawImage,
                  int R, int G, int B);
```

**modeパラメータ:**
| 値 | 動作 |
|----|------|
| 0 | 初期位置推定時（全データ表示） |
| 1 | SLAM中（フィルタ適用） |

---

### 6.3 draw_gen()
GA結果可視化

```cpp
void draw_gen(int mode,
              struct robot_position *robot,
              int step,
              double *outp);
```

**modeパラメータ:**
| 値 | 動作 |
|----|------|
| 0 | 基準位置設定 |
| 1 | 個体群描画 |
| 2 | 最終結果描画 |

---

## 7. 多解像度モジュール (multi_resolution.h)

### 7.1 reduce_resolution()
多解像度マップ生成

```cpp
struct multi_map *reduce_resolution(struct multi_map *root_map,
                                    int **omap);
```

**戻り値:** 最低解像度マップへのポインタ

---

## 8. エッジ抽出モジュール (regiongrowing.h)

### 8.1 getEdge()
CAD画像からエッジ抽出

```cpp
int getEdge(char *boxPath,
            char *cadPath,
            char *edgePath,
            int *init_data);
```

**パラメータ:**
| 名前 | 型 | 説明 |
|------|-----|------|
| boxPath | char* | Box保存パス |
| cadPath | char* | 入力CAD画像パス |
| edgePath | char* | 出力エッジ画像パス |
| init_data | int* | 初期位置データ [x, y, angle] |

---

## 9. RS-LiDARモジュール (myRSLidar.h)

### 9.1 RS_Initial()
RS-LiDAR-16接続初期化

```cpp
bool RS_Initial();
```

---

### 9.2 startThread()
受信スレッド開始

```cpp
void startThread();
```

---

### 9.3 endThread()
受信スレッド終了

```cpp
void endThread();
```

---

## 10. ユーティリティ (random.h)

### 10.1 rnd()
一様乱数生成

```cpp
double rnd();
```

**戻り値:** 0.0 ~ 1.0

---

### 10.2 rndn()
正規乱数生成

```cpp
double rndn();
```

**戻り値:** 標準正規分布

---

### 10.3 rndn_10()
スケール付き正規乱数

```cpp
double rndn_10();
```

**戻り値:** 標準偏差0.1の正規分布

---

### 10.4 cal_probability()
占有確率計算

```cpp
double cal_probability(int k);
```

**パラメータ:** k - 累積観測カウント

**戻り値:** -1.0 ~ 1.0 (負=空き、正=障害物)

---

## 11. 通信関連

### 11.1 Serial.h

```cpp
// モータ制御
int controlMotor(int leftSpeed, int rightSpeed);

// シリアルポート初期化
int serialInit(int port, int baudrate);

// データ送信
int serialSend(HANDLE hCom, char* data, int len);
```

### 11.2 Pipe_Comm.h

```cpp
// パイプ接続
HANDLE pipeConnect(char* pipeName);

// コマンド受信
int pipeReceive(HANDLE hPipe, char* buffer);

// ステータス送信
void StatusSet(char status);
```

### 11.3 illuminance.h

```cpp
// 照度計初期化
int illumInit(int port);

// 照度データ取得
double getIlluminance();
```
