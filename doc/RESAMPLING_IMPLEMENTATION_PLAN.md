# リサンプリング機能追加 実装方針

## 概要

es_slamのスキャン点リサンプリング機能をbackupとnewFuzzy1に追加する。
既存コードへの変更を最小限に抑え、Setting.iniでON/OFF切替可能にする。

---

## 変更ファイル一覧

### 新規追加ファイル（2ファイル）

| ファイル | 説明 |
|----------|------|
| `resampler.h` | リサンプリング関数のヘッダ（ROS非依存） |
| `resampler.cpp` | リサンプリング処理の実装 |

### 修正ファイル（4ファイル）

| ファイル | 変更内容 | 変更行数(目安) |
|----------|----------|---------------|
| `Def.h` | extern変数追加 | +5行 |
| `main.cpp` | Setting.ini読み込み追加 | +6行 |
| `slam.cpp` | リサンプリング呼び出し追加 | +10行 |
| `slam.h` | ヘッダinclude追加 | +1行 |

**合計変更**: 新規2ファイル + 既存4ファイルに約22行追加

---

## 詳細設計

### 1. Setting.ini パラメータ

```ini
[ROBOT]
RESAMPLEMODE=1          ; 0=OFF, 1=ON
RESAMPLE_DTHRES=50      ; 点間隔閾値 [mm] (デフォルト: 50mm)
RESAMPLE_DTHREL=250     ; 最大間隔閾値 [mm] (デフォルト: 250mm)
```

### 2. Def.h 追加内容

```cpp
// リサンプリング設定
extern char resampleMode;
extern double resample_dthreS;
extern double resample_dthreL;

#define RESAMPLE_MODE 0
#define RESAMPLE_DTHRES 50.0
#define RESAMPLE_DTHREL 250.0
```

### 3. main.cpp 追加内容

```cpp
// グローバル変数定義
char resampleMode;
double resample_dthreS;
double resample_dthreL;

// Setting.ini読み込み部分に追加
resampleMode = ::GetPrivateProfileInt("ROBOT", "RESAMPLEMODE", RESAMPLE_MODE, strPath);
resample_dthreS = ::GetPrivateProfileInt("ROBOT", "RESAMPLE_DTHRES", RESAMPLE_DTHRES, strPath);
resample_dthreL = ::GetPrivateProfileInt("ROBOT", "RESAMPLE_DTHREL", RESAMPLE_DTHREL, strPath);
```

### 4. resampler.h（新規）

```cpp
#ifndef RESAMPLER_H
#define RESAMPLER_H

// リサンプリング処理
// x[], y[]: 座標配列（入出力）
// size: 配列サイズ
// validCount: 有効点数（出力）
// dthreS: 点間隔閾値 [mm]
// dthreL: 最大間隔閾値 [mm]
// ERR: 無効値マーカー
void resampleScanPoints(double* x, double* y, int size, int* validCount,
                        double dthreS, double dthreL, double ERR);

#endif
```

### 5. resampler.cpp（新規）

```cpp
#include "resampler.h"
#include <cmath>
#include <vector>

void resampleScanPoints(double* x, double* y, int size, int* validCount,
                        double dthreS, double dthreL, double ERR)
{
    std::vector<double> newX, newY;
    newX.reserve(size);
    newY.reserve(size);

    double dis = 0.0;
    double prevX = ERR, prevY = ERR;
    bool firstValid = true;

    for (int i = 0; i < size; i++) {
        if (x[i] == ERR || y[i] == ERR) continue;

        if (firstValid) {
            newX.push_back(x[i]);
            newY.push_back(y[i]);
            prevX = x[i];
            prevY = y[i];
            firstValid = false;
            continue;
        }

        double dx = x[i] - prevX;
        double dy = y[i] - prevY;
        double L = sqrt(dx*dx + dy*dy);

        if (L < 1e-6) continue;

        if (dis + L < dthreS) {
            dis += L;
            continue;
        }

        if (dis + L >= dthreL) {
            newX.push_back(x[i]);
            newY.push_back(y[i]);
            prevX = x[i];
            prevY = y[i];
            dis = 0.0;
            continue;
        }

        // 補間点を生成
        double ratio = (dthreS - dis) / L;
        double nx = dx * ratio + prevX;
        double ny = dy * ratio + prevY;
        newX.push_back(nx);
        newY.push_back(ny);
        prevX = nx;
        prevY = ny;
        dis = 0.0;
        i--;  // 現在の点を再処理
    }

    // 結果を元の配列にコピー
    *validCount = (int)newX.size();
    for (int i = 0; i < size; i++) {
        if (i < *validCount) {
            x[i] = newX[i];
            y[i] = newY[i];
        } else {
            x[i] = ERR;
            y[i] = ERR;
        }
    }
}
```

### 6. slam.cpp 変更箇所

座標変換ループの直後（753行目付近）に追加:

```cpp
    for (int i = 0; i < SCAN_MARKS; i++)
    {
        if (lrf_data[i] < 100 || lrf_data[i] > 40000) {
            x[i] = ERR;
            y[i] = ERR;
        }
        else
        {
            x[i] = lrf_data[i] * sin(theta[i]);
            y[i] = lrf_data[i] * cos(theta[i]);
        }
    }

    // ★ここに追加
    if (resampleMode == 1) {
        int validCount;
        resampleScanPoints(x, y, SCAN_MARKS, &validCount,
                          resample_dthreS, resample_dthreL, ERR);
    }

    if (robot->slam_count != 0)
    {
        map_mating(robot, omap, amap);
        ...
```

### 7. slam.h 変更

```cpp
#include "resampler.h"  // 追加
```

---

## 実装手順

1. **resampler.h / resampler.cpp を作成**
   - ROS依存を排除したスタンドアロン実装
   - es_slamのロジックを移植

2. **Def.h に変数宣言を追加**

3. **main.cpp にパラメータ読み込みを追加**

4. **slam.cpp にリサンプリング呼び出しを追加**
   - `slam()` と `slam2()` の両方に追加

5. **ビルド設定を更新**
   - Visual Studioプロジェクトにresampler.cppを追加

---

## newFuzzy1への適用

backup と同じ構造のため、同じ変更を適用可能。

変更ファイル:
- `Kinden_slam_V1.17.0.3-1-gng-slam-newFuzzy1/Kinden_slam/Kinden_iRobot_slam_v1/`
  - 同じ4ファイルを修正
  - 同じ2ファイルを追加

---

## 注意点

1. **単位の違い**: es_slamは[m]、backupは[mm]
   - Setting.iniのパラメータは[mm]単位で統一

2. **配列サイズ**:
   - backup: `SCAN_MARKS` (通常1081)
   - リサンプリング後の有効点数は減少するが、配列サイズは維持

3. **slam2()関数**: RS-LiDAR用の関数にも同様の変更が必要

---

## 動作確認

1. `RESAMPLEMODE=0` で従来動作と同一であることを確認
2. `RESAMPLEMODE=1` でリサンプリングが動作することを確認
3. パラメータ変更による点群密度の変化を確認
