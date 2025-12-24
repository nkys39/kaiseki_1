/**
 * @file resampler.cpp
 * @brief スキャン点リサンプリング機能の実装
 *
 * es_slamのScanPointResamplerを非ROS環境向けに移植
 */

#include "resampler.h"
#include <cmath>
#include <vector>

void resampleScanPoints(double* x, double* y, int size, int* validCount,
                        double dthreS, double dthreL, double ERR)
{
    std::vector<double> newX, newY;
    newX.reserve(size);
    newY.reserve(size);

    double dis = 0.0;      // 累積距離
    double prevX = ERR;
    double prevY = ERR;
    bool firstValid = true;

    for (int i = 0; i < size; i++) {
        // 無効点はスキップ
        if (x[i] == ERR || y[i] == ERR) {
            continue;
        }

        // 最初の有効点は必ず追加
        if (firstValid) {
            newX.push_back(x[i]);
            newY.push_back(y[i]);
            prevX = x[i];
            prevY = y[i];
            firstValid = false;
            continue;
        }

        // 前の点との距離を計算
        double dx = x[i] - prevX;
        double dy = y[i] - prevY;
        double L = sqrt(dx * dx + dy * dy);

        // 距離がほぼゼロの場合はスキップ
        if (L < 1e-6) {
            continue;
        }

        // Case 1: 累積距離 + 現在の距離が閾値未満 → 点を追加しない
        if (dis + L < dthreS) {
            dis += L;
            continue;
        }

        // Case 2: 累積距離 + 現在の距離が最大閾値以上 → 現在の点をそのまま追加
        if (dis + L >= dthreL) {
            newX.push_back(x[i]);
            newY.push_back(y[i]);
            prevX = x[i];
            prevY = y[i];
            dis = 0.0;
            continue;
        }

        // Case 3: 補間点を生成
        double ratio = (dthreS - dis) / L;

        // 比率の妥当性チェック
        if (ratio < 0.0) ratio = 0.0;
        if (ratio > 1.0) ratio = 1.0;

        // 補間点の座標を計算
        double nx = dx * ratio + prevX;
        double ny = dy * ratio + prevY;

        newX.push_back(nx);
        newY.push_back(ny);
        prevX = nx;
        prevY = ny;
        dis = 0.0;

        // 現在の点をもう一度処理（補間点と現在の点の間にさらに点が必要な場合）
        i--;
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
