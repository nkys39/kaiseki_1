// File: src/scan_point_resampler.cpp
#include "es_slam/scan_point_resampler.hpp"
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <cmath>
#include <iostream>

ScanPointResampler::ScanPointResampler() 
    : dthreS(0.05)  // デフォルトの点間隔: 5cm
    , dthreL(0.25)  // デフォルトの最大間隔: 25cm
    , dis(0.0)      // 累積距離の初期化
{
}

void ScanPointResampler::setParameters(double distanceThreshold, double lengthThreshold) {
    // パラメータの妥当性チェック
    if (distanceThreshold <= 0.0) {
        throw std::invalid_argument("Distance threshold must be positive");
    }
    if (lengthThreshold <= distanceThreshold) {
        throw std::invalid_argument("Length threshold must be greater than distance threshold");
    }

    dthreS = distanceThreshold;
    dthreL = lengthThreshold;
}

void ScanPointResampler::resamplePoints(Scan2D* scan) {
    if (!scan) {
        throw std::invalid_argument("Null scan pointer provided");
    }

    std::vector<LPoint2D>& lps = scan->lps;
    if (lps.empty()) {
        return;  // 空のスキャンデータは処理しない
    }

    std::vector<LPoint2D> newLps;  // リサンプリング後の点群
    newLps.reserve(lps.size());    // メモリの事前確保で効率化

    // 累積距離の初期化
    dis = 0.0;

    // 最初の点は必ず追加
    LPoint2D prevLp = lps[0];
    LPoint2D np(prevLp.sid, prevLp.x, prevLp.y);
    newLps.push_back(np);

    // 2点目以降の処理
    for (size_t i = 1; i < lps.size(); i++) {
        const LPoint2D& currentLp = lps[i];
        bool inserted = false;

        try {
            // 補間点の探索
            if (findInterpolatePoint(currentLp, prevLp, np, inserted)) {
                newLps.push_back(np);
                prevLp = np;        // 直前点を更新
                dis = 0.0;          // 累積距離をリセット

                if (inserted) {
                    i--;            // 現在の点をもう一度処理
                    continue;
                }
            } else {
                prevLp = currentLp; // 補間点がない場合は現在の点を直前点として保存
            }
        } catch (const std::exception& e) {
            // エラーが発生した場合は警告を出して続行
            // Note: This would need logger access for RCLCPP_WARN
            // For now, using std::cerr as fallback
            std::cerr << "Warning: Error during point interpolation: " << e.what() << std::endl;
            continue;
        }
    }

    // リサンプリングされた点群で更新
    scan->setLps(newLps);
}

bool ScanPointResampler::findInterpolatePoint(
    const LPoint2D& cp,    // current point
    const LPoint2D& pp,    // previous point
    LPoint2D& np,          // new point (output)
    bool& inserted         // insertion flag (output)
) {
    // 初期化
    inserted = false;

    // 2点間の距離を計算
    double dx = cp.x - pp.x;
    double dy = cp.y - pp.y;
    double L = std::sqrt(dx*dx + dy*dy);  // 2点間の直線距離

    // 数値の安定性のためのチェック
    if (L < 1e-6) {
        return false;  // 点が重なっている場合は処理しない
    }

    // Case 1: 累積距離 + 現在の距離が閾値未満
    if (dis + L < dthreS) {
        dis += L;  // 累積距離を更新
        return false;  // 点を追加しない
    }

    // Case 2: 累積距離 + 現在の距離が最大閾値以上
    if (dis + L >= dthreL) {
        np.setData(cp.sid, cp.x, cp.y);
        return true;  // 現在の点をそのまま使用
    }

    // Case 3: 補間点を生成
    try {
        double ratio = (dthreS - dis) / L;  // 補間位置の比率
        
        if (ratio < 0.0 || ratio > 1.0) {
            throw std::runtime_error("Invalid interpolation ratio");
        }

        // 補間点の座標を計算
        double x2 = dx * ratio + pp.x;
        double y2 = dy * ratio + pp.y;

        // 新しい点を設定
        np.setData(cp.sid, x2, y2);
        inserted = true;  // 補間点が挿入されたことを示すフラグ

    } catch (const std::exception& e) {
        // エラーが発生した場合は現在の点を使用
        np.setData(cp.sid, cp.x, cp.y);
        inserted = false;
        // Note: This would need logger access for RCLCPP_WARN
        // For now, using std::cerr as fallback
        std::cerr << "Warning: Interpolation error: " << e.what() << std::endl;
    }

    return true;  // 点を追加する
}