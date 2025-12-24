#ifndef RESAMPLER_H
#define RESAMPLER_H

/**
 * @file resampler.h
 * @brief スキャン点リサンプリング機能
 *
 * es_slamのScanPointResamplerを非ROS環境向けに移植
 * backup/newFuzzy1で使用可能
 */

/**
 * @brief スキャン点のリサンプリング処理
 *
 * 点群データを一定間隔でリサンプリングし、点数を削減する。
 * 元の配列を直接書き換え、リサンプリング後の有効点数を返す。
 *
 * @param x X座標配列 [mm]（入出力）
 * @param y Y座標配列 [mm]（入出力）
 * @param size 配列サイズ
 * @param validCount リサンプリング後の有効点数（出力）
 * @param dthreS 点間隔閾値 [mm]（この間隔以上で点を追加）
 * @param dthreL 最大間隔閾値 [mm]（この間隔以上で強制追加）
 * @param ERR 無効値マーカー
 */
void resampleScanPoints(double* x, double* y, int size, int* validCount,
                        double dthreS, double dthreL, double ERR);

#endif // RESAMPLER_H
