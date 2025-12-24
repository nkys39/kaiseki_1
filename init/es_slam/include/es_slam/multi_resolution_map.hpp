#ifndef ES_SLAM_MULTI_RESOLUTION_MAP_HPP
#define ES_SLAM_MULTI_RESOLUTION_MAP_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

namespace es_slam
{

// マルチ解像度地図構造
class MultiResolutionMap
{
public:
    static constexpr int MAX_LEVELS = 6;  // 最大解像度レベル
    
    struct Level
    {
        cv::Mat occupancy_map;       // 占有格子地図
        cv::Mat normalized_map;      // 正規化地図
        double resolution;           // 解像度 [m/pixel]
        double scale_factor;         // ダウンサンプリング率
        int width;                   // 幅 [pixels]
        int height;                  // 高さ [pixels]
    };
    
    MultiResolutionMap() = default;
    
    // ROS2のOccupancyGridからマルチ解像度ピラミッドを生成
    bool createFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& grid);
    
    // 特定レベルの地図を取得
    const Level& getLevel(int level) const;
    
    // 占有確率を取得（ワールド座標）
    double getOccupancyProbability(double world_x, double world_y, int level = 0) const;
    
    // 占有確率を取得（グリッド座標）
    double getOccupancyProbabilityGrid(int grid_x, int grid_y, int level = 0) const;
    
    // グリッド座標が有効範囲内かチェック
    bool isValidGridCoordinate(int grid_x, int grid_y, int level = 0) const;
    
    // ワールド座標からグリッド座標に変換
    std::pair<int, int> worldToGrid(double world_x, double world_y, int level = 0) const;
    
    // グリッド座標からワールド座標に変換
    std::pair<double, double> gridToWorld(int grid_x, int grid_y, int level = 0) const;
    
    // レベル数を取得
    int getNumLevels() const { return levels_.size(); }
    
    // 元の地図情報を取得
    double getOriginX() const { return origin_x_; }
    double getOriginY() const { return origin_y_; }
    double getBaseResolution() const { return base_resolution_; }
    
    // デバッグ用: 各レベルの地図をOccupancyGridとして取得
    nav_msgs::msg::OccupancyGrid getLevelAsOccupancyGrid(int level) const;
    
private:
    std::vector<Level> levels_;
    double origin_x_ = 0.0;          // 地図原点X [m]
    double origin_y_ = 0.0;          // 地図原点Y [m]
    double base_resolution_ = 0.05;  // 基本解像度 [m/pixel]
    
    // OccupancyGridデータをcv::Matに変換
    cv::Mat occupancyGridToMat(const nav_msgs::msg::OccupancyGrid& grid);
    
    // 占有確率を計算 (tanh関数使用)
    double calculateOccupancyProbability(int8_t occupancy_value) const;
    
    // ダウンサンプリング処理
    cv::Mat downsample(const cv::Mat& src, double scale_factor);
    
    // 地図の正規化処理
    cv::Mat normalizeMap(const cv::Mat& occupancy_map);
    
    // Gaussian blur for smoothing
    cv::Mat applyGaussianBlur(const cv::Mat& src, int kernel_size = 3);
};

// マルチ解像度地図を使った高速マッチング
class MultiResolutionMatcher
{
public:
    struct MatchingParameters
    {
        int start_level = 3;           // 開始レベル（粗い解像度）
        int end_level = 0;             // 終了レベル（細かい解像度）
        double convergence_threshold = 0.4;  // 収束閾値
        int min_iterations_per_level = 35;   // 各レベルの最小反復回数
        bool use_gaussian_blur = true;       // Gaussianフィルタ使用
    };
    
    MultiResolutionMatcher() = default;
    
    void setParameters(const MatchingParameters& params) { params_ = params; }
    
    // マルチ解像度マッチングの実行
    bool performMultiResolutionMatching(
        const MultiResolutionMap& reference_map,
        const sensor_msgs::msg::LaserScan& scan,
        geometry_msgs::msg::Pose2D& best_pose,
        double& best_fitness);
    
    // 現在の解像度レベルを取得
    int getCurrentLevel() const { return current_level_; }
    
private:
    MatchingParameters params_;
    int current_level_ = 0;
    
    // 各レベルでのマッチング処理
    bool matchAtLevel(
        const MultiResolutionMap& reference_map,
        const sensor_msgs::msg::LaserScan& scan,
        int level,
        geometry_msgs::msg::Pose2D& pose,
        double& fitness);
};

}  // namespace es_slam

#endif  // ES_SLAM_MULTI_RESOLUTION_MAP_HPP