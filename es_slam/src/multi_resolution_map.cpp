#include "es_slam/multi_resolution_map.hpp"
#include "es_slam/global_localization.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>

namespace es_slam
{

bool MultiResolutionMap::createFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& grid)
{
    // 元の地図情報を保存
    origin_x_ = grid.info.origin.position.x;
    origin_y_ = grid.info.origin.position.y;
    base_resolution_ = grid.info.resolution;
    
    // OccupancyGridをcv::Matに変換
    cv::Mat base_map = occupancyGridToMat(grid);
    
    if (base_map.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("multi_resolution_map"), "Failed to convert OccupancyGrid to Mat");
        return false;
    }
    
    // マルチ解像度ピラミッドを生成
    levels_.clear();
    
    for (int level = 0; level < MAX_LEVELS; ++level) {
        Level map_level;
        
        // スケールファクターを計算（2のべき乗）
        map_level.scale_factor = std::pow(2.0, level);
        map_level.resolution = base_resolution_ * map_level.scale_factor;
        
        if (level == 0) {
            // ベースレベル
            map_level.occupancy_map = base_map.clone();
        } else {
            // ダウンサンプリング
            map_level.occupancy_map = downsample(levels_[level - 1].occupancy_map, 2.0);
        }
        
        // Gaussianフィルタでスムージング（レベル1以上）
        if (level > 0) {
            map_level.occupancy_map = applyGaussianBlur(map_level.occupancy_map, 3);
        }
        
        // 正規化マップを生成
        map_level.normalized_map = normalizeMap(map_level.occupancy_map);
        
        map_level.width = map_level.occupancy_map.cols;
        map_level.height = map_level.occupancy_map.rows;
        
        levels_.push_back(map_level);
        
        // マップが小さくなりすぎたら終了
        if (map_level.width < 10 || map_level.height < 10) {
            break;
        }
    }
    
    RCLCPP_INFO(rclcpp::get_logger("multi_resolution_map"), 
                "Created multi-resolution map with %zu levels", levels_.size());
    
    return true;
}

const MultiResolutionMap::Level& MultiResolutionMap::getLevel(int level) const
{
    if (level < 0 || level >= static_cast<int>(levels_.size())) {
        throw std::out_of_range("Invalid level index");
    }
    return levels_[level];
}

double MultiResolutionMap::getOccupancyProbability(double world_x, double world_y, int level) const
{
    auto [grid_x, grid_y] = worldToGrid(world_x, world_y, level);
    return getOccupancyProbabilityGrid(grid_x, grid_y, level);
}

double MultiResolutionMap::getOccupancyProbabilityGrid(int grid_x, int grid_y, int level) const
{
    if (!isValidGridCoordinate(grid_x, grid_y, level)) {
        return 0.5;  // Unknown
    }
    
    const Level& map_level = levels_[level];
    float value = map_level.normalized_map.at<float>(grid_y, grid_x);
    
    // tanh関数で確率に変換
    return calculateOccupancyProbability(static_cast<int8_t>(value * 100));
}

bool MultiResolutionMap::isValidGridCoordinate(int grid_x, int grid_y, int level) const
{
    if (level < 0 || level >= static_cast<int>(levels_.size())) {
        return false;
    }
    
    const Level& map_level = levels_[level];
    return grid_x >= 0 && grid_x < map_level.width &&
           grid_y >= 0 && grid_y < map_level.height;
}

std::pair<int, int> MultiResolutionMap::worldToGrid(double world_x, double world_y, int level) const
{
    if (level < 0 || level >= static_cast<int>(levels_.size())) {
        return {-1, -1};
    }
    
    const Level& map_level = levels_[level];
    
    // 地図原点からの相対位置
    double relative_x = world_x - origin_x_;
    double relative_y = world_y - origin_y_;
    
    // グリッド座標に変換
    int grid_x = static_cast<int>(std::floor(relative_x / map_level.resolution));
    int grid_y = static_cast<int>(std::floor(relative_y / map_level.resolution));
    
    return {grid_x, grid_y};
}

std::pair<double, double> MultiResolutionMap::gridToWorld(int grid_x, int grid_y, int level) const
{
    if (level < 0 || level >= static_cast<int>(levels_.size())) {
        return {0.0, 0.0};
    }
    
    const Level& map_level = levels_[level];
    
    // グリッド中心のワールド座標
    double world_x = origin_x_ + (grid_x + 0.5) * map_level.resolution;
    double world_y = origin_y_ + (grid_y + 0.5) * map_level.resolution;
    
    return {world_x, world_y};
}

nav_msgs::msg::OccupancyGrid MultiResolutionMap::getLevelAsOccupancyGrid(int level) const
{
    nav_msgs::msg::OccupancyGrid grid;
    
    if (level < 0 || level >= static_cast<int>(levels_.size())) {
        return grid;
    }
    
    const Level& map_level = levels_[level];
    
    // ヘッダー設定
    grid.header.stamp = rclcpp::Clock().now();
    grid.header.frame_id = "map";
    
    // 地図情報設定
    grid.info.resolution = map_level.resolution;
    grid.info.width = map_level.width;
    grid.info.height = map_level.height;
    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    
    // データコピー
    grid.data.resize(map_level.width * map_level.height);
    
    for (int y = 0; y < map_level.height; ++y) {
        for (int x = 0; x < map_level.width; ++x) {
            float value = map_level.occupancy_map.at<float>(y, x);
            
            if (value < 0) {
                grid.data[y * map_level.width + x] = -1;  // Unknown
            } else {
                grid.data[y * map_level.width + x] = static_cast<int8_t>(value * 100);
            }
        }
    }
    
    return grid;
}

cv::Mat MultiResolutionMap::occupancyGridToMat(const nav_msgs::msg::OccupancyGrid& grid)
{
    cv::Mat mat(grid.info.height, grid.info.width, CV_32FC1);
    
    for (unsigned int y = 0; y < grid.info.height; ++y) {
        for (unsigned int x = 0; x < grid.info.width; ++x) {
            int index = y * grid.info.width + x;
            int8_t value = grid.data[index];
            
            if (value < 0) {
                mat.at<float>(y, x) = -1.0f;  // Unknown
            } else {
                mat.at<float>(y, x) = value / 100.0f;  // [0, 1]に正規化
            }
        }
    }
    
    return mat;
}

double MultiResolutionMap::calculateOccupancyProbability(int8_t occupancy_value) const
{
    if (occupancy_value < 0) {
        return 0.5;  // Unknown
    }
    
    // tanh関数で占有確率を計算（参照実装と同じ）
    double k = occupancy_value / 10.0;
    return std::tanh(k);
}

cv::Mat MultiResolutionMap::downsample(const cv::Mat& src, double scale_factor)
{
    cv::Mat dst;
    cv::resize(src, dst, cv::Size(), 1.0 / scale_factor, 1.0 / scale_factor, cv::INTER_AREA);
    return dst;
}

cv::Mat MultiResolutionMap::normalizeMap(const cv::Mat& occupancy_map)
{
    cv::Mat normalized = occupancy_map.clone();
    
    // -1（未知）の値を保持しながら正規化
    for (int y = 0; y < normalized.rows; ++y) {
        for (int x = 0; x < normalized.cols; ++x) {
            float value = normalized.at<float>(y, x);
            
            if (value >= 0) {
                // 占有確率をtanh関数で変換
                float prob = calculateOccupancyProbability(static_cast<int8_t>(value * 100));
                normalized.at<float>(y, x) = prob;
            }
        }
    }
    
    return normalized;
}

cv::Mat MultiResolutionMap::applyGaussianBlur(const cv::Mat& src, int kernel_size)
{
    cv::Mat dst;
    cv::Mat mask = (src >= 0);  // 既知の領域のみ
    
    // 既知の領域のみにGaussianフィルタを適用
    cv::Mat src_known = src.clone();
    src_known.setTo(0.5, src < 0);  // 未知領域を中間値に
    
    cv::GaussianBlur(src_known, dst, cv::Size(kernel_size, kernel_size), 0);
    
    // 未知領域を復元
    dst.setTo(-1, ~mask);
    
    return dst;
}

// ========== MultiResolutionMatcher Implementation ==========

bool MultiResolutionMatcher::performMultiResolutionMatching(
    const MultiResolutionMap& reference_map,
    const sensor_msgs::msg::LaserScan& scan,
    geometry_msgs::msg::Pose2D& best_pose,
    double& best_fitness)
{
    best_fitness = 0.0;
    
    // 粗い解像度から開始
    for (int level = params_.start_level; level >= params_.end_level; --level) {
        current_level_ = level;
        
        if (level >= reference_map.getNumLevels()) {
            continue;
        }
        
        // 現在のレベルでマッチング
        double level_fitness;
        if (!matchAtLevel(reference_map, scan, level, best_pose, level_fitness)) {
            RCLCPP_WARN(rclcpp::get_logger("multi_resolution_matcher"),
                       "Matching failed at level %d", level);
            continue;
        }
        
        // 適応度更新
        if (level_fitness > best_fitness) {
            best_fitness = level_fitness;
        }
        
        // 収束判定
        if (best_fitness > params_.convergence_threshold) {
            RCLCPP_INFO(rclcpp::get_logger("multi_resolution_matcher"),
                       "Converged at level %d with fitness %.3f", level, best_fitness);
            
            // より細かい解像度へ
            if (level > params_.end_level) {
                continue;
            } else {
                return true;
            }
        }
    }
    
    return best_fitness > params_.convergence_threshold;
}

bool MultiResolutionMatcher::matchAtLevel(
    const MultiResolutionMap& reference_map,
    const sensor_msgs::msg::LaserScan& scan,
    int level,
    geometry_msgs::msg::Pose2D& pose,
    double& fitness)
{
    // このレベルの地図をOccupancyGridに変換
    nav_msgs::msg::OccupancyGrid level_map = reference_map.getLevelAsOccupancyGrid(level);
    
    if (level_map.data.empty()) {
        return false;
    }
    
    // ScanToMapMatcherを使用してマッチング
    ScanToMapMatcher matcher;
    auto result = matcher.evaluatePose(scan, level_map, pose);
    
    pose = result.best_pose;
    fitness = result.fitness;
    
    RCLCPP_DEBUG(rclcpp::get_logger("multi_resolution_matcher"),
                "Level %d: fitness = %.3f, hits = %d/%d",
                level, fitness, result.hit_count, result.total_count);
    
    return true;
}

}  // namespace es_slam