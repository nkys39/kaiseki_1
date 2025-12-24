#ifndef ES_SLAM_GLOBAL_LOCALIZATION_HPP
#define ES_SLAM_GLOBAL_LOCALIZATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <vector>
#include <array>
#include <memory>
#include <random>
#include <cmath>

namespace es_slam
{

// スキャンと地図のマッチング
class ScanToMapMatcher
{
public:
    struct MatchingResult
    {
        double fitness;        // 適応度 [0.0-1.0]
        int hit_count;        // マッチした点数
        int total_count;      // 全スキャン点数
        geometry_msgs::msg::Pose2D best_pose;  // 最適位置
    };
    
    ScanToMapMatcher() = default;
    
    MatchingResult evaluatePose(const sensor_msgs::msg::LaserScan& scan,
                               const nav_msgs::msg::OccupancyGrid& map,
                               const geometry_msgs::msg::Pose2D& candidate_pose);
    
    double calculateFitness(const sensor_msgs::msg::LaserScan& scan,
                           const nav_msgs::msg::OccupancyGrid& map,
                           const geometry_msgs::msg::Pose2D& pose);
                               
private:
    // レーザー点を地図座標に変換
    std::pair<double, double> transformScanPoint(double range, double angle,
                                                 const geometry_msgs::msg::Pose2D& robot_pose);
    
    // 地図座標から格子座標に変換                                               
    std::pair<int, int> worldToGrid(double world_x, double world_y,
                                   const nav_msgs::msg::OccupancyGrid& map);
                                   
    // 占有セル判定
    bool isOccupied(int grid_x, int grid_y, const nav_msgs::msg::OccupancyGrid& map);
};

// 地図と地図のマッチング
class MapToMapMatcher
{
public:
    struct MapMatchingResult
    {
        double fitness;           // 地図重なり適応度
        double scale_factor;      // スケール調整係数
        geometry_msgs::msg::Pose2D transformation;  // 変換行列
        int matched_cells;        // マッチした占有セル数
        int total_cells;          // 総占有セル数
    };
    
    MapToMapMatcher() = default;
    
    MapMatchingResult matchMaps(const nav_msgs::msg::OccupancyGrid& reference_map,
                               const nav_msgs::msg::OccupancyGrid& slam_map,
                               const geometry_msgs::msg::Pose2D& candidate_transform,
                               double scale = 1.0);
    
    double calculateMapFitness(const nav_msgs::msg::OccupancyGrid& ref_map,
                              const nav_msgs::msg::OccupancyGrid& slam_map,
                              const geometry_msgs::msg::Pose2D& transform,
                              double scale = 1.0);
                               
private:
    // SLAM地図の占有セルを参照地図座標系に変換
    std::vector<std::pair<double, double>> transformOccupiedCells(
        const std::vector<std::pair<double, double>>& cells,
        const geometry_msgs::msg::Pose2D& transform,
        double scale_factor);
        
    // 占有セル抽出
    std::vector<std::pair<double, double>> extractOccupiedCells(
        const nav_msgs::msg::OccupancyGrid& map);
};

// 進化戦略遺伝子構造
struct ESGene
{
    struct Candidate
    {
        double x, y, theta, scale;  // 位置・角度・スケール
        double fitness;             // 適応度
        double weight;              // 重み
    };
    
    static constexpr size_t POPULATION_SIZE = 1000;
    std::array<Candidate, POPULATION_SIZE> population;  // 集団
    int best_index = 0;                                // 最良個体インデックス
    double average_weight = 0.0;                       // 平均重み
    Candidate initial_pose;                            // 初期位置
    int iteration_count = 0;                           // 反復回数
};

// 大域的自己位置推定エンジン
class GlobalLocalizationEngine
{
public:
    enum class MatchingMode
    {
        SCAN_TO_MAP,    // スキャン→地図マッチング
        MAP_TO_MAP      // 地図→地図マッチング
    };
    
    struct LocalizationResult
    {
        bool success = false;
        double confidence = 0.0;
        geometry_msgs::msg::PoseWithCovarianceStamped estimated_pose;
        std::string status_message;
    };
    
    struct Parameters
    {
        bool enable_initial_estimation = true;     // 初回自動実行
        double search_range_xy = 2000.0;          // XY探索範囲[mm]
        double search_range_angle = 90.0;         // 角度探索範囲[deg]
        int population_size = 1000;               // ES集団サイズ
        int max_iterations = 100;                 // 最大反復回数
        double fitness_threshold = 0.4;           // 収束判定閾値
        bool enable_scale_adjustment = false;     // スケール調整有効化
        double mutation_rate = 0.1;               // 突然変異率
        double crossover_rate = 0.3;              // 交叉率
    };
    
    explicit GlobalLocalizationEngine(const rclcpp::Logger& logger);
    
    void setParameters(const Parameters& params) { params_ = params; }
    
    LocalizationResult performGlobalLocalization(
        MatchingMode mode,
        const sensor_msgs::msg::LaserScan& scan,
        const nav_msgs::msg::OccupancyGrid& reference_map,
        const nav_msgs::msg::OccupancyGrid& slam_map,
        const geometry_msgs::msg::Pose2D& initial_guess);
    
    // デバッグ用: 現在の集団を取得
    geometry_msgs::msg::PoseArray getCurrentPopulation() const;
    
    // デバッグ用: 進化進捗を取得
    double getEvolutionProgress() const { return static_cast<double>(es_gene_.iteration_count) / params_.max_iterations; }
    
private:
    rclcpp::Logger logger_;
    Parameters params_;
    ScanToMapMatcher scan_matcher_;
    MapToMapMatcher map_matcher_;
    ESGene es_gene_;
    std::mt19937 rng_;
    std::uniform_real_distribution<> uniform_dist_;
    
    // 進化戦略メイン処理
    LocalizationResult runEvolutionaryStrategy(
        MatchingMode mode,
        const sensor_msgs::msg::LaserScan& scan,
        const nav_msgs::msg::OccupancyGrid& reference_map,
        const nav_msgs::msg::OccupancyGrid& slam_map);
    
    // 集団初期化
    void initializePopulation(const geometry_msgs::msg::Pose2D& initial_guess);
    
    // 適応度評価
    void evaluatePopulation(
        MatchingMode mode,
        const sensor_msgs::msg::LaserScan& scan,
        const nav_msgs::msg::OccupancyGrid& reference_map,
        const nav_msgs::msg::OccupancyGrid& slam_map);
    
    // 遺伝的操作
    void applyGeneticOperators();
    
    // 突然変異
    void mutate(ESGene::Candidate& candidate);
    
    // 交叉
    ESGene::Candidate crossover(const ESGene::Candidate& parent1, const ESGene::Candidate& parent2);
    
    // 選択
    int selectParent();
    
    // 最良個体を更新
    void updateBestIndividual();
    
    // 収束判定
    bool hasConverged() const;
};

// 地図管理クラス
class GlobalLocalizationManager
{
public:
    GlobalLocalizationManager() = default;
    
    void setReferenceMap(const nav_msgs::msg::OccupancyGrid& map)
    {
        reference_map_ = map;
        reference_map_initialized_ = true;
    }
    
    void updateWorkingMap(const nav_msgs::msg::OccupancyGrid& map)
    {
        working_map_ = map;
    }
    
    void setCurrentSlamMap(const nav_msgs::msg::OccupancyGrid& map)
    {
        current_slam_map_ = map;
    }
    
    const nav_msgs::msg::OccupancyGrid& getReferenceMap() const { return reference_map_; }
    const nav_msgs::msg::OccupancyGrid& getWorkingMap() const { return working_map_; }
    const nav_msgs::msg::OccupancyGrid& getCurrentSlamMap() const { return current_slam_map_; }
    
    bool isReferenceMapInitialized() const { return reference_map_initialized_; }
    
private:
    nav_msgs::msg::OccupancyGrid reference_map_;      // 参照地図（読み取り専用）
    nav_msgs::msg::OccupancyGrid current_slam_map_;   // SLAM中の地図
    nav_msgs::msg::OccupancyGrid working_map_;        // 作業用地図（更新可能）
    bool reference_map_initialized_ = false;
};

}  // namespace es_slam

#endif  // ES_SLAM_GLOBAL_LOCALIZATION_HPP