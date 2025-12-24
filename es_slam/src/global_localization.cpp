#include "es_slam/global_localization.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace es_slam
{

// ========== ScanToMapMatcher Implementation ==========

ScanToMapMatcher::MatchingResult ScanToMapMatcher::evaluatePose(
    const sensor_msgs::msg::LaserScan& scan,
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Pose2D& candidate_pose)
{
    MatchingResult result;
    result.best_pose = candidate_pose;
    result.fitness = calculateFitness(scan, map, candidate_pose);
    
    // Count hits and total valid points
    result.hit_count = 0;
    result.total_count = 0;
    
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        double range = scan.ranges[i];
        
        // 有効範囲チェック
        if (range < scan.range_min || range > scan.range_max) continue;
        if (range < 0.1 || range > 40.0) continue;  // 100mm-40000mm
        
        double angle = scan.angle_min + i * scan.angle_increment;
        auto [world_x, world_y] = transformScanPoint(range, angle, candidate_pose);
        auto [grid_x, grid_y] = worldToGrid(world_x, world_y, map);
        
        result.total_count++;
        
        if (isOccupied(grid_x, grid_y, map)) {
            result.hit_count++;
        }
    }
    
    return result;
}

double ScanToMapMatcher::calculateFitness(
    const sensor_msgs::msg::LaserScan& scan,
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Pose2D& pose)
{
    int hits = 0, total = 0;
    
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        double range = scan.ranges[i];
        
        // 有効範囲チェック
        if (range < scan.range_min || range > scan.range_max) continue;
        if (range < 0.1 || range > 40.0) continue;  // 100mm-40000mm
        
        // レーザー角度計算
        double angle = scan.angle_min + i * scan.angle_increment;
        
        // 地図座標に変換
        auto [world_x, world_y] = transformScanPoint(range, angle, pose);
        auto [grid_x, grid_y] = worldToGrid(world_x, world_y, map);
        
        total++;
        
        // 占有セル判定
        if (isOccupied(grid_x, grid_y, map)) {
            hits++;
        }
    }
    
    return total > 0 ? static_cast<double>(hits) / total : 0.0;
}

std::pair<double, double> ScanToMapMatcher::transformScanPoint(
    double range, double angle, const geometry_msgs::msg::Pose2D& robot_pose)
{
    // ロボット座標系でのレーザー点
    double local_x = range * std::cos(angle);
    double local_y = range * std::sin(angle);
    
    // ワールド座標系に変換
    double cos_theta = std::cos(robot_pose.theta);
    double sin_theta = std::sin(robot_pose.theta);
    
    double world_x = robot_pose.x + local_x * cos_theta - local_y * sin_theta;
    double world_y = robot_pose.y + local_x * sin_theta + local_y * cos_theta;
    
    return {world_x, world_y};
}

std::pair<int, int> ScanToMapMatcher::worldToGrid(
    double world_x, double world_y, const nav_msgs::msg::OccupancyGrid& map)
{
    // 地図原点からの相対位置
    double relative_x = world_x - map.info.origin.position.x;
    double relative_y = world_y - map.info.origin.position.y;
    
    // グリッド座標に変換
    int grid_x = static_cast<int>(std::floor(relative_x / map.info.resolution));
    int grid_y = static_cast<int>(std::floor(relative_y / map.info.resolution));
    
    return {grid_x, grid_y};
}

bool ScanToMapMatcher::isOccupied(
    int grid_x, int grid_y, const nav_msgs::msg::OccupancyGrid& map)
{
    // 範囲チェック
    if (grid_x < 0 || grid_x >= static_cast<int>(map.info.width) ||
        grid_y < 0 || grid_y >= static_cast<int>(map.info.height)) {
        return false;
    }
    
    // インデックス計算
    int index = grid_y * map.info.width + grid_x;
    
    // 占有判定（50より大きければ占有）
    return map.data[index] > 50;
}

// ========== MapToMapMatcher Implementation ==========

MapToMapMatcher::MapMatchingResult MapToMapMatcher::matchMaps(
    const nav_msgs::msg::OccupancyGrid& reference_map,
    const nav_msgs::msg::OccupancyGrid& slam_map,
    const geometry_msgs::msg::Pose2D& candidate_transform,
    double scale)
{
    MapMatchingResult result;
    result.transformation = candidate_transform;
    result.scale_factor = scale;
    result.fitness = calculateMapFitness(reference_map, slam_map, candidate_transform, scale);
    
    // Extract occupied cells and count matches
    auto slam_occupied = extractOccupiedCells(slam_map);
    auto transformed_cells = transformOccupiedCells(slam_occupied, candidate_transform, scale);
    
    result.total_cells = transformed_cells.size();
    result.matched_cells = 0;
    
    for (const auto& [cell_x, cell_y] : transformed_cells) {
        // ワールド座標をグリッド座標に変換
        double relative_x = cell_x - reference_map.info.origin.position.x;
        double relative_y = cell_y - reference_map.info.origin.position.y;
        
        int grid_x = static_cast<int>(std::floor(relative_x / reference_map.info.resolution));
        int grid_y = static_cast<int>(std::floor(relative_y / reference_map.info.resolution));
        
        if (grid_x >= 0 && grid_x < static_cast<int>(reference_map.info.width) &&
            grid_y >= 0 && grid_y < static_cast<int>(reference_map.info.height)) {
            
            int index = grid_y * reference_map.info.width + grid_x;
            if (reference_map.data[index] > 50) {
                result.matched_cells++;
            }
        }
    }
    
    return result;
}

double MapToMapMatcher::calculateMapFitness(
    const nav_msgs::msg::OccupancyGrid& ref_map,
    const nav_msgs::msg::OccupancyGrid& slam_map,
    const geometry_msgs::msg::Pose2D& transform,
    double scale)
{
    auto slam_occupied = extractOccupiedCells(slam_map);
    auto transformed_cells = transformOccupiedCells(slam_occupied, transform, scale);
    
    int matches = 0;
    int total = transformed_cells.size();
    
    for (const auto& [cell_x, cell_y] : transformed_cells) {
        // ワールド座標をグリッド座標に変換
        double relative_x = cell_x - ref_map.info.origin.position.x;
        double relative_y = cell_y - ref_map.info.origin.position.y;
        
        int grid_x = static_cast<int>(std::floor(relative_x / ref_map.info.resolution));
        int grid_y = static_cast<int>(std::floor(relative_y / ref_map.info.resolution));
        
        // 有効範囲内かチェック
        if (grid_x >= 0 && grid_x < static_cast<int>(ref_map.info.width) &&
            grid_y >= 0 && grid_y < static_cast<int>(ref_map.info.height)) {
            
            // 参照地図の対応位置が占有されているかチェック
            int index = grid_y * ref_map.info.width + grid_x;
            if (ref_map.data[index] > 50) {  // 占有閾値
                matches++;
            }
        }
    }
    
    return total > 0 ? static_cast<double>(matches) / total : 0.0;
}

std::vector<std::pair<double, double>> MapToMapMatcher::transformOccupiedCells(
    const std::vector<std::pair<double, double>>& cells,
    const geometry_msgs::msg::Pose2D& transform,
    double scale_factor)
{
    std::vector<std::pair<double, double>> transformed;
    transformed.reserve(cells.size());
    
    double cos_theta = std::cos(transform.theta);
    double sin_theta = std::sin(transform.theta);
    
    for (const auto& [x, y] : cells) {
        // スケール適用
        double scaled_x = x * scale_factor;
        double scaled_y = y * scale_factor;
        
        // 回転と平行移動
        double new_x = transform.x + scaled_x * cos_theta - scaled_y * sin_theta;
        double new_y = transform.y + scaled_x * sin_theta + scaled_y * cos_theta;
        
        transformed.push_back({new_x, new_y});
    }
    
    return transformed;
}

std::vector<std::pair<double, double>> MapToMapMatcher::extractOccupiedCells(
    const nav_msgs::msg::OccupancyGrid& map)
{
    std::vector<std::pair<double, double>> occupied_cells;
    
    for (unsigned int y = 0; y < map.info.height; ++y) {
        for (unsigned int x = 0; x < map.info.width; ++x) {
            int index = y * map.info.width + x;
            
            // 占有セルを抽出（50より大きい値）
            if (map.data[index] > 50) {
                // グリッド座標からワールド座標に変換
                double world_x = map.info.origin.position.x + x * map.info.resolution + map.info.resolution / 2.0;
                double world_y = map.info.origin.position.y + y * map.info.resolution + map.info.resolution / 2.0;
                
                occupied_cells.push_back({world_x, world_y});
            }
        }
    }
    
    return occupied_cells;
}

// ========== GlobalLocalizationEngine Implementation ==========

GlobalLocalizationEngine::GlobalLocalizationEngine(const rclcpp::Logger& logger)
    : logger_(logger),
      rng_(std::random_device{}()),
      uniform_dist_(0.0, 1.0)
{
}

GlobalLocalizationEngine::LocalizationResult GlobalLocalizationEngine::performGlobalLocalization(
    MatchingMode mode,
    const sensor_msgs::msg::LaserScan& scan,
    const nav_msgs::msg::OccupancyGrid& reference_map,
    const nav_msgs::msg::OccupancyGrid& slam_map,
    const geometry_msgs::msg::Pose2D& initial_guess)
{
    // 初期化
    es_gene_.iteration_count = 0;
    es_gene_.initial_pose.x = initial_guess.x;
    es_gene_.initial_pose.y = initial_guess.y;
    es_gene_.initial_pose.theta = initial_guess.theta;
    es_gene_.initial_pose.scale = 1.0;
    
    // 集団初期化
    initializePopulation(initial_guess);
    
    // 進化戦略実行
    return runEvolutionaryStrategy(mode, scan, reference_map, slam_map);
}

GlobalLocalizationEngine::LocalizationResult GlobalLocalizationEngine::runEvolutionaryStrategy(
    MatchingMode mode,
    const sensor_msgs::msg::LaserScan& scan,
    const nav_msgs::msg::OccupancyGrid& reference_map,
    const nav_msgs::msg::OccupancyGrid& slam_map)
{
    GlobalLocalizationEngine::LocalizationResult result;
    
    RCLCPP_INFO(logger_, "Starting evolutionary strategy global localization");
    
    while (es_gene_.iteration_count < params_.max_iterations) {
        // 適応度評価
        evaluatePopulation(mode, scan, reference_map, slam_map);
        
        // 最良個体更新
        updateBestIndividual();
        
        // 収束判定
        if (hasConverged()) {
            RCLCPP_INFO(logger_, "Converged at iteration %d with fitness %.3f",
                       es_gene_.iteration_count, es_gene_.population[es_gene_.best_index].fitness);
            break;
        }
        
        // 遺伝的操作
        applyGeneticOperators();
        
        es_gene_.iteration_count++;
        
        // 進捗ログ（10反復ごと）
        if (es_gene_.iteration_count % 10 == 0) {
            RCLCPP_DEBUG(logger_, "Iteration %d: Best fitness = %.3f",
                        es_gene_.iteration_count, es_gene_.population[es_gene_.best_index].fitness);
        }
    }
    
    // 結果設定
    const auto& best = es_gene_.population[es_gene_.best_index];
    result.success = best.fitness > params_.fitness_threshold;
    result.confidence = best.fitness;
    
    // PoseWithCovarianceStamped設定
    result.estimated_pose.header.stamp = rclcpp::Clock().now();
    result.estimated_pose.header.frame_id = "map";
    result.estimated_pose.pose.pose.position.x = best.x;
    result.estimated_pose.pose.pose.position.y = best.y;
    result.estimated_pose.pose.pose.position.z = 0.0;
    
    // Quaternion from yaw
    result.estimated_pose.pose.pose.orientation.x = 0.0;
    result.estimated_pose.pose.pose.orientation.y = 0.0;
    result.estimated_pose.pose.pose.orientation.z = std::sin(best.theta / 2.0);
    result.estimated_pose.pose.pose.orientation.w = std::cos(best.theta / 2.0);
    
    // Covariance (位置推定の不確実性)
    result.estimated_pose.pose.covariance[0] = (1.0 - best.fitness) * 0.5;  // x variance
    result.estimated_pose.pose.covariance[7] = (1.0 - best.fitness) * 0.5;  // y variance
    result.estimated_pose.pose.covariance[35] = (1.0 - best.fitness) * 0.1; // theta variance
    
    if (result.success) {
        result.status_message = "Global localization succeeded with confidence " + 
                               std::to_string(result.confidence);
    } else {
        result.status_message = "Global localization failed - fitness below threshold";
    }
    
    return result;
}

void GlobalLocalizationEngine::initializePopulation(const geometry_msgs::msg::Pose2D& initial_guess)
{
    std::normal_distribution<> pos_dist_x(0.0, params_.search_range_xy / 3.0);
    std::normal_distribution<> pos_dist_y(0.0, params_.search_range_xy / 3.0);
    std::normal_distribution<> angle_dist(0.0, params_.search_range_angle * M_PI / 180.0 / 3.0);
    std::normal_distribution<> scale_dist(1.0, 0.1);
    
    for (size_t i = 0; i < ESGene::POPULATION_SIZE; ++i) {
        auto& candidate = es_gene_.population[i];
        
        // 初期推定値周辺にランダムに生成
        candidate.x = initial_guess.x + pos_dist_x(rng_);
        candidate.y = initial_guess.y + pos_dist_y(rng_);
        candidate.theta = initial_guess.theta + angle_dist(rng_);
        
        // 角度を[-π, π]に正規化
        while (candidate.theta > M_PI) candidate.theta -= 2.0 * M_PI;
        while (candidate.theta < -M_PI) candidate.theta += 2.0 * M_PI;
        
        // スケール調整が有効な場合
        if (params_.enable_scale_adjustment) {
            candidate.scale = scale_dist(rng_);
            candidate.scale = std::max(0.5, std::min(2.0, candidate.scale));
        } else {
            candidate.scale = 1.0;
        }
        
        candidate.fitness = 0.0;
        candidate.weight = 0.0;
    }
    
    // 中心に一つは初期推定値そのものを配置
    es_gene_.population[0].x = initial_guess.x;
    es_gene_.population[0].y = initial_guess.y;
    es_gene_.population[0].theta = initial_guess.theta;
    es_gene_.population[0].scale = 1.0;
}

void GlobalLocalizationEngine::evaluatePopulation(
    MatchingMode mode,
    const sensor_msgs::msg::LaserScan& scan,
    const nav_msgs::msg::OccupancyGrid& reference_map,
    const nav_msgs::msg::OccupancyGrid& slam_map)
{
    double total_fitness = 0.0;
    
    for (size_t i = 0; i < ESGene::POPULATION_SIZE; ++i) {
        auto& candidate = es_gene_.population[i];
        
        geometry_msgs::msg::Pose2D pose;
        pose.x = candidate.x;
        pose.y = candidate.y;
        pose.theta = candidate.theta;
        
        if (mode == MatchingMode::SCAN_TO_MAP) {
            candidate.fitness = scan_matcher_.calculateFitness(scan, reference_map, pose);
        } else {  // MAP_TO_MAP
            candidate.fitness = map_matcher_.calculateMapFitness(reference_map, slam_map, pose, candidate.scale);
        }
        
        total_fitness += candidate.fitness;
    }
    
    // 重み計算
    double avg_fitness = total_fitness / ESGene::POPULATION_SIZE;
    es_gene_.average_weight = avg_fitness;
    
    for (auto& candidate : es_gene_.population) {
        candidate.weight = candidate.fitness / (avg_fitness + 1e-6);
    }
}

void GlobalLocalizationEngine::applyGeneticOperators()
{
    std::vector<ESGene::Candidate> new_population;
    new_population.reserve(ESGene::POPULATION_SIZE);
    
    // エリート保存（上位10%）
    std::vector<size_t> indices(ESGene::POPULATION_SIZE);
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [this](size_t a, size_t b) {
        return es_gene_.population[a].fitness > es_gene_.population[b].fitness;
    });
    
    size_t elite_count = ESGene::POPULATION_SIZE / 10;
    for (size_t i = 0; i < elite_count; ++i) {
        new_population.push_back(es_gene_.population[indices[i]]);
    }
    
    // 残りは交叉と突然変異で生成
    while (new_population.size() < ESGene::POPULATION_SIZE) {
        if (uniform_dist_(rng_) < params_.crossover_rate) {
            // 交叉
            int parent1_idx = selectParent();
            int parent2_idx = selectParent();
            auto child = crossover(es_gene_.population[parent1_idx], es_gene_.population[parent2_idx]);
            
            // 突然変異
            if (uniform_dist_(rng_) < params_.mutation_rate) {
                mutate(child);
            }
            
            new_population.push_back(child);
        } else {
            // 選択と突然変異
            int parent_idx = selectParent();
            auto child = es_gene_.population[parent_idx];
            mutate(child);
            new_population.push_back(child);
        }
    }
    
    // 新しい集団に置き換え
    es_gene_.population = std::array<ESGene::Candidate, ESGene::POPULATION_SIZE>();
    for (size_t i = 0; i < ESGene::POPULATION_SIZE; ++i) {
        es_gene_.population[i] = new_population[i];
    }
}

void GlobalLocalizationEngine::mutate(ESGene::Candidate& candidate)
{
    std::normal_distribution<> pos_mutation(0.0, params_.search_range_xy * 0.1);
    std::normal_distribution<> angle_mutation(0.0, params_.search_range_angle * M_PI / 180.0 * 0.1);
    std::normal_distribution<> scale_mutation(0.0, 0.05);
    
    candidate.x += pos_mutation(rng_);
    candidate.y += pos_mutation(rng_);
    candidate.theta += angle_mutation(rng_);
    
    // 角度正規化
    while (candidate.theta > M_PI) candidate.theta -= 2.0 * M_PI;
    while (candidate.theta < -M_PI) candidate.theta += 2.0 * M_PI;
    
    if (params_.enable_scale_adjustment) {
        candidate.scale += scale_mutation(rng_);
        candidate.scale = std::max(0.5, std::min(2.0, candidate.scale));
    }
}

ESGene::Candidate GlobalLocalizationEngine::crossover(
    const ESGene::Candidate& parent1, const ESGene::Candidate& parent2)
{
    ESGene::Candidate child;
    
    // 重み付き平均で交叉
    double total_weight = parent1.weight + parent2.weight;
    double w1 = parent1.weight / total_weight;
    double w2 = parent2.weight / total_weight;
    
    child.x = w1 * parent1.x + w2 * parent2.x;
    child.y = w1 * parent1.y + w2 * parent2.y;
    
    // 角度の交叉（円周上の補間）
    double sin_theta = w1 * std::sin(parent1.theta) + w2 * std::sin(parent2.theta);
    double cos_theta = w1 * std::cos(parent1.theta) + w2 * std::cos(parent2.theta);
    child.theta = std::atan2(sin_theta, cos_theta);
    
    child.scale = w1 * parent1.scale + w2 * parent2.scale;
    
    return child;
}

int GlobalLocalizationEngine::selectParent()
{
    // ルーレット選択
    double total_fitness = 0.0;
    for (const auto& candidate : es_gene_.population) {
        total_fitness += candidate.fitness;
    }
    
    double threshold = uniform_dist_(rng_) * total_fitness;
    double cumulative = 0.0;
    
    for (size_t i = 0; i < ESGene::POPULATION_SIZE; ++i) {
        cumulative += es_gene_.population[i].fitness;
        if (cumulative >= threshold) {
            return i;
        }
    }
    
    return ESGene::POPULATION_SIZE - 1;
}

void GlobalLocalizationEngine::updateBestIndividual()
{
    double max_fitness = -1.0;
    
    for (size_t i = 0; i < ESGene::POPULATION_SIZE; ++i) {
        if (es_gene_.population[i].fitness > max_fitness) {
            max_fitness = es_gene_.population[i].fitness;
            es_gene_.best_index = i;
        }
    }
}

bool GlobalLocalizationEngine::hasConverged() const
{
    // 収束条件：最良適応度が閾値を超え、かつ最小反復回数を満たす
    const auto& best = es_gene_.population[es_gene_.best_index];
    return best.fitness > params_.fitness_threshold && 
           es_gene_.iteration_count > 35;
}

geometry_msgs::msg::PoseArray GlobalLocalizationEngine::getCurrentPopulation() const
{
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = rclcpp::Clock().now();
    pose_array.header.frame_id = "map";
    
    for (const auto& candidate : es_gene_.population) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = candidate.x;
        pose.position.y = candidate.y;
        pose.position.z = 0.0;
        
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = std::sin(candidate.theta / 2.0);
        pose.orientation.w = std::cos(candidate.theta / 2.0);
        
        pose_array.poses.push_back(pose);
    }
    
    return pose_array;
}

}  // namespace es_slam