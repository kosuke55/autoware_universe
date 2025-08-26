# Goal Planner 安全性チェック改善の実装計画

## 背景と課題

### 現状の問題
- `isSafePath`メソッドは現在速度(`current_velocity`)から`ego_predicted_path`を生成している
- 危険と判定されると停止線が挿入され、車両速度が0に近づく
- 速度低下により`ego_predicted_path`が短くなり、再び「安全」と誤判定される可能性がある
- これにより不要な再発進が発生する

### 解決方針
- `parking_path`に埋め込まれた計画速度を使用して安全性チェックを行う
- 計画速度ベースと現在速度ベースの両方で安全性を確認
- 両方が安全な場合のみ、安全と判定する

## 調査結果

### parking_pathの速度情報
- `PathPointWithLaneId`構造体には`longitudinal_velocity_mps`フィールドが存在
- 駐車動作区間：`pull_over_velocity`（デフォルト3.0 m/s）以下に制限
- ゴール地点：速度0.0 m/s
- 計画速度が明確に埋め込まれており、現在速度とは独立

## 実装計画

### 1. 新関数の追加

#### ファイル構成
- **ヘッダ**: `autoware_behavior_path_goal_planner_module/include/autoware/behavior_path_goal_planner_module/util.hpp`
- **実装**: `autoware_behavior_path_goal_planner_module/src/util.cpp`

#### 関数仕様

```cpp
namespace autoware::behavior_path_planner::goal_planner_utils
{

std::vector<utils::path_safety_checker::PoseWithVelocityStamped> 
createPredictedPath(
  const PathWithLaneId & parking_path,
  const PathWithLaneId & current_path,
  const geometry_msgs::msg::Pose & current_pose,
  const double time_horizon,
  const double time_resolution);

}
```

##### 引数
- `parking_path`: 計画速度が埋め込まれた駐車パス
- `current_path`: 現在の参照パス（`current_pull_over_path`）
- `current_pose`: 現在の車両位置
- `time_horizon`: 予測時間の長さ[秒]
- `time_resolution`: 時間刻み幅[秒]

##### 戻り値
- 時間ベースで生成された予測パス

##### 開始位置の決定ロジック
- `current_path`上で`current_pose`のインデックスを取得
- `current_path`上で`parking_path`の最初の点のインデックスを取得
- インデックスを比較：
  - `current_pose`のインデックス < `parking_path`開始点のインデックスの場合：
    - `parking_path`の最初から予測パスを生成
  - それ以外の場合：
    - `current_pose`に対応する`parking_path`上の位置から予測パスを生成

#### 実装詳細

```cpp
std::vector<utils::path_safety_checker::PoseWithVelocityStamped> createPredictedPath(
  const PathWithLaneId & parking_path,
  const PathWithLaneId & current_path,
  const geometry_msgs::msg::Pose & current_pose,
  const double time_horizon,
  const double time_resolution)
{
  using autoware::behavior_path_planner::utils::path_safety_checker::PoseWithVelocityStamped;
  using autoware::motion_utils::calcInterpolatedPose;
  using autoware::motion_utils::calcSignedArcLength;
  using autoware::motion_utils::findNearestIndex;
  
  if (parking_path.points.empty()) {
    return {};
  }
  
  // Find indices on paths
  const auto ego_idx = findNearestIndex(current_path.points, current_pose.position);
  const auto & parking_start_pose = parking_path.points.front().point.pose;
  const auto parking_start_idx = findNearestIndex(current_path.points, parking_start_pose.position);
  
  const auto total_length = calcSignedArcLength(
    parking_path.points, 0, parking_path.points.size() - 1);
  
  const auto & points = parking_path.points;
  
  // Build cumulative lengths once for efficient reuse
  std::vector<double> cumulative_lengths;
  cumulative_lengths.reserve(points.size());
  cumulative_lengths.push_back(0.0);
  
  for (size_t i = 1; i < points.size(); ++i) {
    const auto length = autoware_utils::calc_distance2d(
      points[i - 1].point.pose.position, points[i].point.pose.position);
    cumulative_lengths.push_back(cumulative_lengths.back() + length);
  }
  
  // Velocity interpolation with efficient segment search
  const auto interpolate_velocity = [&](const double target_length) -> double {
    if (points.size() < 2) {
      return points.front().point.longitudinal_velocity_mps;
    }
    
    // Binary search for the target segment
    const auto upper = std::upper_bound(
      cumulative_lengths.begin(), cumulative_lengths.end(), target_length);
    
    if (upper == cumulative_lengths.end()) {
      return points.back().point.longitudinal_velocity_mps;
    }
    
    const auto idx = std::distance(cumulative_lengths.begin(), upper);
    if (idx == 0) {
      return points.front().point.longitudinal_velocity_mps;
    }
    
    // Linear interpolation between velocities
    const auto segment_start_length = cumulative_lengths[idx - 1];
    const auto segment_end_length = cumulative_lengths[idx];
    const auto segment_length = segment_end_length - segment_start_length;
    
    if (segment_length < 1e-6) {  // Avoid division by zero
      return points[idx - 1].point.longitudinal_velocity_mps;
    }
    
    const auto ratio = std::clamp(
      (target_length - segment_start_length) / segment_length, 0.0, 1.0);
    
    const auto & v0 = points[idx - 1].point.longitudinal_velocity_mps;
    const auto & v1 = points[idx].point.longitudinal_velocity_mps;
    return v0 + ratio * (v1 - v0);
  };
  
  // Determine start length
  const auto start_length = [&]() -> double {
    if (ego_idx < parking_start_idx) {
      return 0.0;  // Start from beginning of parking_path
    }
    // Start from current pose position on parking_path
    const auto idx = findNearestIndex(parking_path.points, current_pose.position);
    return calcSignedArcLength(parking_path.points, 0, idx);
  }();
  
  // Generate predicted path using range-based approach
  std::vector<PoseWithVelocityStamped> predicted_path;
  predicted_path.reserve(static_cast<size_t>(time_horizon / time_resolution) + 1);
  
  for (double t = 0.0, length = start_length; t < time_horizon; t += time_resolution) {
    if (length >= total_length) {
      break;  // Reached end of path
    }
    
    const auto pose = calcInterpolatedPose(parking_path.points, length);
    const auto velocity = interpolate_velocity(length);
    predicted_path.emplace_back(t, pose, velocity);
    
    // Update length for next time step
    length += velocity * time_resolution;
  }
  
  return predicted_path;
}
```

### 2. isSafePathメソッドの修正

#### 修正箇所
`goal_planner_module.cpp`の`isSafePath`メソッド（約2421-2551行）

#### 修正内容

```cpp
// パスの取得
const auto & current_pull_over_path = pull_over_path.getCurrentPath();
const auto parking_path = pull_over_path.parking_path();

// 計画速度ベースの予測パス生成
const auto ego_predicted_path = goal_planner_utils::createPredictedPath(
  parking_path,
  current_pull_over_path,
  current_pose,
  is_object_front ? ego_predicted_path_params_->time_horizon_for_front_object
                   : ego_predicted_path_params_->time_horizon_for_rear_object,
  ego_predicted_path_params_->time_resolution);

const bool current_is_safe = std::invoke([&]() {
  // 計画速度ベースでのチェック
  if (parameters_.safety_check_params.method == "RSS") {
    return autoware::behavior_path_planner::utils::path_safety_checker::checkSafetyWithRSS(
      parking_path, ego_predicted_path, filtered_objects, collision_check,
      planner_data->parameters, safety_check_params_.rss_params,
      objects_filtering_params_.use_all_predicted_path, hysteresis_factor,
      safety_check_params_.collision_check_yaw_diff_threshold);
  }
  if (parameters_.safety_check_params.method == "integral_predicted_polygon") {
    return utils::path_safety_checker::checkSafetyWithIntegralPredictedPolygon(
      ego_predicted_path, vehicle_info_, filtered_objects,
      objects_filtering_params_.check_all_predicted_path,
      parameters_.safety_check_params.integral_predicted_polygon_params, collision_check);
  }
  // エラー処理
});
```

## 変更ファイル一覧

1. `autoware_behavior_path_goal_planner_module/include/autoware/behavior_path_goal_planner_module/util.hpp`
   - 新関数の宣言を追加

2. `autoware_behavior_path_goal_planner_module/src/util.cpp`
   - 新関数の実装を追加

3. `autoware_behavior_path_goal_planner_module/src/goal_planner_module.cpp`
   - `isSafePath`メソッドを修正
   - 両方の予測パスで安全性チェックを実施
