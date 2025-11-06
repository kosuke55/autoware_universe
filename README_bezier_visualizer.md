# Autoware Bezier Curve Visualizer

インタラクティブなベジエ曲線可視化ツール。Autowareの`autoware_bezier_sampler`と`behavior_path_goal_planner_module`で使用されている経路計画アルゴリズムを再現します。

## 概要

このツールは、Autowareの駐車経路計画（Pull-Over）で使用される**状態制約付き5次ベジエ曲線**を可視化します。車両の位置・向き・曲率を制約として考慮した経路生成を視覚的に理解できます。

## 特徴

### 🎯 Autoware実装の完全再現
- **5次ベジエ曲線**: 6つの制御点による滑らかな経路
- **状態制約**: 位置・ヨー角・曲率を同時に満たす
- **制御点自動計算**: 速度・加速度パラメータから制御点を生成

### 📊 リアルタイム可視化
- **曲率プロファイル**: 経路全体の曲率変化をグラフ表示
- **状態表示**: 初期・終端の位置と向きを矢印で可視化
- **制御点表示**: ベジエ曲線の6つの制御点を表示

### 🎛️ インタラクティブ制御
- **スライダー操作**: パラメータをリアルタイムで変更
- **動的更新**: 変更が即座に曲線に反映

## 数学的背景

### 5次ベジエ曲線の定式化

5次ベジエ曲線は6つの制御点 **P₀, P₁, P₂, P₃, P₄, P₅** で定義されます：

```
B(t) = Σ(i=0 to 5) [C(5,i) × (1-t)^(5-i) × t^i × Pᵢ]
```

ここで `C(5,i)` は二項係数、`t ∈ [0, 1]` はパラメータです。

### 制御点の計算

制御点は初期・終端の**状態**（位置、速度、加速度）から計算されます：

#### 基本制御点 (P₀, P₅)
```
P₀ = 初期位置
P₅ = 終端位置
```

#### 速度依存制御点 (P₁, P₄)
```
P₁ = P₀ + (1/5) × 初期速度ベクトル
P₄ = P₅ - (1/5) × 終端速度ベクトル
```

#### 加速度依存制御点 (P₂, P₃)
```
P₂ = 2×P₁ - P₀ + (1/20) × 初期加速度ベクトル
P₃ = 2×P₄ - P₅ - (1/20) × 終端加速度ベクトル
```

### 速度・加速度ベクトル

#### 接線速度（Tangent Velocity）
```
v_init = v_init_coeff × distance × [cos(θ_init), sin(θ_init)]
v_final = v_final_coeff × distance × [cos(θ_final), sin(θ_final)]
```

#### 加速度（Acceleration）
加速度には**接線成分**と**法線成分**（曲率による）があります：

```
a_init = a_tangent + a_normal
       = acc_coeff × distance × tangent_unit
         + curvature_init × |v_init|² × normal_unit
```

ここで：
- `distance`: 初期位置と終端位置の距離
- `tangent_unit`: 接線方向の単位ベクトル
- `normal_unit`: 法線方向の単位ベクトル（接線に垂直）
- `curvature`: 曲率 [1/m]

## 実装の対応関係

### ソースコードとの対応

| このツール | Autowareソース | 説明 |
|-----------|---------------|------|
| `AutowareBezierCurve` | `autoware_bezier_sampler/bezier.hpp` | 5次ベジエ曲線クラス |
| `generate_bezier_from_states()` | `bezier_sampling.cpp:88-106` | 制御点生成ロジック |
| `v_init_coeff`, `v_final_coeff` | `bezier_pull_over.cpp:193-202` | パラメータサンプリング |

### パラメータの意味

#### v_init_coeff (初期速度係数)
- **範囲**: 0.1 〜 2.0
- **意味**: 経路の**立ち上がり**の速さを制御
- **効果**:
  - 小さい値 (0.1): ゆっくり曲がり始める（急な初期曲率）
  - 大きい値 (2.0): すぐに曲がり始める（緩やかな初期曲率）
- **用途**: 駐車開始時の操舵特性

#### v_final_coeff (終端速度係数)
- **範囲**: 0.1 〜 2.0
- **意味**: 経路の**収束**の速さを制御
- **効果**:
  - 小さい値 (0.1): ゆっくり目標姿勢に近づく
  - 大きい値 (2.0): 素早く目標姿勢に到達
- **用途**: 駐車完了時の姿勢精度
- **注意**: `bezier_pull_over.cpp:197`では最大0.25にスケーリング

#### acc_coeff (加速度係数)
- **範囲**: 0.0 〜 10.0
- **意味**: 経路の**滑らかさ**を制御
- **効果**:
  - 小さい値 (0.0): 最小限の加速度変化
  - 大きい値 (10.0): より大きな加速度変化を許容
- **用途**: 乗り心地と経路の自由度のトレードオフ

## 使い方

### 1. 必要なパッケージのインストール

```bash
pip install numpy matplotlib
```

### 2. プログラムの実行

```bash
# Autoware実装再現版（推奨）
python autoware_bezier_visualizer.py

# シンプル版（基本的なベジエ曲線）
python bezier_curve_visualizer.py
```

### 3. インタラクティブ操作

- **スライダー操作**: 画面下部のスライダーでパラメータを調整
  - `v_init`: 初期速度係数（0.1 〜 2.0）
  - `v_final`: 終端速度係数（0.1 〜 2.0）
  - `acc`: 加速度係数（0.0 〜 10.0）

- **表示要素**:
  - 青い実線: ベジエ曲線（経路）
  - 赤い破線: 制御点と制御多角形
  - 緑の矢印: 初期状態（位置と向き）
  - 赤の矢印: 終端状態（位置と向き）
  - 下部グラフ: 曲率プロファイル

## 実用例

### 駐車経路計画での使用

Autowareの`BezierPullOver`プランナーでは、以下のように使用されます：

```cpp
// bezier_pull_over.cpp:193-202
for (unsigned i = 0; i <= n_sample_v_init; ++i) {
  for (unsigned j = 0; j <= n_sample_v_final; j++) {
    for (unsigned k = 0; k <= n_sample_acc; k++) {
      const double v_init_coeff = std::max(min_v_coeff, i * (1.0 / n_sample_v_init));
      const double v_final_coeff = std::max(min_v_coeff, j * 0.25 / (1.0 / n_sample_v_final));
      const double acc_coeff = k * (10.0 / n_sample_acc);
      // ベジエ曲線を生成してパスを評価
    }
  }
}
```

複数のパラメータ組み合わせで経路を生成し、安全性・実行可能性を評価します。

### パラメータ推奨値

#### 通常の駐車（並列駐車）
```python
v_init_coeff = 0.5   # 適度な立ち上がり
v_final_coeff = 0.1  # ゆっくり収束（精度重視）
acc_coeff = 0.0      # 滑らかな経路
```

#### 素早い駐車（縦列駐車）
```python
v_init_coeff = 1.0   # 早めの立ち上がり
v_final_coeff = 0.2  # やや早い収束
acc_coeff = 5.0      # 柔軟な経路
```

#### 曲率制約が厳しい場合
```python
v_init_coeff = 0.8   # 緩やかな初期曲率
v_final_coeff = 0.8  # 緩やかな終端曲率
acc_coeff = 2.0      # 適度な自由度
```

## ファイル構成

```
├── autoware_bezier_visualizer.py    # Autoware実装再現版（推奨）
├── bezier_curve_visualizer.py       # シンプル版
└── README_bezier_visualizer.md      # このファイル
```

### autoware_bezier_visualizer.py
- **完全再現**: Autowareの実装ロジックを忠実に再現
- **状態制約**: 位置・向き・曲率を考慮
- **行列計算**: 最適化された計算方式
- **用途**: Autowareの動作理解・パラメータチューニング

### bezier_curve_visualizer.py
- **シンプル**: 標準的なベジエ曲線の可視化
- **制御点操作**: マウスドラッグで制御点を移動可能
- **教育向け**: ベジエ曲線の基本理解

## 理論的背景

### なぜ5次ベジエ曲線か？

車両経路計画では以下の制約を満たす必要があります：

1. **位置制約** (C⁰連続性): 経路が目標位置を通過
2. **接線制約** (C¹連続性): 経路の向きが目標向きと一致
3. **曲率制約** (C²連続性): 経路の曲率が目標曲率と一致

これらを満たすには最低でも**5次の多項式**が必要です：
- 3次: 位置と接線のみ制約可能
- 4次: 曲率も制約可能だが自由度不足
- **5次**: 位置・接線・曲率を制約し、かつ最適化の自由度あり

### 曲率の重要性

車両には**最小回転半径**（最大曲率）があります：

```
curvature_max = 1 / R_min
```

例: 最小回転半径5mの車両
```
curvature_max = 1 / 5.0 = 0.2 [1/m]
```

経路の曲率がこれを超えると物理的に実行不可能になります。

## トラブルシューティング

### Q: 曲線が不自然な形になる
**A**: パラメータの組み合わせを調整してください：
- `v_init_coeff`と`v_final_coeff`の差が大きすぎる場合
- `acc_coeff`が大きすぎる場合

### Q: 初期・終端で向きが合っていない
**A**: これは正常です。ヨー角（heading）は状態として設定されていますが、スライダーでは変更できません。コード内の`initial_state['heading']`と`final_state['heading']`を直接編集してください。

### Q: 曲率プロファイルが不連続
**A**: これは数値計算の誤差です。`num_points`を増やすと滑らかになります：
```python
self.num_points = 200  # デフォルトは100
```

### Q: Matplotlibが表示されない
**A**: バックエンドの問題の可能性があります：
```bash
# Macの場合
export MPLBACKEND=MacOSX

# Linuxの場合
export MPLBACKEND=TkAgg
```

## 参考資料

### Autowareソースコード
- [`autoware_bezier_sampler/bezier.hpp`](planning/sampling_based_planner/autoware_bezier_sampler/include/autoware_bezier_sampler/bezier.hpp)
- [`autoware_bezier_sampler/bezier_sampling.cpp`](planning/sampling_based_planner/autoware_bezier_sampler/src/bezier_sampling.cpp)
- [`bezier_pull_over.cpp`](planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/src/pull_over_planner/bezier_pull_over.cpp)

### 学術論文
このアルゴリズムは以下の論文に基づいています：

> A. Artuñedo et al., "Real-Time Motion Planning Approach for Automated Driving in Urban Environments", Section IV

### 関連概念
- **Bezier Curves**: グラフィックス・CADで広く使用される曲線表現
- **Frenet Frame**: 曲線に沿った局所座標系（経路計画で使用）
- **Curvature Constraints**: 車両の運動学的制約
- **Quintic Polynomials**: 5次多項式による経路表現

## ライセンス

このツールはAutowareプロジェクトの一部として、Apache License 2.0の下で提供されます。

```
Copyright 2024 TIER IV, Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
```

## 貢献

改善提案やバグ報告は、Autoware Universeリポジトリの[Issues](https://github.com/autowarefoundation/autoware.universe/issues)にお願いします。

## まとめ

このツールを使用することで：
- ✅ Autowareの駐車経路計画アルゴリズムを視覚的に理解
- ✅ パラメータが経路形状に与える影響を直感的に把握
- ✅ 実際の車両での経路計画パラメータをチューニング

経路計画の理解を深め、より良い自動運転システムの開発にお役立てください！
