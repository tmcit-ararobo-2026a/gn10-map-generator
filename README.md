# gn10-map-generator

gn10-localizationパッケージのマップ生成・可視化機能を提供するROS 2パッケージです。

## 目次

1. [概要](#1-概要)
2. [コントリビューション](#2-コントリビューション)
3. [ビルド・使い方](#3-ビルド使い方)
4. [システム構成](#4-システム構成)
5. [ライセンス](#5-ライセンス)

## 1. 概要

本リポジトリは、2026年NHK学生ロボコンAチーム向けのROS 2（Humble）制御システムおよびマップ生成・自己位置推定周辺ソフトウェアを集約したワークスペースです。

LiDARの点群データ（`sensor_msgs/msg/PointCloud2`）から自動で障害物やフィールドオブジェクト（円柱・直方体）を抽出し、JSONフォーマットのマップ（`field_map.json`）として書き出す機能や、そのマップをRViz 2上で3Dマーカー（`visualization_msgs/msg/MarkerArray`）として可視化するパッケージ等を提供します。

## 2. コントリビューション

[CONTRIBUTING.md](./CONTRIBUTING.md) を参照してください。

## 3. ビルド・使い方

### 動作環境
- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **主要依存ライブラリ**:
  - Point Cloud Library (PCL 1.12+) (`libpcl-dev`)
  - `pcl_conversions`
  - `nlohmann_json`
  - `tf2_ros` / `tf2_sensor_msgs`

### 依存関係のインストール

システムライブラリをインストールします。

```bash
sudo apt update
sudo apt install -y libpcl-dev ros-humble-pcl-conversions ros-humble-tf2-sensor-msgs nlohmann-json3-dev

```

### ビルド手順

ワークスペース直下で `colcon` を用いてビルドします。

```bash
cd ~/ws/ros2_ws

# ワークスペース全体のビルド
colcon build

# 特定パッケージのみビルドする場合 (例: gn10-map-generator)
colcon build --packages-select gn10-map-generator

# 環境変数の読み込み
source install/setup.bash

```

---

### ノードの実行方法

#### 1. 点群からマップ（JSON）の自動生成ノード (`pointcloud_to_map`)

LiDARの点群を `base_link` 座標系へ変換し、高さフィルタリングおよび Euclidean Cluster 抽出を行ってオブジェクトを検出します。**ノード起動中に点群を受信し続け、`Ctrl + C` でノードを終了したタイミングで JSON ファイルへ出力**します。

```bash
ros2 run gn10-map-generator pointcloud_to_map --ros-args \
  -r input_pointcloud:=/livox/lidar \
  -p target_frame:=base_link \
  -p output_json_path:=field_map.json \
  -p z_min_crop:=0.05 \
  -p z_max_crop:=2.00

```

* **主要パラメータ**:
* `target_frame` (default: `"base_link"`): マップの基準とする座標系
* `output_json_path` (default: `"field_map.json"`): 出力先パス
* `z_min_crop` / `z_max_crop`: 床面や天井を除外するためのZ軸閾値(m)
* `cluster_tolerance`: 同一物体とみなす点群間距離(m)



#### 2. マップ可視化ノード (`map_visualizer_node`)

出力された `field_map.json` を読み込み、RViz 2で描画可能な `MarkerArray` を配信（1Hz）します。

```bash
ros2 run gn10-map-generator map_visualizer_node --ros-args \
  -p json_path:=field_map.json \
  -p map_frame:=base_link

```

* **RViz 2 での確認方法**:
1. `ros2 run rviz2 rviz2` を起動
2. Fixed Frame を `base_link`（または `map`）に設定
3. **Add** -> **By topic** -> `/field_map_markers` -> **MarkerArray** を追加



## 4. システム構成

### ROS2ノード構成 & マップ生成フロー

```text
[ 3D-LiDAR / Livox ]
       │
       ▼ sensor_msgs/msg/PointCloud2 (例: /livox/lidar)
┌────────────────────────────────────────────────────────┐
│  pointcloud_to_map Node                                │
│  1. TF2により LiDARフレーム ──> base_linkフレーム へ変換 │
│  2. Z軸 (高さ) フィルタリング (PassThrough)            │
│  3. クラスタリング (EuclideanClusterExtraction)       │
│  4. AABB計算 ＆ 円柱/直方体の判定                      │
│  5. SIGINT (Ctrl+C) 時に JSONへ書き出し                │
└──────────────────────────┬─────────────────────────────┘
                           │
                           ▼ JSON Output (field_map.json)
┌──────────────────────────┴─────────────────────────────┐
│  map_visualizer_node                                   │
│  1. JSONの解析 (FieldObjectの構造体化)                 │
│  2. MarkerArray への変換 (Cube / Cylinder)             │
└──────────────────────────┬─────────────────────────────┘
                           │
                           ▼ visualization_msgs/msg/MarkerArray (/field_map_markers)
                    [ RViz 2 Display ]

```

## 5. ライセンス

本リポジトリは [MITライセンス](https://www.google.com/search?q=./LICENSE) のもとで公開されています。