#include <rclcpp/rclcpp.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <algorithm>

using json = nlohmann::json;

struct BoundingBox2D {
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
    size_t point_count;
    std::string type;
};

class PcdToMapOfflineNode : public rclcpp::Node
{
public:
    PcdToMapOfflineNode() : Node("pcd_to_map_offline_node")
    {
        // 入出力パス
        this->declare_parameter("input_pcd_path", "fast_lio_map.pcd");
        this->declare_parameter("output_json_path", "field_map.json");

        // TF 座標変換パラメータ (デフォルト値を今回のセンサ設置設定に反映)
        this->declare_parameter("tf_x", 0.20);
        this->declare_parameter("tf_y", -0.25);
        this->declare_parameter("tf_z", 1.09);
        this->declare_parameter("tf_roll_rad", 3.13);     // 上下反転
        this->declare_parameter("tf_pitch_rad", -0.273);  // 傾き補正
        this->declare_parameter("tf_yaw_rad", 0.0);

        // 高さ（Z軸）フィルタリング範囲 (base_link 基準)
        this->declare_parameter("z_min_crop", 0.05);
        this->declare_parameter("z_max_crop", 2.00);

        // クラスタリング・密度設定
        this->declare_parameter("voxel_leaf_size", 0.02);              // 2cm グリッド
        this->declare_parameter("cluster_tolerance", 0.06);            // 6cm
        this->declare_parameter("min_cluster_size", 30);
        this->declare_parameter("max_cluster_size", 25000);

        // 幾何学・重複排除フィルタ
        this->declare_parameter("min_object_size_xy", 0.05);           // 最小物理サイズ 5cm
        this->declare_parameter("iou_overlap_threshold", 0.30);        // IoU 重なり率 30% 以上で重複抑制

        processPcdFile();
    }

private:
    float computeIoU(const BoundingBox2D& b1, const BoundingBox2D& b2)
    {
        float inter_x_min = std::max(b1.x_min, b2.x_min);
        float inter_x_max = std::min(b1.x_max, b2.x_max);
        float inter_y_min = std::max(b1.y_min, b2.y_min);
        float inter_y_max = std::min(b1.y_max, b2.y_max);

        if (inter_x_max <= inter_x_min || inter_y_max <= inter_y_min) {
            return 0.0f;
        }

        float inter_area = (inter_x_max - inter_x_min) * (inter_y_max - inter_y_min);
        float area1 = (b1.x_max - b1.x_min) * (b1.y_max - b1.y_min);
        float area2 = (b2.x_max - b2.x_min) * (b2.y_max - b2.y_min);

        return inter_area / (area1 + area2 - inter_area);
    }

    void processPcdFile()
    {
        std::string pcd_path = this->get_parameter("input_pcd_path").as_string();
        std::string json_path = this->get_parameter("output_json_path").as_string();

        RCLCPP_INFO(this->get_logger(), "Loading PCD file from: %s", pcd_path.c_str());

        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *raw_cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read PCD file: %s", pcd_path.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded PCD successfully. Points: %ld", raw_cloud->size());

        // 1. TF 座標変換 (LiDAR 座標系 -> base_link 座標系)
        double tx = this->get_parameter("tf_x").as_double();
        double ty = this->get_parameter("tf_y").as_double();
        double tz = this->get_parameter("tf_z").as_double();
        double roll  = this->get_parameter("tf_roll_rad").as_double();
        double pitch = this->get_parameter("tf_pitch_rad").as_double();
        double yaw   = this->get_parameter("tf_yaw_rad").as_double();

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << tx, ty, tz;
        transform.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
                       * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
                       * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*raw_cloud, *cloud_transformed, transform);

        // 2. PassThrough で高さ (Z軸) 制限 (base_link 基準)
        double z_min = this->get_parameter("z_min_crop").as_double();
        double z_max = this->get_parameter("z_max_crop").as_double();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z_cropped(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_transformed);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min, z_max);
        pass.filter(*cloud_z_cropped);

        if (cloud_z_cropped->empty()) {
            RCLCPP_WARN(this->get_logger(), "No points found within specified Z range (%.2f ~ %.2f)", z_min, z_max);
            return;
        }

        // 3. VoxelGrid ダウンサンプリング
        double leaf_size = this->get_parameter("voxel_leaf_size").as_double();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud_z_cropped);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.filter(*cloud_downsampled);

        RCLCPP_INFO(this->get_logger(), "Filtered points (TF applied, Z-cropped & Downsampled): %ld", cloud_downsampled->size());

        // 4. Euclidean Clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(this->get_parameter("cluster_tolerance").as_double());
        ec.setMinClusterSize(this->get_parameter("min_cluster_size").as_int());
        ec.setMaxClusterSize(this->get_parameter("max_cluster_size").as_int());
        ec.setInputCloud(cloud_downsampled);
        ec.extract(cluster_indices);

        double min_size_xy = this->get_parameter("min_object_size_xy").as_double();
        std::vector<BoundingBox2D> raw_boxes;

        // 5. クラスタごとの形状解析とノイズ除去
        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (int idx : indices.indices) {
                cluster->push_back((*cloud_downsampled)[idx]);
            }

            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);

            float dx = max_pt.x - min_pt.x;
            float dy = max_pt.y - min_pt.y;

            if (dx < min_size_xy && dy < min_size_xy) {
                continue; // 極小ノイズカット
            }

            float param1 = dx / 2.0f;
            float param2 = dy / 2.0f;

            std::string type_str = "BOX";
            float aspect_ratio = std::abs(param1 - param2) / std::max(param1, param2);
            if (aspect_ratio < 0.10f) {
                type_str = "CYLINDER";
            }

            BoundingBox2D box;
            box.x_min = min_pt.x;
            box.x_max = max_pt.x;
            box.y_min = min_pt.y;
            box.y_max = max_pt.y;
            box.z_min = min_pt.z;
            box.z_max = max_pt.z;
            box.point_count = cluster->size();
            box.type = type_str;

            raw_boxes.push_back(box);
        }

        // 6. IoU による重複抑制 (NMS)
        float iou_thresh = static_cast<float>(this->get_parameter("iou_overlap_threshold").as_double());
        std::vector<BoundingBox2D> final_boxes;
        std::vector<bool> suppressed(raw_boxes.size(), false);

        std::vector<size_t> indices(raw_boxes.size());
        for (size_t i = 0; i < indices.size(); ++i) indices[i] = i;
        std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
            return raw_boxes[a].point_count > raw_boxes[b].point_count;
        });

        for (size_t idx_a : indices) {
            if (suppressed[idx_a]) continue;

            const auto& box_a = raw_boxes[idx_a];
            final_boxes.push_back(box_a);

            for (size_t idx_b : indices) {
                if (suppressed[idx_b] || idx_a == idx_b) continue;

                if (computeIoU(box_a, raw_boxes[idx_b]) > iou_thresh) {
                    suppressed[idx_b] = true;
                }
            }
        }

        // 7. JSON 保存
        json json_array = json::array();
        int obj_id = 0;

        for (const auto& box : final_boxes) {
            float center_x = (box.x_min + box.x_max) / 2.0f;
            float center_y = (box.y_min + box.y_max) / 2.0f;
            float param1   = (box.x_max - box.x_min) / 2.0f;
            float param2   = (box.y_max - box.y_min) / 2.0f;

            if (box.type == "CYLINDER") {
                param1 = (param1 + param2) / 2.0f;
                param2 = 0.0f;
            }

            json obj;
            obj["comment"] = "auto_extracted_obj_" + std::to_string(obj_id++);
            obj["type"]    = box.type;
            obj["x"]       = center_x;
            obj["y"]       = center_y;
            obj["z_min"]   = box.z_min;
            obj["z_max"]   = box.z_max;
            obj["param1"]  = param1;
            obj["param2"]  = param2;

            json_array.push_back(obj);
        }

        std::ofstream out_file(json_path);
        if (out_file.is_open()) {
            out_file << std::setw(4) << json_array << std::endl;
            RCLCPP_INFO(this->get_logger(), "Successfully generated JSON map! Saved %ld objects to %s",
                        json_array.size(), json_path.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open output JSON file: %s", json_path.c_str());
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PcdToMapOfflineNode>();
    rclcpp::shutdown();
    return 0;
}