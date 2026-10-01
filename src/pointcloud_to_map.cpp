#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// TF2 関連
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

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

class PcdToMapJsonNode : public rclcpp::Node
{
public:
    PcdToMapJsonNode() : Node("pcd_to_map_json_node")
    {
        this->declare_parameter("output_json_path", "field_map.json");
        this->declare_parameter("target_frame", "map");
        this->declare_parameter("robot_frame", "base_link");
        this->declare_parameter("z_min_crop", 0.05);
        this->declare_parameter("z_max_crop", 2.00);
        
        // ロボット除外用パラメータ
        this->declare_parameter("robot_filter_radius", 0.50);
        this->declare_parameter("robot_box_x", 0.80);
        this->declare_parameter("robot_box_y", 0.80);
        
        // クラスタリング・フィルタリング用パラメータ
        this->declare_parameter("voxel_leaf_size", 0.02);              // 2cmグリッド
        this->declare_parameter("cluster_tolerance", 0.06);            // クラスタ結合距離（小さめにして分離精度向上）
        this->declare_parameter("min_cluster_size", 30);
        this->declare_parameter("max_cluster_size", 25000);

        // 幾何学・物理サイズによるフィルタリング閾値（時間経過で増えるノイズを排除）
        this->declare_parameter("min_object_size_xy", 0.05);           // 最小サイズ 5cm (これ未満の極小ノイズは無視)
        this->declare_parameter("iou_overlap_threshold", 0.30);        // 重複率(IoU)が30%以上のものだけ統合

        target_frame_ = this->get_parameter("target_frame").as_string();
        robot_frame_  = this->get_parameter("robot_frame").as_string();

        accumulated_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_pcd_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "input_pointcloud", 10,
            std::bind(&PcdToMapJsonNode::pointCloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PCD to Map Node initialized. Target: %s, Robot: %s",
                    target_frame_.c_str(), robot_frame_.c_str());
    }

    ~PcdToMapJsonNode() override
    {
        if (!accumulated_cloud_ || accumulated_cloud_->empty()) {
            RCLCPP_WARN(this->get_logger(), "No pointcloud accumulated. Skipping JSON export.");
            return;
        }
        processAndSaveMap();
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::msg::PointCloud2 cloud_robot_frame;

        // 1. LiDAR -> robot_frame (base_link) に変換
        try {
            geometry_msgs::msg::TransformStamped tf_to_robot = 
                tf_buffer_->lookupTransform(robot_frame_, msg->header.frame_id, tf2::TimePointZero);
            tf2::doTransform(*msg, cloud_robot_frame, tf_to_robot);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "TF Error (LiDAR -> %s): %s", robot_frame_.c_str(), ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_robot(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_robot_frame, *pcl_robot);

        double radius = this->get_parameter("robot_filter_radius").as_double();
        double radius_sq = radius * radius;
        double box_x = this->get_parameter("robot_box_x").as_double() / 2.0;
        double box_y = this->get_parameter("robot_box_y").as_double() / 2.0;

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_robot_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // 2. NaN除去 & ロボット領域除外
        for (const auto& pt : pcl_robot->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }

            double dist_sq = pt.x * pt.x + pt.y * pt.y;
            if (dist_sq < radius_sq) {
                continue;
            }

            if (std::abs(pt.x) < box_x && std::abs(pt.y) < box_y) {
                continue;
            }

            pcl_robot_filtered->push_back(pt);
        }

        if (pcl_robot_filtered->empty()) {
            return;
        }

        // 3. フィルタ後の点群を target_frame (map) へ変換
        sensor_msgs::msg::PointCloud2 filtered_msg, cloud_target_frame;
        pcl::toROSMsg(*pcl_robot_filtered, filtered_msg);
        filtered_msg.header.frame_id = robot_frame_;
        filtered_msg.header.stamp = this->now();

        try {
            geometry_msgs::msg::TransformStamped tf_to_target = 
                tf_buffer_->lookupTransform(target_frame_, robot_frame_, tf2::TimePointZero);
            tf2::doTransform(filtered_msg, cloud_target_frame, tf_to_target);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "TF Error (%s -> %s): %s", robot_frame_.c_str(), target_frame_.c_str(), ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_target(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_target_frame, *pcl_target);
        
        // 4. 点群をオンライン蓄積する前に、極端な異常値（ノイズ）をカット
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bounded(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(pcl_target);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(-1.0, 3.0); // 実用的な高さの範囲に制限
        pass_z.filter(*cloud_bounded);

        *accumulated_cloud_ += *cloud_bounded;

        // 蓄積点群全体の範囲も制限してから VoxelGrid に渡す
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_voxel(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_z_acc;
        pass_z_acc.setInputCloud(accumulated_cloud_);
        pass_z_acc.setFilterFieldName("z");
        pass_z_acc.setFilterLimits(-1.0, 3.0);
        pass_z_acc.filter(*cloud_to_voxel);

        double leaf_size = this->get_parameter("voxel_leaf_size").as_double();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud_to_voxel);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.filter(*cloud_downsampled);

        accumulated_cloud_ = cloud_downsampled;
    }

    // 2D IoU (Intersection over Union) の計算
    float computeIoU(const BoundingBox2D& b1, const BoundingBox2D& b2)
    {
        float inter_x_min = std::max(b1.x_min, b2.x_min);
        float inter_x_max = std::min(b1.x_max, b2.x_max);
        float inter_y_min = std::max(b1.y_min, b2.y_min);
        float inter_y_max = std::min(b1.y_max, b2.y_max);

        if (inter_x_max <= inter_x_min || inter_y_max <= inter_y_min) {
            return 0.0f; // 重なりなし
        }

        float inter_area = (inter_x_max - inter_x_min) * (inter_y_max - inter_y_min);
        float area1 = (b1.x_max - b1.x_min) * (b1.y_max - b1.y_min);
        float area2 = (b2.x_max - b2.x_min) * (b2.y_max - b2.y_min);

        return inter_area / (area1 + area2 - inter_area);
    }

    void processAndSaveMap()
    {
        // 1. Z軸切り出し
        double z_min = this->get_parameter("z_min_crop").as_double();
        double z_max = this->get_parameter("z_max_crop").as_double();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(accumulated_cloud_);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min, z_max);
        pass.filter(*cloud_filtered);

        // 2. クラスタリング
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(this->get_parameter("cluster_tolerance").as_double());
        ec.setMinClusterSize(this->get_parameter("min_cluster_size").as_int());
        ec.setMaxClusterSize(this->get_parameter("max_cluster_size").as_int());
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

        double min_size_xy = this->get_parameter("min_object_size_xy").as_double();
        std::vector<BoundingBox2D> raw_boxes;

        // 3. 各クラスタの抽出＆「物理サイズ」によるノイズフィルター
        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (int idx : indices.indices) {
                cluster->push_back((*cloud_filtered)[idx]);
            }

            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);

            float dx = max_pt.x - min_pt.x;
            float dy = max_pt.y - min_pt.y;

            // 幅または奥行きが小さすぎる超極小ノイズ（ゴースト）を排除
            if (dx < min_size_xy && dy < min_size_xy) {
                continue;
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

        // 4. 重なり率（IoU）ベースによる高精度な重複除去（同一物体の被りだけを抑制）
        float iou_thresh = static_cast<float>(this->get_parameter("iou_overlap_threshold").as_double());
        std::vector<BoundingBox2D> final_boxes;
        std::vector<bool> suppressed(raw_boxes.size(), false);

        // 点数が多く情報量が豊富なクラスタ順（または面積順）に優先採用
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

                // 重なり度合 (IoU) が一定値を超えている場合のみ、「同一物体の重複」とみなして抑制
                if (computeIoU(box_a, raw_boxes[idx_b]) > iou_thresh) {
                    suppressed[idx_b] = true;
                }
            }
        }

        // 5. JSON化
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

        std::string out_path = this->get_parameter("output_json_path").as_string();
        std::ofstream out_file(out_path);
        if (out_file.is_open()) {
            out_file << std::setw(4) << json_array << std::endl;
            RCLCPP_INFO(this->get_logger(), "Successfully saved %ld precision objects to %s",
                        json_array.size(), out_path.c_str());
        }
    }

    std::string target_frame_;
    std::string robot_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcd_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PcdToMapJsonNode>());
    rclcpp::shutdown();
    return 0;
}