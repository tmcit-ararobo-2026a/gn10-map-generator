#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// TF2 関連のインクルード
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

#include <nlohmann/json.hpp>
#include <fstream>
#include <iomanip>

using json = nlohmann::json;

class PcdToMapJsonNode : public rclcpp::Node
{
public:
    PcdToMapJsonNode() : Node("pcd_to_map_json_node")
    {
        this->declare_parameter("output_json_path", "field_map.json");
        this->declare_parameter("target_frame", "base_link"); // 点群を変換する目標フレーム
        this->declare_parameter("z_min_crop", 0.05);
        this->declare_parameter("z_max_crop", 2.00);
        this->declare_parameter("cluster_tolerance", 0.08);
        this->declare_parameter("min_cluster_size", 50);
        this->declare_parameter("max_cluster_size", 25000);

        target_frame_ = this->get_parameter("target_frame").as_string();

        // TF2 の初期化
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_pcd_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "input_pointcloud", 10,
            std::bind(&PcdToMapJsonNode::pointCloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PCD to Map JSON Node started. Target frame: %s", target_frame_.c_str());
    }

    ~PcdToMapJsonNode() override
    {
        if (!latest_cloud_ || latest_cloud_->empty()) {
            RCLCPP_WARN(this->get_logger(), "No pointcloud received. Skipping JSON export.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Processing pointcloud in %s frame and saving JSON...", target_frame_.c_str());
        processAndSaveMap();
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::msg::PointCloud2 transformed_msg;

        // target_frame (base_link) への座標変換を試みる
        try {
            // TF変換が利用可能かチェックして変換を実行
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, tf2::TimePointZero);

            tf2::doTransform(*msg, transformed_msg, transform_stamped);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Could not transform pointcloud from %s to %s: %s",
                                 msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(transformed_msg, *cloud);

        if (!cloud->empty()) {
            latest_cloud_ = cloud;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                 "Receiving & transformed PointCloud... (Points: %ld)", latest_cloud_->size());
        }
    }

    void processAndSaveMap()
    {
        // 1. base_link 基準の高さ方向 (Z軸) フィルタリング
        double z_min = this->get_parameter("z_min_crop").as_double();
        double z_max = this->get_parameter("z_max_crop").as_double();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(latest_cloud_);
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

        json json_array = json::array();
        int obj_id = 0;

        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (int idx : indices.indices) {
                cluster->push_back((*cloud_filtered)[idx]);
            }

            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);

            float center_x = (min_pt.x + max_pt.x) / 2.0f;
            float center_y = (min_pt.y + max_pt.y) / 2.0f;
            float param1   = (max_pt.x - min_pt.x) / 2.0f;
            float param2   = (max_pt.y - min_pt.y) / 2.0f;

            std::string type_str = "BOX";
            float aspect_ratio = std::abs(param1 - param2) / std::max(param1, param2);
            if (aspect_ratio < 0.10f) {
                type_str = "CYLINDER";
                param1 = (param1 + param2) / 2.0f;
                param2 = 0.0f;
            }

            json obj;
            obj["comment"] = "auto_extracted_obj_" + std::to_string(obj_id++);
            obj["type"]    = type_str;
            obj["x"]       = center_x;
            obj["y"]       = center_y;
            obj["z_min"]   = min_pt.z;
            obj["z_max"]   = max_pt.z;
            obj["param1"]  = param1;
            obj["param2"]  = param2;

            json_array.push_back(obj);
        }

        // 3. JSONファイルへ保存
        std::string out_path = this->get_parameter("output_json_path").as_string();
        std::ofstream out_file(out_path);
        if (out_file.is_open()) {
            out_file << std::setw(4) << json_array << std::endl;
            RCLCPP_INFO(this->get_logger(), "Successfully saved %ld objects (in %s frame) to %s",
                        json_array.size(), target_frame_.c_str(), out_path.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", out_path.c_str());
        }
    }

    std::string target_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcd_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PcdToMapJsonNode>());
    rclcpp::shutdown();
    return 0;
}