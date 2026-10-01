#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>

using json = nlohmann::json;

enum ObjectType { CYLINDER, BOX, VISUAL_BOX };

struct FieldObject {
    std::string comment;
    ObjectType type;
    float center_x, center_y;
    float z_min, z_max;
    float param1, param2;
};

class MapVisualizerNode : public rclcpp::Node
{
public:
    MapVisualizerNode() : Node("map_visualizer_node")
    {
        // パラメータ宣言
        this->declare_parameter("json_path", "field_map.json");
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("publish_rate_hz", 1.0); // 表示用なので1Hz程度で十分

        map_frame_ = this->get_parameter("map_frame").as_string();
        std::string json_path = this->get_parameter("json_path").as_string();

        // Publisher の作成
        pub_map_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("field_map_markers", 10);

        // JSONファイルの読み込み
        map_objects_ = loadFromJSON(json_path);
        RCLCPP_INFO(this->get_logger(), "Loaded %ld objects from %s", map_objects_.size(), json_path.c_str());

        // 定期タイマーの作成 (定期的にMarkerArrayを配信)
        double rate_hz = this->get_parameter("publish_rate_hz").as_double();
        auto timer_period = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = this->create_wall_timer(
            timer_period, std::bind(&MapVisualizerNode::publishFieldMapMarkers, this));
    }

private:
    std::vector<FieldObject> loadFromJSON(const std::string& file_path)
    {
        std::vector<FieldObject> map;
        std::ifstream f(file_path);
        if (!f.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open map file: %s", file_path.c_str());
            return map;
        }

        try {
            json j = json::parse(f);
            for (const auto& item : j) {
                FieldObject obj;
                if (item.contains("comment")) {
                    obj.comment = item.at("comment").get<std::string>();
                } else {
                    obj.comment = "";
                }
                std::string type_str = item.at("type").get<std::string>();
                obj.type             = (type_str == "CYLINDER") ? CYLINDER :
                                       (type_str == "VISUAL_BOX") ? VISUAL_BOX : BOX;
                obj.center_x         = item.at("x").get<float>();
                obj.center_y         = item.at("y").get<float>();
                obj.z_min            = item.at("z_min").get<float>();
                obj.z_max            = item.at("z_max").get<float>();
                obj.param1           = item.at("param1").get<float>();
                obj.param2           = item.at("param2").get<float>();
                map.push_back(obj);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
        }

        return map;
    }

    void publishFieldMapMarkers()
    {
        if (map_objects_.empty()) {
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
        for (const auto& obj : map_objects_) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = map_frame_;
            marker.header.stamp    = this->now();
            marker.ns              = "field_objects";
            marker.id              = id++;
            marker.action          = visualization_msgs::msg::Marker::ADD;

            marker.color.r = 0.1f;
            marker.color.g = 0.8f;
            marker.color.b = 0.4f;
            marker.color.a = 0.6f;

            const float height        = obj.z_max - obj.z_min;
            marker.pose.position.x    = obj.center_x;
            marker.pose.position.y    = obj.center_y;
            marker.pose.position.z    = obj.z_min + height / 2.0f;
            marker.pose.orientation.w = 1.0;

            if (obj.type == BOX || obj.type == VISUAL_BOX) {
                marker.type    = visualization_msgs::msg::Marker::CUBE;
                marker.scale.x = obj.param1 * 2.0f;
                marker.scale.y = obj.param2 * 2.0f;
                marker.scale.z = height;
            } else if (obj.type == CYLINDER) {
                marker.type    = visualization_msgs::msg::Marker::CYLINDER;
                marker.scale.x = obj.param1 * 2.0f;
                marker.scale.y = obj.param1 * 2.0f;
                marker.scale.z = height;
                marker.color.r = 0.9f;
                marker.color.g = 0.3f;
                marker.color.b = 0.1f;
            }
            marker_array.markers.push_back(marker);
        }
        pub_map_markers_->publish(marker_array);
    }

    std::string map_frame_;
    std::vector<FieldObject> map_objects_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_map_markers_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}