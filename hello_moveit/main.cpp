// #include "tf_test/tf_test.hpp"

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
// #include <serviceinterface.h> // local
#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/SocketAddress.h>
#include <Poco/Net/StreamSocket.h>

namespace vr_track_tcp {

tf2::Transform m_to_mat(const std::vector<float>& vr_pose_matrix) {
    auto rotation = tf2::Matrix3x3(
        vr_pose_matrix[2],
        vr_pose_matrix[0],
        vr_pose_matrix[1],
        vr_pose_matrix[6],
        vr_pose_matrix[4],
        vr_pose_matrix[5],
        vr_pose_matrix[10],
        vr_pose_matrix[8],
        vr_pose_matrix[9]
    );
    auto translation = tf2::Vector3(vr_pose_matrix[3], vr_pose_matrix[7], vr_pose_matrix[11]);
    return tf2::Transform(rotation, translation);
}

class VrTrackTcp: public rclcpp::Node {
private:
    tf2_ros::TransformBroadcaster tf_broadcaster;

public:
    explicit VrTrackTcp(const rclcpp::NodeOptions& options):
        Node("vr_track_tcp", options),
        tf_broadcaster(this) {
        RCLCPP_INFO(this->get_logger(), "Node has been started.");
        this->start_server();
    }

    ~VrTrackTcp() override = default;

private:
    void start_server() {
        try {
            Poco::Net::ServerSocket server_socket(3001);
            RCLCPP_INFO(this->get_logger(), "Server started on port 3001.");

            while (rclcpp::ok()) {
                try {
                    Poco::Net::StreamSocket client_socket = server_socket.acceptConnection();
                    RCLCPP_INFO(this->get_logger(), "Client connected.");
                    this->handle_client(client_socket);
                } catch (Poco::Exception& e) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "#1 Poco Exception: %s",
                        e.displayText().c_str()
                    );
                }
            }
        } catch (Poco::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "#2 Poco Exception: %s", e.displayText().c_str());
        }
    }

    void handle_client(Poco::Net::StreamSocket& socket) {
        std::vector<float> buffer(12);
        while (rclcpp::ok()) {
            // 这是等待式的吧
            size_t bytes_received =
                socket.receiveBytes(buffer.data(), buffer.size() * sizeof(float));
            if (bytes_received == buffer.size() * sizeof(float)) {
                auto transform = m_to_mat(buffer);
                this->publish_transform(transform);
            } else {
                RCLCPP_WARN(this->get_logger(), "Incomplete data received.");
            }
        }
    }

    void publish_transform(const tf2::Transform& transform) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "ref";
        transform_stamped.child_frame_id = "tracker_random";
        const tf2::Quaternion quaternion = transform.getRotation();
        const tf2::Vector3 translation_vector = transform.getOrigin();
        transform_stamped.transform.translation = tf2::toMsg(translation_vector);
        transform_stamped.transform.rotation = tf2::toMsg(quaternion);
        this->tf_broadcaster.sendTransform(transform_stamped);
    }
};

} // namespace vr_track_tcp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vr_track_tcp::VrTrackTcp)
