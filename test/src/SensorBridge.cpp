#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Define a class for the SensorBridge node that sends PoseStamped messages
class SensorBridge : public rclcpp::Node {
public:
    SensorBridge() : Node("SensorBridge") {
        // Create a publisher on the "move_to" topic
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("move_to", 10);

        // Create a timer to periodically publish messages (every 1 second)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&SensorBridge::publishPose, this)
        );
    }

private:
    // Function to create and publish a PoseStamped message
    auto generatePoseMsg(float x, float y, float z, float qx, float qy, float qz, float qw) {
        geometry_msgs::msg::Pose msg;
        msg.orientation.x = qx;
        msg.orientation.y = qy;
        msg.orientation.z = qz;
        msg.orientation.w = qw;
        msg.position.x = x;
        msg.position.y = y;
        msg.position.z = z;
        return msg;
    }

    // Define dimensions and positions (can be adjusted based on your requirements)
    const float jenga_length = 0.075;
    const float jenga_width = 0.025;
    const float jenga_height = 0.015;
    const float x_first_block_center_point = 0.15;
    const float y_first_block_center_point = 0.35;
    const float offset_from_obstacle = 0.2; // Define your offset value
    const int j = 6; // Example multiplier

    void publishPose() {
        auto msg = geometry_msgs::msg::PoseStamped();  // Create the message
        msg.header.frame_id = "world";  // Set the frame ID
        msg.header.stamp = this->get_clock()->now();  // Set the timestamp

        // Set position values
        msg.pose = generatePoseMsg(
            x_first_block_center_point - jenga_length / 2 - offset_from_obstacle, //x
            y_first_block_center_point,                                           //y
            j * jenga_height,                                                     //z
            0.0, -0.707, 0.0, -0.707                                              // Quaternion values point at positive x
        );

        publisher_->publish(msg);  // Publish the message
    }

    // Members to store the publisher and timer objects
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// Main function to initialize the ROS2 node and start spinning
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);  // Initialize ROS2
    rclcpp::spin(std::make_shared<SensorBridge>());  // Start the node and publish messages
    rclcpp::shutdown();  // Clean up and exit
    return 0;
}
