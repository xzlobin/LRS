#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>

class LuaPublisher : public rclcpp::Node {
public:
    LuaPublisher() : Node("lua_publisher_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("lua_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&LuaPublisher::publish_message, this));
    }

private:
    void publish_message() {
        std::string input;
        std::cout << "> ";
        std::getline(std::cin, input);

        auto message = std_msgs::msg::String();
        message.data = input;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LuaPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
