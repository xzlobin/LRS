#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

#include <fstream>
#include <sstream>

using namespace std::chrono_literals;
using namespace std;

struct Waypoint {
    double x, y ,z;
    string type;
    string action;
};

class TemplateDroneControl : public rclcpp::Node
{
public:
    TemplateDroneControl() : Node("template_drone_control_node")
    {
        // Set up ROS publishers, subscribers and service clients
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&TemplateDroneControl::state_cb, this, std::placeholders::_1));
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");
        land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/land");

        rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
        custom_qos.depth = 1;
        custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(custom_qos.history, 1), custom_qos);
        local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/mavros/local_position/pose", qos, std::bind(&TemplateDroneControl::local_pos_cb, this, std::placeholders::_1));

        // Wait for MAVROS SITL connection
        while (rclcpp::ok() && !current_state_.connected)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }

        mavros_msgs::srv::SetMode::Request guided_set_mode_req;
        guided_set_mode_req.custom_mode = "GUIDED";

        mavros_msgs::srv::CommandBool::Request arming;
        arming.value = true;

        while (!set_mode_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the set_mode service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
        }
        auto result = set_mode_client_->async_send_request(std::make_shared<mavros_msgs::srv::SetMode::Request>(guided_set_mode_req));

        while (rclcpp::ok() && current_state_.mode != "GUIDED") {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }
        RCLCPP_INFO(this->get_logger(), "Drone is in GUIDED Mode...");

        while (!arming_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the arming_client_ service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
        }
        //Arm
        auto result_armed = arming_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandBool::Request>(arming));


        RCLCPP_INFO(this->get_logger(), "Sending position command");
        // TODO: Implement position controller and mission commands here
        // Load trajectory 
        loadTrajectory("trajectory.txt"); 
        
        // Start the mission
        timer_ = this->create_wall_timer(500ms, std::bind(&TemplateDroneControl::timerCallback, this));
    }

    void timerCallback() {
    if (!trajectory_.empty()) {
        auto wp = trajectory_.front();
        trajectory_.erase(trajectory_.begin());
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = wp.x;
        pose.pose.position.y = wp.y;
        pose.pose.position.z = wp.z;
        
        if (wp.action == "takeoff") {
            auto takeoff_req = std::make_shared < mavros_msgs::srv::CommandTOL::Request > ();
            takeoff_req - > min_pitch = 0;
            takeoff_req - > yaw = 0;
            takeoff_req - > altitude = wp.z;
            takeoff_client_ - > async_send_request(takeoff_req);
        } else if (wp.action == "land") {
            auto land_req = std::make_shared < mavros_msgs::srv::CommandTOL::Request > ();
            land_req - > min_pitch = 0;
            land_req - > yaw = 0;
            land_req - > altitude = wp.z;
            land_client_ - > async_send_request(land_req);
        } else if (wp.action == "landtakeoff") {
            auto land_req = std::make_shared < mavros_msgs::srv::CommandTOL::Request > ();
            land_req - > min_pitch = 0;
            land_req - > yaw = 0;
            land_req - > altitude = wp.z;
            land_client_ - > async_send_request(land_req);
            auto takeoff_req = std::make_shared < mavros_msgs::srv::CommandTOL::Request > ();
            takeoff_req - > min_pitch = 0;
            takeoff_req - > yaw = 0;
            takeoff_req - > altitude = wp.z;
            takeoff_client_ - > async_send_request(takeoff_req);
        } else if (wp.action == "yaw180") {
            pose.pose.orientation.z = sin(180.0 / 2.0 * M_PI / 180.0); 
            pose.pose.orientation.w = cos(180.0 / 2.0 * M_PI / 180.0); 
        }

        local_pos_pub_ - > publish(pose);
    }
    }

    void loadTrajectory(const std::string &filename) { 
        std::ifstream file(filename);
        std::string line;
         while (std::getline(file, line)) {
             std::istringstream ss(line);
              Waypoint wp; std::string speed, action;
              std::getline(ss, wp.x, ',');
              std::getline(ss, wp.y, ',');
              std::getline(ss, wp.z, ',');
              std::getline(ss, speed, ',');
              std::getline(ss, action, ',');
              wp.speed = speed; wp.action = action;
              trajectory_.push_back(wp); 
    }

private:

    void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped current_local_pos_ = *msg;

        // To obtain the position of the drone use this data fields withing the message, please note, that this is the local position of the drone in the NED frame so it is different to the map frame
        // current_local_pos_.pose.position.x
        // current_local_pos_.pose.position.y
        // current_local_pos_.pose.position.z
        // you can do the same for orientation, but you will not need it for this seminar


        RCLCPP_INFO(this->get_logger(), "Current Local Position: %f, %f, %f", current_local_pos_.pose.position.x, current_local_pos_.pose.position.y, current_local_pos_.pose.position.z);
    }
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Current State: %s", current_state_.mode.c_str());
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;

    std::vector<Waypoint> trajectory_;

    mavros_msgs::msg::State current_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemplateDroneControl>());
    rclcpp::shutdown();
    return 0;
}
