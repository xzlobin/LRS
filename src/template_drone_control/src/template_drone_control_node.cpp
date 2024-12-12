#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/msg/string.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <regex>

//#include <LuaCpp.hpp>

using namespace std::chrono_literals;
using namespace std;

//using namespace LuaCpp;
//using namespace LuaCpp::Registry;
//using namespace LuaCpp::Engine;

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
        //lua_ = &lua;
        // Set up ROS publishers, subscribers and service clients
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&TemplateDroneControl::state_cb, this, std::placeholders::_1));

        lua_sub_ = this->create_subscription<std_msgs::msg::String>(
            "lua_topic", 10, std::bind(&TemplateDroneControl::interpret, this, std::placeholders::_1));

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

        this->prepare_take_off_init();

        RCLCPP_INFO(this->get_logger(), "DRONE READY");
    }

    bool is_at_waypoint(const Waypoint& wp, const string& type)
    {
        double eps = (type == "hard" ? 0.1 : 1);
        return (abs(current_local_pos_.pose.position.x-wp.x) < eps &&
                abs(current_local_pos_.pose.position.y-wp.y) < eps &&
                abs(current_local_pos_.pose.position.z-wp.z) < eps);
    }

    void timerCallback() {
    if (!trajectory_.empty()) {
        auto wp = trajectory_.front();
        active_waypoint_ = wp;
        trajectory_.erase(trajectory_.begin());
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = wp.x;
        pose.pose.position.y = wp.y;
        pose.pose.position.z = wp.z;

        if (wp.action == "takeoff") {
            auto takeoff_req = std::make_shared < mavros_msgs::srv::CommandTOL::Request > ();
            takeoff_req -> min_pitch = 0;
            takeoff_req -> yaw = 0;
            takeoff_req -> altitude = wp.z;
            takeoff_client_ -> async_send_request(takeoff_req);
            RCLCPP_INFO(this->get_logger(), "Taking off...");
            while( rclcpp::ok() && abs(current_local_pos_.pose.position.z-wp.z) > 0.1 )
            {
                //rclcpp::spin_some(this->get_node_base_interface());
                std::this_thread::sleep_for(100ms);
            }
            std::this_thread::sleep_for(1000ms);
            RCLCPP_INFO(this->get_logger(), "Take off done: -> %f, %f, %f, %s, %s", wp.x, wp.y, wp.z, wp.type.c_str(), wp.action.c_str());
        }
        if (wp.action.find("yaw") != string::npos) {
            regex rg("yaw(\\d+)");
            smatch match;
            regex_search(wp.action, match, rg);
            auto yaw = stod(match[1]);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw * M_PI / 180);
            pose.pose.orientation = tf2::toMsg(q);
            RCLCPP_INFO(this->get_logger(), "Yaw adjustment: %f", yaw);
        }

        //--------------------------------------//
        RCLCPP_INFO(this->get_logger(), "Publishing point: %f, %f, %f, %s, %s", wp.x, wp.y, wp.z, wp.type.c_str(), wp.action.c_str());
        if (wp.action.find("circleR") != string::npos) {
            regex rg("circleR(\\d+)");
            smatch match;
            regex_search(wp.action, match, rg);
            auto R = stod(match[1]);
            
            RCLCPP_INFO(this->get_logger(), "Circle R: %f", R);
            circle_around(wp.x, wp.y, wp.z, R);
        }
        else
        {
            local_pos_pub_ -> publish(pose);
            while( rclcpp::ok() && !is_at_waypoint(wp, wp.type) )
            {
                check_state();
                //rclcpp::spin_some(this->get_node_base_interface());
                std::this_thread::sleep_for(10ms);
            }
            RCLCPP_INFO(this->get_logger(), "Reached point: %f, %f, %f, %s, %s", wp.x, wp.y, wp.z, wp.type.c_str(), wp.action.c_str());
        }
        //--------------------------------------//
        
        if (wp.action == "land") {
            auto land_req = std::make_shared < mavros_msgs::srv::CommandTOL::Request > ();
            land_req -> min_pitch = 0;
            land_req -> yaw = 0;
            land_req -> altitude = wp.z;
            land_client_ -> async_send_request(land_req);
            RCLCPP_INFO(this->get_logger(), "Landing...");
            while( rclcpp::ok() && abs(current_local_pos_.pose.position.z-wp.z) > 0.1 )
            {
                //rclcpp::spin_some(this->get_node_base_interface());
                std::this_thread::sleep_for(100ms);
            }
            std::this_thread::sleep_for(1000ms);
            RCLCPP_INFO(this->get_logger(), "Landing done: <- %f, %f, %f, %s, %s", wp.x, wp.y, wp.z, wp.type.c_str(), wp.action.c_str());
        } else if (wp.action == "landtakeoff") {
            auto land_req = std::make_shared < mavros_msgs::srv::CommandTOL::Request > ();
            land_req -> min_pitch = 0;
            land_req -> yaw = 0;
            land_req -> altitude = 0;
            land_client_ -> async_send_request(land_req);
            RCLCPP_INFO(this->get_logger(), "Landing...");
            while( rclcpp::ok() && abs(current_local_pos_.pose.position.z) > 0.1 )
            {
                //rclcpp::spin_some(this->get_node_base_interface());
                std::this_thread::sleep_for(100ms);
            }
            std::this_thread::sleep_for(10000ms);
            RCLCPP_INFO(this->get_logger(), "Landing done, taking off...");

            prepare_take_off();

            auto takeoff_req = std::make_shared < mavros_msgs::srv::CommandTOL::Request > ();
            takeoff_req -> min_pitch = 0;
            takeoff_req -> yaw = 0;
            takeoff_req -> altitude = wp.z;
            takeoff_client_ -> async_send_request(takeoff_req);
            while( rclcpp::ok() && abs(current_local_pos_.pose.position.z-wp.z) > 0.1 )
            {
                //rclcpp::spin_some(this->get_node_base_interface());
                std::this_thread::sleep_for(100ms);
            }
            std::this_thread::sleep_for(1000ms);
            RCLCPP_INFO(this->get_logger(), "Take off done: %f, %f, %f, %s, %s", wp.x, wp.y, wp.z, wp.type.c_str(), wp.action.c_str());
        }

        if (wp.type == "hard") 
        {
            this_thread::sleep_for(5000ms);
        }
    }
    }

    void prepare_take_off_init()
    {
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
            RCLCPP_INFO(this->get_logger(), "Waiting for arming service...");
        }
        //Arm
        auto result_armed = arming_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandBool::Request>(arming));
        while (rclcpp::ok() && !current_state_.armed) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }
        RCLCPP_INFO(this->get_logger(), "Drone is ARMED...");
    }

    void prepare_take_off()
    {
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
            //rclcpp::spin_some(this->get_node_base_interface());
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
            RCLCPP_INFO(this->get_logger(), "Waiting for arming service...");
        }
        //Arm
        auto result_armed = arming_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandBool::Request>(arming));
        while (rclcpp::ok() && !current_state_.armed) {
            //rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }
        RCLCPP_INFO(this->get_logger(), "Drone is ARMED...");
    }

    void loadTrajectory(const std::string &filename) { 
        std::ifstream file(filename);
        std::string line;
        cout << "Loading trajectory:\n";
         while (std::getline(file, line)) {
              std::istringstream ss(line);
              Waypoint wp; std::string type, action;
              string sx, sy, sz;
              std::getline(ss, sx, ',');
              std::getline(ss, sy, ',');
              std::getline(ss, sz, ',');
              std::getline(ss, type, ',');
              std::getline(ss, action, ',');
              wp.x = stod(sx);
              wp.y = stod(sy);
              wp.z = stod(sz);
              wp.type = type; wp.action = action;
              trajectory_.push_back(wp); 
              cout << wp.x << ", " << wp.y << ", " << wp.z << ", " << wp.type << ", " << wp.action << "\n";
        }
    }

    void check_state(){
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = current_local_pos_.pose.position.x;
        pose.pose.position.y = current_local_pos_.pose.position.y;
        pose.pose.position.z = current_local_pos_.pose.position.z;

        if(replay_state_ != 1)
        {
            local_pos_pub_ -> publish(pose);
            while(replay_state_ != 1){
                this_thread::sleep_for(100ms);
            }
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = active_waypoint_.x;
            pose.pose.position.y = active_waypoint_.y;
            pose.pose.position.z = active_waypoint_.z;
            local_pos_pub_ -> publish(pose);
        }
    }

    void drone_stop()
    {
        // request stop
        replay_state_ = 0;
    }

    void drone_continue()
    {
        // replay active
        replay_state_ = 1;
    }

    void circle_around(double x_, double y_, double z_, double radius_)
    {
        double speed_ = 1;
        auto message = geometry_msgs::msg::PoseStamped();   
        double phase = 0;
        int delay_ms = 50;
        std::chrono::milliseconds duration(delay_ms);
        auto start_time = this->now();

        while(phase < 2*M_PI) speed_ * elapsed_time
        {
            double x = x_ + radius_ * std::cos(phase);
            double y = y_ + radius_ * std::sin(phase);
            double z = z_;
            double yaw = std::atan2(target_y_ - y, target_x_ - x);

            message.pose.position.x = x;
            message.pose.position.y = y;
            message.pose.position.z = z;
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            message.pose.orientation = tf2::toMsg(q);
            message.header.stamp = this->now();
            publisher_->publish(message);

            Waypoint wp;
            wp.x = x;
            wp.y = y;
            wp.z = z;
            wp.type = "hard"; wp.action = "";
            while( rclcpp::ok() && !is_at_waypoint(wp,"hard") )
            {
                std::this_thread::sleep_for(duration);
            }
            if(phase == 0.){
                start_time = this->now();
            }
            auto current_time = this->now();
            double elapsed_time = (current_time - start_time).seconds();
            phase += speed_ * elapsed_time;
        }
    }

    

    void start_spinning() {
        spin_thread_ = std::thread([this]() {
            rclcpp::spin(this->get_node_base_interface());
        });
    }
    void stop_spinning() {
        rclcpp::shutdown();
        if (spin_thread_.joinable()){
            spin_thread_.join();
        }
    }
    void do_thing()
    {
        // Load trajectory 
        loadTrajectory("/home/nzuri/trajectory.txt"); 
        
        while(rclcpp::ok())
        {
            timerCallback();
            this_thread::sleep_for(100ms);
        }
    }

private:
    void spin_some()
    {
        rclcpp::spin_some(this->get_node_base_interface());
    }

    void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        //geometry_msgs::msg::PoseStamped current_local_pos_ = *msg;
        this->current_local_pos_ = *msg;

        // To obtain the position of the drone use this data fields withing the message, please note, that this is the local position of the drone in the NED frame so it is different to the map frame
        // current_local_pos_.pose.position.x
        // current_local_pos_.pose.position.y
        // current_local_pos_.pose.position.z
        // you can do the same for orientation, but you will not need it for this seminar


        //RCLCPP_INFO(this->get_logger(), "Current Local Position: %f, %f, %f", current_local_pos_.pose.position.x, current_local_pos_.pose.position.y, current_local_pos_.pose.position.z);
    }
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
        //RCLCPP_INFO(this->get_logger(), "Current State: %s", current_state_.mode.c_str());
    }

    void interpret(const std_msgs::msg::String::SharedPtr msg)
    {
        auto& str = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received string: %s", str.c_str());

        try{
            if(str.find("stop") != string::npos)
            {
                drone_stop();
            }
            if(str.find("continue") != string::npos)
            {
                drone_continue();
            }
            if (str.find("circleR") != string::npos) {
                regex rg("circleR(\\d+)");
                smatch match;
                regex_search(str, match, rg);
                auto R = stod(match[1]);
            
                RCLCPP_INFO(this->get_logger(), "Circle R: %f", R);
                circle_around(current_local_pos_.pose.position.x, current_local_pos_.pose.position.y, current_local_pos_.pose.position.z, R);
            }
        } catch (std::runtime_error& e){
            RCLCPP_INFO(this->get_logger(), "Critical ERROR: %s", e.what());
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
    geometry_msgs::msg::PoseStamped current_local_pos_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lua_sub_;

    std::vector<Waypoint> trajectory_;
    Waypoint active_waypoint_;

    int replay_state_ = 0;

    mavros_msgs::msg::State current_state_;

    std::thread spin_thread_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TemplateDroneControl>();

    node->start_spinning();

    node->do_thing();

    node->stop_spinning();

    //rclcpp::spin(std::make_shared<TemplateDroneControl>());
    //rclcpp::shutdown();
    return 0;
}
