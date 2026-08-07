#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp> // Thư viện để gửi con số vận tốc

class RovBrain : public rclcpp::Node {
public:
    RovBrain() : Node("rov_brain_node") {
        // 1. Khởi tạo Nhà đài phát sóng (Publisher) cho Thruster 1
        // Webots tự động mở sẵn cổng /thruster_1/cmd_vel chờ bạn gửi tín hiệu
        thruster1_pub_ = this->create_publisher<std_msgs::msg::Float64>("/thruster_1/cmd_vel", 10);

        // 2. Tạo một nhịp tim (Timer) chạy liên tục 10 lần/giây (10Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RovBrain::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Bo nao da thuc tinh! Dang truyen dien cho Thruster 1...");
    }

private:
    // Hàm này sẽ tự động được gọi 10 lần mỗi giây
    void timer_callback() {
        // Tạo một gói hàng chứa dữ liệu
        auto msg = std_msgs::msg::Float64();
        
        // Đặt lệnh vận tốc xoay là 10.0 rad/s
        msg.data = 10.0; 
        
        // Phát sóng gói hàng đi!
        thruster1_pub_->publish(msg);
    }

    // Khai báo các biến nội bộ
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thruster1_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RovBrain>());
    rclcpp::shutdown();
    return 0;
}
