#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <map>
#include <vector>
#include <string>
#include <algorithm>

using std::placeholders::_1;

static constexpr int GRID_ROWS = 2;
static constexpr int GRID_COLS = 3;
static constexpr int GRID_CELLS = 6;
static const std::string FUSED_FRAME = "fused";

class FusionNode : public rclcpp::Node
{
public:
    FusionNode() : Node("fusion_node")
    {
        declare_parameter("det_inputs", std::vector<std::string>());
        declare_parameter("img_inputs", std::vector<std::string>());
        declare_parameter("output_det", "/fused/detections");
        declare_parameter("output_img", "/fused/debug_image");
        declare_parameter("rate", 10.0);

        det_topics_ = get_parameter("det_inputs").as_string_array();
        img_topics_ = get_parameter("img_inputs").as_string_array();
        output_det_topic_ = get_parameter("output_det").as_string();
        output_img_topic_ = get_parameter("output_img").as_string();
        rate_ = get_parameter("rate").as_double();

        if (det_topics_.empty() || img_topics_.empty())
            throw std::runtime_error("Detection and image topics required");

        rclcpp::QoS qos(10);

        // Detection subscribers
        for (const auto &topic : det_topics_)
        {
            recv_seq_[topic] = 0;
            last_pub_seq_[topic] = 0;

            auto sub = create_subscription<vision_msgs::msg::Detection2DArray>(
                topic, qos,
                [this, topic](vision_msgs::msg::Detection2DArray::SharedPtr msg)
                {
                    latest_det_[topic] = *msg;
                    recv_seq_[topic]++;
                });

            det_subs_.push_back(sub);
        }

        // Image subscribers
        for (const auto &topic : img_topics_)
        {
            auto sub = create_subscription<sensor_msgs::msg::Image>(
                topic, qos,
                [this, topic](sensor_msgs::msg::Image::SharedPtr msg)
                {
                    latest_img_[topic] = msg;
                });

            img_subs_.push_back(sub);
        }

        det_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(output_det_topic_, qos);
        img_pub_ = create_publisher<sensor_msgs::msg::Image>(output_img_topic_, qos);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate_)),
            std::bind(&FusionNode::timerCallback, this));
    }

private:

    void timerCallback()
    {
        publishDetections();
        publishStitchedImage();
    }

    void publishDetections()
    {
        vision_msgs::msg::Detection2DArray out_msg;
        out_msg.header.stamp = now();
        out_msg.header.frame_id = FUSED_FRAME;

        for (const auto &topic : det_topics_)
        {
            if (recv_seq_[topic] > last_pub_seq_[topic])
            {
                const auto &msg = latest_det_[topic];
                out_msg.detections.insert(
                    out_msg.detections.end(),
                    msg.detections.begin(),
                    msg.detections.end());
            }
        }

        det_pub_->publish(out_msg);

        for (const auto &topic : det_topics_)
            last_pub_seq_[topic] = recv_seq_[topic];
    }

    void publishStitchedImage()
    {
        std::vector<cv::Mat> tiles;

        for (size_t i = 0; i < GRID_CELLS; ++i)
        {
            if (i < img_topics_.size() &&
                latest_img_.count(img_topics_[i]) &&
                latest_img_[img_topics_[i]])
            {
                try
                {
                    auto cv_ptr = cv_bridge::toCvCopy(
                        latest_img_[img_topics_[i]], "bgr8");
                    tiles.push_back(cv_ptr->image);
                }
                catch (...)
                {
                    tiles.emplace_back();
                }
            }
            else
            {
                tiles.emplace_back();
            }
        }

        const int target_w = 320;
        const int target_h = 240;

        for (auto &tile : tiles)
        {
            if (tile.empty())
                tile = cv::Mat::zeros(target_h, target_w, CV_8UC3);
            else
                cv::resize(tile, tile, cv::Size(target_w, target_h));
        }

        std::vector<cv::Mat> rows;
        for (int r = 0; r < GRID_ROWS; ++r)
        {
            cv::Mat row;
            cv::hconcat(
                std::vector<cv::Mat>(
                    tiles.begin() + r * GRID_COLS,
                    tiles.begin() + (r + 1) * GRID_COLS),
                row);
            rows.push_back(row);
        }

        cv::Mat panorama;
        cv::vconcat(rows, panorama);

        auto img_msg = cv_bridge::CvImage(
                           std_msgs::msg::Header(), "bgr8", panorama)
                           .toImageMsg();

        img_msg->header.stamp = now();
        img_msg->header.frame_id = FUSED_FRAME;

        img_pub_->publish(*img_msg);
    }

    std::vector<std::string> det_topics_;
    std::vector<std::string> img_topics_;
    std::string output_det_topic_;
    std::string output_img_topic_;
    double rate_;

    std::vector<rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr> det_subs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> img_subs_;

    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::map<std::string, vision_msgs::msg::Detection2DArray> latest_det_;
    std::map<std::string, sensor_msgs::msg::Image::SharedPtr> latest_img_;
    std::map<std::string, int> recv_seq_;
    std::map<std::string, int> last_pub_seq_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FusionNode>());
    rclcpp::shutdown();
    return 0;
}
