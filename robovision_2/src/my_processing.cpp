#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>


class ImageProcessingNode : public rclcpp::Node
{
public:
    ImageProcessingNode() : Node("image_processing"), is_image_(false)
    {
        counter_ = 0;

        //Subscribers
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image", 10, std::bind(&ImageProcessingNode::callback_image, this, std::placeholders::_1));
        
        //Processing
        image_processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&ImageProcessingNode::image_processing, this));

        RCLCPP_INFO(this->get_logger(), "Starting image_processing application in cpp...");
    }

private:

    bool is_image_;
    int counter_;
    int text_pos = 0;
    cv::Mat image_;
    cv::Mat image_gray;
    cv::Mat mask;
    cv::Mat image_filtered;
    sensor_msgs::msg::Image::SharedPtr image_msg_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::TimerBase::SharedPtr image_processing_timer_;
    
    void callback_image(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        is_image_ = true;
    }

    void image_processing()
    {
        if (is_image_){
            cv::inRange(image_, cv::Scalar(0,0,220), cv::Scalar(240,255,240), mask);
            cv::cvtColor(image_, image_gray, CV_BGR2GRAY);
            cv::Mat image_gray_2 = cv::Mat::zeros(image_.rows,image_.cols, CV_8UC1);
            cv::Mat rotated_image = image_.clone();

            for(int i=0; i<image_.rows; i++)
                for(int j=0; j<image_.cols; j++)
                {
                    int gray_val = 0.11*image_.at<cv::Vec3b>(i,j)[0] + 0.59*image_.at<cv::Vec3b>(i,j)[1] + 0.3*image_.at<cv::Vec3b>(i,j)[2];
                    image_gray_2.at<uchar>(i,j) = (unsigned char)gray_val;
                }
            for(int i=0; i<rotated_image.rows; i++)
            {
                for(int j=0; j<rotated_image.cols; j++)
                {
                    if(j >= rotated_image.cols/2 )
                    {
                        cv::Vec<unsigned char, 3> temp = rotated_image.at<cv::Vec3b>(i, j);
                        rotated_image.at<cv::Vec3b>(i, j) = rotated_image.at<cv::Vec3b>(rotated_image.rows-i-1, rotated_image.cols-j-1);
                        rotated_image.at<cv::Vec3b>(rotated_image.rows-i-1, rotated_image.cols-j-1) =  temp;
                    }
                    // swap the (row, col) 3-vector
                    cv::Vec<unsigned char, 3> temp = rotated_image.at<cv::Vec3b>(rotated_image.rows-i - 1, rotated_image.cols - j - 1);
                    rotated_image.at<cv::Vec3b>(rotated_image.rows-i-1, rotated_image.cols-j-1) = rotated_image.at<cv::Vec3b>(i, j);
                    rotated_image.at<cv::Vec3b>(i, j) =  temp;
                }
            }
            image_.copyTo(image_filtered, mask);
            if (counter_ == 0)
            {

			    // std::cout << "size: rows: " << image_.rows << 
                //              ", cols: " << image_.cols << 
                //              ", depth: " << image_.channels() <<
                //              ", size: " << image_.size << std::endl;

                // std::cout << "image_gray size: rows: " << image_gray.rows << 
                //  ", cols: " << image_gray.cols << 
                //  ", depth: " << image_gray.channels() << std::endl;

                int row_id, col_id;
                row_id = image_.rows/2;
                col_id = image_.cols/2;

                std::cout << "pixel value in img at row=" << row_id <<
                            ", col=" << col_id <<
                            " is: " << image_.at<cv::Vec3b>(row_id,col_id) << std::endl;
                std::cout << "pixel value in img_gray at row=" << row_id <<
                            ", col=" << col_id <<
                            " is: " << (int)image_gray.at<uchar>(row_id,col_id) << std::endl;
            }
            cv::putText(image_,
                std::to_string(counter_),
                cv::Point(text_pos, 25), //change these values cv::Point(col_id, row_id)
                cv::FONT_HERSHEY_DUPLEX,
                1.0,
                CV_RGB(255, text_pos, 0), //change these values CV_RGB(red, green, blue)
                2);

            // cv::imshow("gray", image_gray);
            // cv::imshow("mask", mask);
            // cv::imshow("image_filtered", image_filtered);
            // cv::imshow("gray_2", image_gray_2);
            cv::imshow("rotated", rotated_image);
            cv::imshow("view", image_);
            cv::waitKey(1);

            counter_++;
            text_pos++;
            if(text_pos >= image_.rows){
                text_pos = 0;
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageProcessingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
