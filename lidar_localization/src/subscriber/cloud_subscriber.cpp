/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#include "lidar_localization/subscriber/cloud_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization {
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    CloudData cloud_data;
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));

    new_cloud_data_.push_back(cloud_data);
}

void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff) {
    if (new_cloud_data_.size() > 0) {
        //在cloud_data_buff.end()之前插入一份new_cloud_data_.begin(), new_cloud_data_.end()之间的拷贝，并返回第一个新元素的位置
        cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        // new_cloud_data_.clear();
        new_cloud_data_.erase(new_cloud_data_.begin(), new_cloud_data_.end());
    }
}
} // namespace data_input