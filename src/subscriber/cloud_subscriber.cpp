#include "subscriber/cloud_subscriber.hpp"
#include "sensor_data/ranges_data.hpp"
#include <vector>

namespace mrobot_frame {

CloudSubscriber::CloudSubscriber(ros::NodeHandle &nh, std::string topic_name,
                                 size_t buff_size)
    : nh_(nh), scan_filter_sub_(NULL), scan_filter_(NULL) {
  // scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(
  //     nh, topic_name, 100); //用message_filters库来订阅"/scan"
  // // tf::MessageFilter，订阅激光数据同时和odom_frame之间转换时间同步
  // scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(
  //     *scan_filter_sub_, tf_, odom_frame_,
  //     5);
  //     //创建一个tf::MessageFilter对象，用于将激光数据转换到odom_frame_坐标系下。
  // // scan_filter_注册回调函数laserCallback
  // scan_filter_->registerCallback(
  //     boost::bind(&CloudSubscriber::msg_callback, this, _1)); //
  dataset_ = new karto::Dataset();
  if (!nh_.getParam("laser_frame", laser_frame_))
    laser_frame_ = "base_laser_link";

  subscriber_ = nh_.subscribe(topic_name, buff_size,
                              &CloudSubscriber::msg_callback, this);
}

CloudSubscriber::~CloudSubscriber() {
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
  if (dataset_)
    delete dataset_;
}

void CloudSubscriber::msg_callback(
    const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
  buff_mutex_.lock();
  RangesData ranges_data;
  ranges_data.time = scan_msg->header.stamp;
  double angle_reading = scan_msg->angle_min;
  double angle_increment = (scan_msg->angle_max - scan_msg->angle_min) /
                           (scan_msg->ranges.size() - 1);
  for (std::vector<float>::const_iterator it = scan_msg->ranges.begin();
       it != scan_msg->ranges.end(); it++) {
    ranges_data.readings.push_back(*it);
    ranges_data.angles.push_back(angle_reading);
    angle_reading += angle_increment;
  }

  new_ranges_data_.push_back(ranges_data);
  buff_mutex_.unlock();

  laser_ = getLaser(scan_msg);
}

void CloudSubscriber::ParseData(std::deque<RangesData> &ranges_data_buff) {
  buff_mutex_.lock();

  if (new_ranges_data_.size() > 0) {

    ranges_data_buff.insert(ranges_data_buff.end(), new_ranges_data_.begin(),
                            new_ranges_data_.end());
    new_ranges_data_.clear();
  }

  buff_mutex_.unlock();
}

void CloudSubscriber::ParseData(std::deque<CloudData> &cloud_data_buff) {
  buff_mutex_.lock();

  if (new_cloud_data_.size() > 0) {

    cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(),
                           new_cloud_data_.end());
    new_cloud_data_.clear();
  }

  buff_mutex_.unlock();
}

karto::LaserRangeFinder *
CloudSubscriber::getLaser(const sensor_msgs::LaserScan::ConstPtr &scan) {
  // Check whether we know about this laser yet
  if (lasers_.find(scan->header.frame_id) == lasers_.end()) {
    // New laser; need to create a Karto device for it.

    // Get the laser's pose, relative to base.
    tf::Stamped<tf::Pose> ident;
    tf::Stamped<tf::Transform> laser_pose;
    ident.setIdentity();
    ident.frame_id_ = scan->header.frame_id;
    ident.stamp_ = scan->header.stamp;
    try {
      tf_.transformPose(laser_frame_, ident, laser_pose);
    } catch (tf::TransformException e) {
      ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
               e.what());
      return NULL;
    }

    double yaw = tf::getYaw(laser_pose.getRotation());

    ROS_INFO("laser %s's pose wrt base: %.3f %.3f %.3f",
             scan->header.frame_id.c_str(), laser_pose.getOrigin().x(),
             laser_pose.getOrigin().y(), yaw);
    // To account for lasers that are mounted upside-down,
    // we create a point 1m above the laser and transform it into the laser
    // frame if the point's z-value is <=0, it is upside-down

    tf::Vector3 v;
    v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
    tf::Stamped<tf::Vector3> up(v, scan->header.stamp, laser_frame_);

    try {
      tf_.transformPoint(scan->header.frame_id, up, up);
      ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
    } catch (tf::TransformException &e) {
      ROS_WARN("Unable to determine orientation of laser: %s", e.what());
      return NULL;
    }

    bool inverse = lasers_inverted_[scan->header.frame_id] = up.z() <= 0;
    if (inverse)
      ROS_INFO("laser is mounted upside-down");

    // Create a laser range finder device and copy in data from the first
    // scan
    std::string name = scan->header.frame_id;
    karto::LaserRangeFinder *laser =
        karto::LaserRangeFinder::CreateLaserRangeFinder(
            karto::LaserRangeFinder_Custom, karto::Name(name));
    laser->SetOffsetPose(karto::Pose2(laser_pose.getOrigin().x(),
                                      laser_pose.getOrigin().y(), yaw));
    laser->SetMinimumRange(scan->range_min);
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);
    // TODO: expose this, and many other parameters
    // laser_->SetRangeThreshold(12.0);

    // Store this laser device for later
    lasers_[scan->header.frame_id] = laser;

    // Add it to the dataset, which seems to be necessary
    dataset_->Add(laser);
  }

  return lasers_[scan->header.frame_id];
}

karto::LaserRangeFinder *CloudSubscriber::getLaser() { return laser_; }

} // namespace mrobot_frame
