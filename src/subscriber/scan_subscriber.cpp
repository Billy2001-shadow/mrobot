#include "subscriber/scan_subscriber.hpp"

namespace mrobot_frame {

ScanSubscriber::ScanSubscriber(ros::NodeHandle &nh, std::string topic_name,
                               size_t buff_size)
    : nh_(nh) {
  // laser_frame 和 odom_frame 是为了获取 ？
  nh_.param<std::string>("laser_frame", laser_frame_, "base_laser_link");
  nh_.param<std::string>("odom_frame", odom_frame_, "odom");

  subscriber_ =
      nh_.subscribe(topic_name, buff_size, &ScanSubscriber::msg_callback, this);

  dataset_ = new karto::Dataset();
}

ScanSubscriber::~ScanSubscriber() {
  if (dataset_)
    delete dataset_;
}

void ScanSubscriber::msg_callback(
    const sensor_msgs::LaserScan::ConstPtr &scan_msg_ptr) {
  buff_mutex_.lock();
  LaserScanData laser_scan_data;
  laser_scan_data.time = scan_msg_ptr->header.stamp;
  for (int i = 0; i < scan_msg_ptr->ranges.size(); i++) {
    if (scan_msg_ptr->ranges[i] < scan_msg_ptr->range_min ||
        scan_msg_ptr->ranges[i] > scan_msg_ptr->range_max) {
      double angle =
          scan_msg_ptr->angle_min + i * scan_msg_ptr->angle_increment;
      laser_scan_data.range_readings.push_back(0);
      laser_scan_data.angles_readings.push_back(angle);
      continue;
    }
    double angle = scan_msg_ptr->angle_min + i * scan_msg_ptr->angle_increment;
    laser_scan_data.range_readings.push_back(scan_msg_ptr->ranges[i]);
    laser_scan_data.angles_readings.push_back(angle);
  }
  new_scan_data_.push_back(laser_scan_data);
  buff_mutex_.unlock();

  laser_ = getLaser(scan_msg_ptr);
}

void ScanSubscriber::ParseData(std::deque<LaserScanData> &ranges_data_buff) {
  buff_mutex_.lock();
  if (new_scan_data_.size() > 0) {
    ranges_data_buff.insert(ranges_data_buff.end(), new_scan_data_.begin(),
                            new_scan_data_.end());
    new_scan_data_.clear();
  }
  buff_mutex_.unlock();
}

karto::LaserRangeFinder *
ScanSubscriber::getLaser(const sensor_msgs::LaserScan::ConstPtr &scan) {
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

karto::LaserRangeFinder *ScanSubscriber::getLaser() { return laser_; }

} // namespace mrobot_frame
