#include <bitset>
#include <locale>
#include <sstream>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <openvr.h>

#include "VRTracking.h"

/**
 * @note ROS parameters:
 *   - update_rate (double): rate of the broadcast, in Hertz
 */
class ViveTrackerNode
{
public:
  explicit ViveTrackerNode(ros::NodeHandle const& nh);
  ~ViveTrackerNode() = default;

  void run();

  ViveTrackerNode(const ViveTrackerNode &) = delete;
  ViveTrackerNode(ViveTrackerNode &&) = delete;
  ViveTrackerNode &operator=(const ViveTrackerNode &) = delete;
  ViveTrackerNode &operator=(ViveTrackerNode &&) = delete;

private:
  void sendViveWorldFrame();

  static std::bitset<vr::TrackedDeviceClass::TrackedDeviceClass_Max> parseTrackedClasses(std::string const& input);

private:
  ros::NodeHandle _nh;
  ros::Publisher _odometryPub;

  tf2_ros::TransformBroadcaster _transformBroadcaster;

  VRTracking _vrTracking;

  /// Frequency of the update loop, in Hertz.
  double _updateRate = 20.0;
  std::bitset<vr::TrackedDeviceClass::TrackedDeviceClass_Max> _trackedClasses;
};

/// Class to parse comma delimited number list.
class comma_ctype: public std::ctype<char>
{
public:
  comma_ctype(size_t refs = 0)
          : std::ctype<char>(_comma_mask, refs)
  {
    std::copy_n(classic_table(), table_size, _comma_mask);
    _comma_mask[','] = (mask)space;
  }

private:
  mask _comma_mask[table_size];
};

ViveTrackerNode::ViveTrackerNode(ros::NodeHandle const& nh)
        : _nh { nh }
        , _odometryPub { _nh.advertise<nav_msgs::Odometry>("odometry", 20) }
        , _updateRate { _nh.param("update_rate", 30.0) }
        , _trackedClasses { parseTrackedClasses(_nh.param("tracked_classes", std::string("1,2,3"))) }
{
  if (_trackedClasses.none()) {
    throw std::invalid_argument("No device classes selected");
  }

  ROS_DEBUG_STREAM("vive_tracker_node started, update_rate is " << _updateRate << " and followed classes are " << _trackedClasses);
}

std::bitset<vr::TrackedDeviceClass::TrackedDeviceClass_Max> ViveTrackerNode::parseTrackedClasses(std::string const& input)
{
  std::bitset<vr::TrackedDeviceClass::TrackedDeviceClass_Max> trackedClasses;
  std::istringstream trackedClassesInput { input };
  std::locale commaLocale(std::locale::classic(), new comma_ctype);
  trackedClassesInput.imbue(commaLocale);

  std::size_t classNumber;
  trackedClassesInput >> classNumber;

  while (trackedClassesInput)
  {
    if (classNumber > 0 && classNumber < trackedClasses.size())
      trackedClasses[classNumber] = true;
    else
      ROS_WARN_STREAM('"' << classNumber << "\" is not a valid class number. The valid classes are between 1 and " << trackedClasses.size() - 1);

    trackedClassesInput >> classNumber;
  }

  return trackedClasses;
}

void ViveTrackerNode::run()
{
  sendViveWorldFrame();

  ros::Rate loopRate(_updateRate);

  while (ros::ok())
  {
    _vrTracking.update();

    for (vr::TrackedDeviceIndex_t index = 0; index < vr::k_unMaxTrackedDeviceCount; ++index)
    {
      float transform[3][4];

      if (!_vrTracking.isConnected(index)
          || !_vrTracking.isPoseValid(index)
          || !_trackedClasses[_vrTracking.getDeviceClass(index)]
          || _vrTracking.getDevicePose(index, transform) != VRTracking::TrackingState::Ok)
        continue;

      tf2::Quaternion quaternion;
      tf2::Matrix3x3 rotMatrix { transform[0][0], transform[0][1], transform[0][2],
                                 transform[1][0], transform[1][1], transform[1][2],
                                 transform[2][0], transform[2][1], transform[2][2] };
      rotMatrix.getRotation(quaternion);

      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.frame_id = "vive_world";
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.child_frame_id = std::string("vive_device_") + std::to_string(index);
      transformStamped.transform.translation.x = transform[0][3];
      transformStamped.transform.translation.y = transform[1][3];
      transformStamped.transform.translation.z = transform[2][3];
      transformStamped.transform.rotation.x = quaternion.x();
      transformStamped.transform.rotation.y = quaternion.y();
      transformStamped.transform.rotation.z = quaternion.z();
      transformStamped.transform.rotation.w = quaternion.w();

      _transformBroadcaster.sendTransform(transformStamped);

      float velocity[3];
      float angularVelocity[3];
      if (_vrTracking.getDeviceVelocity(index, velocity) != VRTracking::TrackingState::Ok
          || _vrTracking.getDeviceAngularVelocity(index, angularVelocity) != VRTracking::TrackingState::Ok)
        continue;

      nav_msgs::Odometry::Ptr odometry = boost::make_shared<nav_msgs::Odometry>();
      odometry->header.stamp = transformStamped.header.stamp;
      odometry->header.frame_id = transformStamped.header.frame_id;
      odometry->child_frame_id = transformStamped.child_frame_id;
      odometry->pose.pose.position.x = transformStamped.transform.translation.x;
      odometry->pose.pose.position.y = transformStamped.transform.translation.y;
      odometry->pose.pose.position.z = transformStamped.transform.translation.z;
      odometry->pose.pose.orientation = transformStamped.transform.rotation;
      odometry->twist.twist.linear.x = velocity[0];
      odometry->twist.twist.linear.y = velocity[1];
      odometry->twist.twist.linear.z = velocity[2];
      odometry->twist.twist.angular.x = angularVelocity[0];
      odometry->twist.twist.angular.y = angularVelocity[1];
      odometry->twist.twist.angular.z = angularVelocity[2];

      _odometryPub.publish(odometry);
    }

    ros::spinOnce();
    loopRate.sleep();
  }
}

void ViveTrackerNode::sendViveWorldFrame()
{
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.header.stamp = ros::Time::now();
  transform.child_frame_id = "vive_world";
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  _transformBroadcaster.sendTransform(transform);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vive_tracker");

  // We need an handle reference in addition to the tracker handle,
  // otherwise the ROS_LOG calls will not show anything after the ViveTrackerNode destruction.
  ros::NodeHandle mainHandle { "~" };

  try
  {
    ViveTrackerNode trackerNode { mainHandle };

    trackerNode.run();
  }
  catch (std::runtime_error &e)
  {
    ROS_ERROR_STREAM(e.what());
    return 1;
  }

  return 0;
}
