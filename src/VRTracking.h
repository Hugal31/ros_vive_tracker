#pragma once

#include <array>

#include <openvr.h>

class VRTracking
{
public:
  /// @brief State of the tracking (ok, lost, soon rotation only, etc...)
  enum class TrackingState
  {
    Ok,
    Error,
  };

  VRTracking();
  ~VRTracking();

  void update();

  bool isConnected(vr::TrackedDeviceIndex_t index) { return _trackedPoses[index].bDeviceIsConnected; }
  bool isPoseValid(vr::TrackedDeviceIndex_t index) { return _trackedPoses[index].bPoseIsValid; }

  TrackingState getDevicePose(vr::TrackedDeviceIndex_t index, float matrix[3][4]) const;
  TrackingState getDeviceVelocity(vr::TrackedDeviceIndex_t index, float matrix[3]) const;
  TrackingState getDeviceAngularVelocity(vr::TrackedDeviceIndex_t index, float matrix[3]) const;
  vr::TrackedDeviceClass getDeviceClass(vr::TrackedDeviceIndex_t index) { return _vrSystem->GetTrackedDeviceClass(index); }

  std::string getDeviceName(vr::TrackedDeviceIndex_t index) { return getDeviceStringProperty(index, vr::Prop_TrackingSystemName_String); }
  std::string getDeviceSerial(vr::TrackedDeviceIndex_t index) { return getDeviceStringProperty(index, vr::Prop_SerialNumber_String); }

  VRTracking(const VRTracking &) = delete;
  VRTracking(VRTracking &&) = delete;
  VRTracking &operator=(const VRTracking &) = delete;
  VRTracking &operator=(VRTracking &&) = delete;

private:
  std::string getDeviceStringProperty(vr::TrackedDeviceIndex_t index, vr::TrackedDeviceProperty property);

private:
  vr::IVRSystem *_vrSystem;

  std::array<vr::TrackedDevicePose_t, vr::k_unMaxTrackedDeviceCount> _trackedPoses;
};
