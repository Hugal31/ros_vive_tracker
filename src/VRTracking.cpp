#include <cstring>
#include <stdexcept>
#include <string>

#include "VRTracking.h"

VRTracking::VRTracking()
{
  vr::EVRInitError  vrError = vr::VRInitError_None;

  _vrSystem = vr::VR_Init(&vrError, vr::VRApplication_Other);
  if (vrError != vr::VRInitError_None)
  {
    _vrSystem = nullptr;
    throw std::runtime_error(std::string("Could not init VR runtime: ") + vr::VR_GetVRInitErrorAsEnglishDescription(vrError));
  }
}

VRTracking::~VRTracking()
{
  if (_vrSystem)
    vr::VR_Shutdown();
}

void VRTracking::update()
{
  _vrSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0, _trackedPoses.data(), _trackedPoses.size());
}

VRTracking::TrackingState VRTracking::getDevicePose(vr::TrackedDeviceIndex_t index, float matrix[3][4]) const
{
  vr::TrackedDevicePose_t const& pose = _trackedPoses[index];

  switch (pose.eTrackingResult)
  {
    case vr::TrackingResult_Running_OK:
      break;

    default:
    return TrackingState::Error;
  }

  std::memcpy(matrix, pose.mDeviceToAbsoluteTracking.m, sizeof(float[3][4]));

  return TrackingState::Ok;
}

VRTracking::TrackingState VRTracking::getDeviceVelocity(vr::TrackedDeviceIndex_t index, float matrix[3]) const
{
  vr::TrackedDevicePose_t const& pose = _trackedPoses[index];

  switch (pose.eTrackingResult)
  {
    case vr::TrackingResult_Running_OK:
      break;

    default:
      return TrackingState::Error;
  }

  std::memcpy(matrix, &pose.vVelocity, sizeof(float[3]));

  return TrackingState::Ok;
}

VRTracking::TrackingState VRTracking::getDeviceAngularVelocity(vr::TrackedDeviceIndex_t index, float matrix[3]) const
{
  vr::TrackedDevicePose_t const& pose = _trackedPoses[index];

  switch (pose.eTrackingResult)
  {
    case vr::TrackingResult_Running_OK:
      break;

    default:
      return TrackingState::Error;
  }

  std::memcpy(matrix, &pose.vAngularVelocity, sizeof(float[3]));

  return TrackingState::Ok;
}

std::string VRTracking::getDeviceStringProperty(vr::TrackedDeviceIndex_t index, vr::TrackedDeviceProperty property)
{
  std::array<char, 256> value {};
  vr::TrackedPropertyError error = vr::TrackedProp_Success;

  size_t length = _vrSystem->GetStringTrackedDeviceProperty(index, property, value.data(), value.size(), &error);
  if (error == vr::TrackedProp_Success) {
    throw std::runtime_error(_vrSystem->GetPropErrorNameFromEnum(error));
  }

  return std::string { value.data(), length };
}
