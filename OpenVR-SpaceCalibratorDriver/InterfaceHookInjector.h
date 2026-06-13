#pragma once

#include <openvr_driver.h>

static void DetourTrackedDevicePoseUpdated(void* _this, uint32_t unWhichDevice, const vr::DriverPose_t & newPose, uint32_t unPoseStructSize);

void InjectHooks(vr::IVRDriverContext *pDriverContext);
void DisableHooks();