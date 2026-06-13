#pragma once

#include "IPCServer.h"

#include <openvr_driver.h>
#include <chrono>

class ServerTrackedDeviceProvider : public vr::IServerTrackedDeviceProvider
{
public:
	////// Start vr::IServerTrackedDeviceProvider functions

	/** initializes the driver. This will be called before any other methods are called. */
	virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override;

	/** cleans up the driver right before it is unloaded */
	virtual void Cleanup() override;

	/** Returns the version of the ITrackedDeviceServerDriver interface used by this driver */
	virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }

	/** Allows the driver do to some work in the main loop of the server. */
	virtual void RunFrame() { }

	/** Returns true if the driver wants to block Standby mode. */
	virtual bool ShouldBlockStandbyMode() { return false; }

	/** Called when the system is entering Standby mode. The driver should switch itself into whatever sort of low-power
	* state it has. */
	virtual void EnterStandby() { }

	/** Called when the system is leaving Standby mode. The driver should switch itself back to
	full operation. */
	virtual void LeaveStandby() { }

	////// End vr::IServerTrackedDeviceProvider functions

	ServerTrackedDeviceProvider() : server(this) { }
	void SetDeviceTransform(const protocol::SetDeviceTransform &newTransform);
	void SetHmdTracker(const protocol::SetHmdTracker &cmd);
	void SetSlamSync(const protocol::SetSlamSync &cmd);
	bool HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose);

private:
	void UpdateDrift(const vr::HmdQuaternion_t &correctedRotation, const double (&correctedPosition)[3],
		const vr::HmdQuaternion_t &rawRotation, const double (&rawPosition)[3]);
	void ApplyDrift(vr::DriverPose_t &pose) const;

	IPCServer server;

	struct DeviceTransform
	{
		bool enabled = false;
		vr::HmdVector3d_t translation;
		vr::HmdQuaternion_t rotation;
		double scale;
	};

	DeviceTransform transforms[vr::k_unMaxTrackedDeviceCount];

	struct HmdTracker
	{
		bool enabled = false;
		bool native = false;
		bool slamFallback = true;
		bool disableAngVel = false;
		float predictionTime = 1.0f;
		uint32_t hmdID = vr::k_unTrackedDeviceIndex_Hmd;
		uint32_t trackerID = vr::k_unTrackedDeviceIndexInvalid;
		vr::HmdQuaternion_t offsetRotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t offsetTranslation = { 0, 0, 0 };
		vr::HmdQuaternion_t calibrationRotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t calibrationTranslation = { 0, 0, 0 };
	} hmdTracker;

	bool slamSync[vr::k_unMaxTrackedDeviceCount];

	struct DriftCorrection
	{
		enum Level { TINY = 0, SMALL = 1, LARGE = 2 };

		bool valid = false;
		int level = TINY;
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t translation = { 0, 0, 0 };
		LARGE_INTEGER lastUpdate = {};
	} drift;
};