#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include "InterfaceHookInjector.h"

#include <cmath>

#include "../Version.h"

vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext* pDriverContext)
{
	TRACE("ServerTrackedDeviceProvider::Init()");
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

	OpenLogFile();
	LOG("OpenVR-SpaceCalibratorDriver " SPACECAL_VERSION_STRING " loaded");

	memset(transforms, 0, vr::k_unMaxTrackedDeviceCount * sizeof DeviceTransform);
	memset(slamSync, 0, sizeof slamSync);

	InjectHooks(pDriverContext);
	server.Run();

	return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::Cleanup()
{
	LOG("OpenVR-SpaceCalibratorDriver unloaded");
	CloseLogFile();

	TRACE("ServerTrackedDeviceProvider::Cleanup()");
	server.Stop();
	DisableHooks();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs) {
	return {
		(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
		(lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
		(lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
		(lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
	};
}

inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double(&vector)[3]) {
	vr::HmdQuaternion_t vectorQuat = { 0.0, vector[0], vector[1] , vector[2] };
	vr::HmdQuaternion_t conjugate = { quat.w, -quat.x, -quat.y, -quat.z };
	auto rotatedVectorQuat = quat * vectorQuat * conjugate;
	return { rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z };
}

inline vr::HmdQuaternion_t quaternionNormalize(vr::HmdQuaternion_t q) {
	double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	if (n > 0.0) {
		q.w /= n; q.x /= n; q.y /= n; q.z /= n;
	}
	return q;
}

inline vr::HmdQuaternion_t quaternionConjugate(const vr::HmdQuaternion_t& q) {
	return { q.w, -q.x, -q.y, -q.z };
}

inline double quaternionAngleBetween(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b) {
	double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
	dot = fabs(dot);
	if (dot > 1.0) dot = 1.0;
	return 2.0 * acos(dot);
}

inline vr::HmdQuaternion_t quaternionProjectYaw(const vr::HmdQuaternion_t& q) {
	double n = std::sqrt(q.w * q.w + q.y * q.y);
	if (n < 1e-9)
		return { 1, 0, 0, 0 }; // 180 deg tilt
	return { q.w / n, 0.0, q.y / n, 0.0 };
}

inline vr::HmdQuaternion_t quaternionSlerp(const vr::HmdQuaternion_t& a, vr::HmdQuaternion_t b, double t) {
	double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
	if (dot < 0.0) {
		dot = -dot;
		b.w = -b.w; b.x = -b.x; b.y = -b.y; b.z = -b.z;
	}

	double wa, wb;
	if (dot > 0.9995) {
		wa = 1.0 - t;
		wb = t;
	}
	else {
		double theta = acos(dot);
		double s = sin(theta);
		wa = sin((1.0 - t) * theta) / s;
		wb = sin(t * theta) / s;
	}

	return quaternionNormalize({
		wa * a.w + wb * b.w,
		wa * a.x + wb * b.x,
		wa * a.y + wb * b.y,
		wa * a.z + wb * b.z
		});
}

template < class T >
inline vr::HmdQuaternion_t HmdQuaternion_FromMatrix(const T& matrix)
{
	vr::HmdQuaternion_t q{};

	q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;

	q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
	q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
	q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);

	return q;
}

void ServerTrackedDeviceProvider::SetDeviceTransform(const protocol::SetDeviceTransform& newTransform)
{
	auto& tf = transforms[newTransform.openVRID];
	tf.enabled = newTransform.enabled;

	if (newTransform.updateTranslation)
		tf.translation = newTransform.translation;

	if (newTransform.updateRotation)
		tf.rotation = newTransform.rotation;

	if (newTransform.updateScale)
		tf.scale = newTransform.scale;
}

void ServerTrackedDeviceProvider::SetHmdTracker(const protocol::SetHmdTracker& cmd)
{
	hmdTracker.enabled = cmd.enabled;
	hmdTracker.native = cmd.native;
	hmdTracker.slamFallback = cmd.slamFallback;
	hmdTracker.disableAngVel = cmd.disableAngVel;
	hmdTracker.predictionTime = cmd.predictionTime;
	hmdTracker.hmdID = cmd.hmdID;
	hmdTracker.trackerID = cmd.trackerID;
	hmdTracker.offsetRotation = cmd.offsetRotation;
	hmdTracker.offsetTranslation = cmd.offsetTranslation;
	hmdTracker.calibrationRotation = cmd.calibrationRotation;
	hmdTracker.calibrationTranslation = cmd.calibrationTranslation;

	if (!cmd.enabled)
	{
		drift.valid = false;
		drift.level = DriftCorrection::TINY;
		memset(slamSync, 0, sizeof slamSync);
	}
}

void ServerTrackedDeviceProvider::SetSlamSync(const protocol::SetSlamSync& cmd)
{
	if (cmd.openVRID < vr::k_unMaxTrackedDeviceCount)
		slamSync[cmd.openVRID] = cmd.enabled;
}

void ServerTrackedDeviceProvider::UpdateDrift(const vr::HmdQuaternion_t& correctedRotation, const double(&correctedPosition)[3],
	const vr::HmdQuaternion_t& rawRotation, const double(&rawPosition)[3])
{
	vr::HmdQuaternion_t instRot = quaternionProjectYaw(quaternionNormalize(correctedRotation * quaternionConjugate(rawRotation)));
	vr::HmdVector3d_t instRotatedRaw = quaternionRotateVector(instRot, rawPosition);
	vr::HmdVector3d_t instTrans = {
		correctedPosition[0] - instRotatedRaw.v[0],
		correctedPosition[1] - instRotatedRaw.v[1],
		correctedPosition[2] - instRotatedRaw.v[2]
	};

	LARGE_INTEGER now, freq;
	QueryPerformanceCounter(&now);
	QueryPerformanceFrequency(&freq);

	if (!drift.valid)
	{
		drift.rotation = instRot;
		drift.translation = instTrans;
		drift.level = DriftCorrection::TINY;
		drift.lastUpdate = now;
		drift.valid = true;
		return;
	}

	double dt = (now.QuadPart - drift.lastUpdate.QuadPart) / (double)freq.QuadPart;
	drift.lastUpdate = now;
	if (dt < 0.0) dt = 0.0;
	if (dt > 0.1) dt = 0.1;

	vr::HmdVector3d_t headCur = quaternionRotateVector(drift.rotation, rawPosition);
	double dx = headCur.v[0] + drift.translation.v[0] - correctedPosition[0];
	double dy = headCur.v[1] + drift.translation.v[1] - correctedPosition[1];
	double dz = headCur.v[2] + drift.translation.v[2] - correctedPosition[2];
	double transDelta = sqrt(dx * dx + dy * dy + dz * dz);
	double rotDelta = quaternionAngleBetween(drift.rotation, instRot);

	const double deg = 3.14159265358979323846 / 180.0;

	if (transDelta > 0.5 || rotDelta > 30.0 * deg)
	{
		drift.rotation = instRot;
		drift.translation = instTrans;
		drift.level = DriftCorrection::TINY;
		return;
	}

	int level;
	if (transDelta > 0.10 || rotDelta > 5.0 * deg)
		level = DriftCorrection::LARGE;
	else if (transDelta > 0.015 || rotDelta > 1.0 * deg)
		level = DriftCorrection::SMALL;
	else
		level = DriftCorrection::TINY;

	if (transDelta < 0.005 && rotDelta < 0.3 * deg)
		drift.level = DriftCorrection::TINY;
	else if (level > drift.level)
		drift.level = level;

	double rate;
	switch (drift.level)
	{
	case DriftCorrection::LARGE:
		rate = 2.0; break;
	case DriftCorrection::SMALL:
		rate = 0.4; break;
	default:
		rate = 0.05;
		break;
	}

	double alpha = dt * rate;
	if (alpha > 1.0)
		alpha = 1.0;
	if (alpha < 0.0 || isnan(alpha))
		alpha = 0.0;

	vr::HmdQuaternion_t newRot = quaternionSlerp(drift.rotation, instRot, alpha);

	double fx = (headCur.v[0] + drift.translation.v[0]) * (1.0 - alpha) + correctedPosition[0] * alpha;
	double fy = (headCur.v[1] + drift.translation.v[1]) * (1.0 - alpha) + correctedPosition[1] * alpha;
	double fz = (headCur.v[2] + drift.translation.v[2]) * (1.0 - alpha) + correctedPosition[2] * alpha;

	vr::HmdVector3d_t headNew = quaternionRotateVector(newRot, rawPosition);
	drift.rotation = newRot;
	drift.translation = { fx - headNew.v[0], fy - headNew.v[1], fz - headNew.v[2] };
}

void ServerTrackedDeviceProvider::ApplyDrift(vr::DriverPose_t& pose) const
{
	pose.qWorldFromDriverRotation = quaternionNormalize(drift.rotation * pose.qWorldFromDriverRotation);

	vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(drift.rotation, pose.vecWorldFromDriverTranslation);
	pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + drift.translation.v[0];
	pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + drift.translation.v[1];
	pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + drift.translation.v[2];
}

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t& pose)
{
	auto& tf = transforms[openVRID];
	if (tf.enabled && !hmdTracker.native)
	{
		pose.qWorldFromDriverRotation = tf.rotation * pose.qWorldFromDriverRotation;

		pose.vecPosition[0] *= tf.scale;
		pose.vecPosition[1] *= tf.scale;
		pose.vecPosition[2] *= tf.scale;

		vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(tf.rotation, pose.vecWorldFromDriverTranslation);
		pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + tf.translation.v[0];
		pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + tf.translation.v[1];
		pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + tf.translation.v[2];
	}

	if (hmdTracker.enabled)
	{
		if (openVRID == hmdTracker.hmdID)
		{
			bool rawValid = pose.poseIsValid && pose.deviceIsConnected && pose.result == vr::TrackingResult_Running_OK;
			vr::HmdQuaternion_t rawRotation = { 1, 0, 0, 0 };
			double rawPosition[3] = { 0, 0, 0 };
			if (rawValid)
			{
				rawRotation = quaternionNormalize(pose.qWorldFromDriverRotation * pose.qRotation * pose.qDriverFromHeadRotation);

				vr::HmdVector3d_t headLocal = quaternionRotateVector(pose.qRotation, pose.vecDriverFromHeadTranslation);
				double driverLocal[3] = {
					pose.vecPosition[0] + headLocal.v[0],
					pose.vecPosition[1] + headLocal.v[1],
					pose.vecPosition[2] + headLocal.v[2]
				};
				vr::HmdVector3d_t world = quaternionRotateVector(pose.qWorldFromDriverRotation, driverLocal);
				rawPosition[0] = world.v[0] + pose.vecWorldFromDriverTranslation[0];
				rawPosition[1] = world.v[1] + pose.vecWorldFromDriverTranslation[1];
				rawPosition[2] = world.v[2] + pose.vecWorldFromDriverTranslation[2];
			}

			vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(openVRID);

			vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
			vr::VRServerDriverHost()->GetRawTrackedDevicePoses((1.0 / vr::VRProperties()->GetFloatProperty(container, vr::Prop_DisplayFrequency_Float)) * hmdTracker.predictionTime, poses, vr::k_unMaxTrackedDeviceCount);

			const auto& tp = poses[hmdTracker.trackerID];
			if (tp.bPoseIsValid)
			{
				vr::HmdQuaternion_t trackerQuat = HmdQuaternion_FromMatrix(tp.mDeviceToAbsoluteTracking);

				vr::HmdQuaternion_t trackerRefRotation = quaternionNormalize(hmdTracker.calibrationRotation * trackerQuat);

				double trackerPos[3] = {
					tp.mDeviceToAbsoluteTracking.m[0][3],
					tp.mDeviceToAbsoluteTracking.m[1][3],
					tp.mDeviceToAbsoluteTracking.m[2][3]
				};

				vr::HmdVector3d_t trackerRefPosition = quaternionRotateVector(hmdTracker.calibrationRotation, trackerPos);

				trackerRefPosition.v[0] += hmdTracker.calibrationTranslation.v[0];
				trackerRefPosition.v[1] += hmdTracker.calibrationTranslation.v[1];
				trackerRefPosition.v[2] += hmdTracker.calibrationTranslation.v[2];

				vr::HmdQuaternion_t hmdRotation = quaternionNormalize(hmdTracker.native ? trackerQuat * hmdTracker.offsetRotation : trackerRefRotation * hmdTracker.offsetRotation);
				vr::HmdVector3d_t offset = quaternionRotateVector(hmdTracker.native ? trackerQuat : trackerRefRotation, hmdTracker.offsetTranslation.v);

				pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
				pose.vecWorldFromDriverTranslation[0] = 0;
				pose.vecWorldFromDriverTranslation[1] = 0;
				pose.vecWorldFromDriverTranslation[2] = 0;

				pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
				pose.vecDriverFromHeadTranslation[0] = 0;
				pose.vecDriverFromHeadTranslation[1] = 0;
				pose.vecDriverFromHeadTranslation[2] = 0;

				if (hmdTracker.native) {
					pose.qRotation = hmdRotation;
					pose.vecPosition[0] = trackerPos[0] + offset.v[0];
					pose.vecPosition[1] = trackerPos[1] + offset.v[1];
					pose.vecPosition[2] = trackerPos[2] + offset.v[2];
				}
				else {
					pose.qRotation = hmdRotation;
					pose.vecPosition[0] = trackerRefPosition.v[0] + offset.v[0];
					pose.vecPosition[1] = trackerRefPosition.v[1] + offset.v[1];
					pose.vecPosition[2] = trackerRefPosition.v[2] + offset.v[2];
				}

				double trackerVel[3] = {
					tp.vVelocity.v[0],
					tp.vVelocity.v[1],
					tp.vVelocity.v[2]
				};

				double trackerAngVel[3] = {
					tp.vAngularVelocity.v[0],
					tp.vAngularVelocity.v[1],
					tp.vAngularVelocity.v[2]
				};

				vr::HmdVector3d_t vel = quaternionRotateVector(hmdTracker.calibrationRotation, trackerVel);
				vr::HmdVector3d_t angVel = quaternionRotateVector(hmdTracker.calibrationRotation, trackerAngVel);

				if (hmdTracker.native) {
					vr::HmdVector3d_t tangential = {
						trackerAngVel[1] * offset.v[2] - trackerAngVel[2] * offset.v[1],
						trackerAngVel[2] * offset.v[0] - trackerAngVel[0] * offset.v[2],
						trackerAngVel[0] * offset.v[1] - trackerAngVel[1] * offset.v[0]
					};

					for (int i = 0; i < 3; i++)
					{
						pose.vecVelocity[i] = trackerVel[i] + tangential.v[i];
						if (!hmdTracker.disableAngVel)
							pose.vecAngularVelocity[i] = trackerAngVel[i];
					}
				}
				else {
					vr::HmdVector3d_t tangential = {
						angVel.v[1] * offset.v[2] - angVel.v[2] * offset.v[1],
						angVel.v[2] * offset.v[0] - angVel.v[0] * offset.v[2],
						angVel.v[0] * offset.v[1] - angVel.v[1] * offset.v[0]
					};

					for (int i = 0; i < 3; i++)
					{
						pose.vecVelocity[i] = vel.v[i] + tangential.v[i];
						if (!hmdTracker.disableAngVel)
							pose.vecAngularVelocity[i] = angVel.v[i];
					}
				}

				pose.poseIsValid = true;
				pose.deviceIsConnected = true;
				pose.result = vr::TrackingResult_Running_OK;
				pose.shouldApplyHeadModel = false;
				pose.poseTimeOffset = 0;

				if (rawValid)
				{
					double linSpeed = sqrt(
						trackerVel[0] * trackerVel[0] +
						trackerVel[1] * trackerVel[1] +
						trackerVel[2] * trackerVel[2]);

					double angSpeed = sqrt(
						trackerAngVel[0] * trackerAngVel[0] +
						trackerAngVel[1] * trackerAngVel[1] +
						trackerAngVel[2] * trackerAngVel[2]);

					const double maxLinSpeed = 1.0;
					const double maxAngSpeed = 1.5;

					if (!drift.valid || (linSpeed < maxLinSpeed && angSpeed < maxAngSpeed))
						UpdateDrift(pose.qRotation, pose.vecPosition, rawRotation, rawPosition);
				}
			}
			else {
				if (!hmdTracker.slamFallback) {
					if (hmdTracker.native) {
						pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
						pose.vecWorldFromDriverTranslation[0] = 0;
						pose.vecWorldFromDriverTranslation[1] = 0;
						pose.vecWorldFromDriverTranslation[2] = 0;
					}
					else {
						pose.qWorldFromDriverRotation = hmdTracker.calibrationRotation;
						pose.vecWorldFromDriverTranslation[0] = hmdTracker.calibrationTranslation.v[0];
						pose.vecWorldFromDriverTranslation[1] = hmdTracker.calibrationTranslation.v[1];
						pose.vecWorldFromDriverTranslation[2] = hmdTracker.calibrationTranslation.v[2];
					}

					pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
					pose.vecDriverFromHeadTranslation[0] = 0;
					pose.vecDriverFromHeadTranslation[1] = 0;
					pose.vecDriverFromHeadTranslation[2] = 0;

					pose.qRotation = { 1, 0, 0, 0 };
					pose.vecPosition[0] = 0;
					pose.vecPosition[1] = 0;
					pose.vecPosition[2] = 0;

					for (int i = 0; i < 3; i++)
					{
						pose.vecVelocity[i] = 0;
						pose.vecAngularVelocity[i] = 0;
					}

					pose.poseIsValid = false;
					pose.deviceIsConnected = true;
					pose.result = vr::TrackingResult_Running_OutOfRange;
					pose.shouldApplyHeadModel = false;
					pose.poseTimeOffset = 0;
				}
				else if (drift.valid) {
					ApplyDrift(pose);
				}
			}
		}
		else if (slamSync[openVRID] && drift.valid)
		{
			ApplyDrift(pose);
		}
	}

	return true;
}