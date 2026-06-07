#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include "InterfaceHookInjector.h"

#include <cmath>

vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext *pDriverContext)
{
	TRACE("ServerTrackedDeviceProvider::Init()");
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

	memset(transforms, 0, vr::k_unMaxTrackedDeviceCount * sizeof DeviceTransform);

	InjectHooks(this, pDriverContext);
	server.Run();

	return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::Cleanup()
{
	TRACE("ServerTrackedDeviceProvider::Cleanup()");
	server.Stop();
	DisableHooks();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t &lhs, const vr::HmdQuaternion_t &rhs) {
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

void ServerTrackedDeviceProvider::SetDeviceTransform(const protocol::SetDeviceTransform &newTransform)
{
	auto &tf = transforms[newTransform.openVRID];
	tf.enabled = newTransform.enabled;

	if (newTransform.updateTranslation)
		tf.translation = newTransform.translation;

	if (newTransform.updateRotation)
		tf.rotation = newTransform.rotation;

	if (newTransform.updateScale)
		tf.scale = newTransform.scale;
}

void ServerTrackedDeviceProvider::SetHmdTracker(const protocol::SetHmdTracker &cmd)
{
	hmdTracker.enabled = cmd.enabled;
	hmdTracker.native = cmd.native;
	hmdTracker.hmdID = cmd.hmdID;
	hmdTracker.trackerID = cmd.trackerID;
	hmdTracker.offsetRotation = cmd.offsetRotation;
	hmdTracker.offsetTranslation = cmd.offsetTranslation;
	hmdTracker.calibrationRotation = cmd.calibrationRotation;
	hmdTracker.calibrationTranslation = cmd.calibrationTranslation;
}

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose)
{
	auto &tf = transforms[openVRID];
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
			vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(openVRID);

			vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
			vr::VRServerDriverHost()->GetRawTrackedDevicePoses(1.0 / vr::VRProperties()->GetFloatProperty(container, vr::Prop_DisplayFrequency_Float), poses, vr::k_unMaxTrackedDeviceCount);

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
				
				vr::HmdVector3d_t vel    = quaternionRotateVector(hmdTracker.calibrationRotation, trackerVel);
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
						pose.vecAngularVelocity[i] = angVel.v[i];
					}
				}

				pose.poseIsValid = true;
				pose.deviceIsConnected = true;
				pose.result = vr::TrackingResult_Running_OK;
				pose.shouldApplyHeadModel = false;
				pose.poseTimeOffset = 0;
			}
			else {
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
		}
	}

	return true;
}

