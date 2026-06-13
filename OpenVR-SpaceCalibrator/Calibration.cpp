#define WIN32_LEAN_AND_MEAN

#include "stdafx.h"
#include "Calibration.h"
#include "Configuration.h"
#include "IPCClient.h"

#include <string>
#include <vector>
#include <iostream> 
#include <algorithm>
#include <cmath>

#include <Eigen/Dense>


static IPCClient Driver;
CalibrationContext CalCtx;

void InitCalibrator()
{
	Driver.Connect();
}

struct Pose
{
	Eigen::Matrix3d rot;
	Eigen::Vector3d trans;

	Pose() { }
	Pose(vr::HmdMatrix34_t hmdMatrix)
	{
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				rot(i,j) = hmdMatrix.m[i][j];
			}
		}
		trans = Eigen::Vector3d(hmdMatrix.m[0][3], hmdMatrix.m[1][3], hmdMatrix.m[2][3]);
	}
	Pose(double x, double y, double z) : trans(Eigen::Vector3d(x,y,z)) { }
};

struct Sample
{
	Pose ref, target;
	bool valid;
	Sample() : valid(false) { }
	Sample(Pose ref, Pose target) : valid(true), ref(ref), target(target) { }
};

struct DSample
{
	bool valid;
	Eigen::Vector3d ref, target;
};

bool StartsWith(const std::string &str, const std::string &prefix)
{
	if (str.length() < prefix.length())
		return false;

	return str.compare(0, prefix.length(), prefix) == 0;
}

bool EndsWith(const std::string &str, const std::string &suffix)
{
	if (str.length() < suffix.length())
		return false;

	return str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
}

Eigen::Vector3d AxisFromRotationMatrix3(Eigen::Matrix3d rot)
{
	return Eigen::Vector3d(rot(2,1) - rot(1,2), rot(0,2) - rot(2,0), rot(1,0) - rot(0,1));
}

double AngleFromRotationMatrix3(Eigen::Matrix3d rot)
{
	return acos((rot(0,0) + rot(1,1) + rot(2,2) - 1.0) / 2.0);
}

struct DetectionState
{
	std::vector<uint32_t> candidates;
	std::vector<std::vector<double>> candidateSpeeds;
	std::vector<double> hmdSpeeds;
	std::vector<Eigen::Matrix3d> prevRot; // [0] = HMD, [i+1] = candidates[i]
	bool havePrev = false;
	double prevTime = 0;

	void Clear()
	{
		candidates.clear();
		candidateSpeeds.clear();
		hmdSpeeds.clear();
		prevRot.clear();
		havePrev = false;
		prevTime = 0;
	}
};

static DetectionState Detection;

static std::string GetDeviceSerial(uint32_t id)
{
	char serial[vr::k_unMaxPropertyStringSize] = {};
	vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, serial, vr::k_unMaxPropertyStringSize);
	return std::string(serial);
}

static std::string GetDeviceTrackingSystem(uint32_t id)
{
	char system[vr::k_unMaxPropertyStringSize] = {};
	vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, system, vr::k_unMaxPropertyStringSize);
	return std::string(system);
}

static double AngularSpeedBetween(const Eigen::Matrix3d &cur, const Eigen::Matrix3d &prev, double dt)
{
	Eigen::Matrix3d delta = cur * prev.transpose();
	double c = (delta(0,0) + delta(1,1) + delta(2,2) - 1.0) / 2.0;
	if (c > 1.0) c = 1.0;
	if (c < -1.0) c = -1.0;
	return acos(c) / dt;
}

static double PearsonCorrelation(const std::vector<double> &a, const std::vector<double> &b)
{
	if (a.size() != b.size() || a.empty())
		return 0.0;

	double meanA = 0, meanB = 0;
	for (size_t i = 0; i < a.size(); i++) { meanA += a[i]; meanB += b[i]; }
	meanA /= a.size();
	meanB /= b.size();

	double cov = 0, varA = 0, varB = 0;
	for (size_t i = 0; i < a.size(); i++)
	{
		double da = a[i] - meanA, db = b[i] - meanB;
		cov += da * db;
		varA += da * da;
		varB += db * db;
	}

	if (varA < 1e-9 || varB < 1e-9)
		return 0.0;

	return cov / std::sqrt(varA * varB);
}

DSample DeltaRotationSamples(Sample s1, Sample s2)
{
	// Difference in rotation between samples.
	auto dref = s1.ref.rot * s2.ref.rot.transpose();
	auto dtarget = s1.target.rot * s2.target.rot.transpose();

	// When stuck together, the two tracked objects rotate as a pair,
	// therefore their axes of rotation must be equal between any given pair of samples.
	DSample ds;
	ds.ref = AxisFromRotationMatrix3(dref);
	ds.target = AxisFromRotationMatrix3(dtarget);

	// Reject samples that were too close to each other.
	auto refA = AngleFromRotationMatrix3(dref);
	auto targetA = AngleFromRotationMatrix3(dtarget);
	ds.valid = refA > 0.4 && targetA > 0.4 && ds.ref.norm() > 0.01 && ds.target.norm() > 0.01;

	ds.ref.normalize();
	ds.target.normalize();
	return ds;
}

Eigen::Vector3d CalibrateRotation(const std::vector<Sample> &samples)
{
	std::vector<DSample> deltas;

	for (size_t i = 0; i < samples.size(); i++)
	{
		for (size_t j = 0; j < i; j++)
		{
			auto delta = DeltaRotationSamples(samples[i], samples[j]);
			if (delta.valid)
				deltas.push_back(delta);
		}
	}
	char buf[256];
	snprintf(buf, sizeof buf, "Got %zd samples with %zd delta samples\n", samples.size(), deltas.size());
	CalCtx.Log(buf);

	// Kabsch algorithm

	Eigen::MatrixXd refPoints(deltas.size(), 3), targetPoints(deltas.size(), 3);
	Eigen::Vector3d refCentroid(0,0,0), targetCentroid(0,0,0);

	for (size_t i = 0; i < deltas.size(); i++)
	{
		refPoints.row(i) = deltas[i].ref;
		refCentroid += deltas[i].ref;

		targetPoints.row(i) = deltas[i].target;
		targetCentroid += deltas[i].target;
	}

	refCentroid /= (double) deltas.size();
	targetCentroid /= (double) deltas.size();

	for (size_t i = 0; i < deltas.size(); i++)
	{
		refPoints.row(i) -= refCentroid;
		targetPoints.row(i) -= targetCentroid;
	}

	auto crossCV = refPoints.transpose() * targetPoints;

	Eigen::BDCSVD<Eigen::MatrixXd> bdcsvd;
	auto svd = bdcsvd.compute(crossCV, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::Matrix3d i = Eigen::Matrix3d::Identity();
	if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0)
	{
		i(2,2) = -1;
	}

	Eigen::Matrix3d rot = svd.matrixV() * i * svd.matrixU().transpose();
	rot.transposeInPlace();

	Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;

	snprintf(buf, sizeof buf, "Calibrated rotation: yaw=%.2f pitch=%.2f roll=%.2f\n", euler[1], euler[2], euler[0]);
	CalCtx.Log(buf);
	return euler;
}

Eigen::Vector3d CalibrateTranslation(const std::vector<Sample> &samples, const Eigen::Matrix3d &rotation)
{
	std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> deltas;

	for (size_t i = 0; i < samples.size(); i++)
	{
		Sample s_i = samples[i];
		s_i.target.rot = rotation * s_i.target.rot;
		s_i.target.trans = rotation * s_i.target.trans;

		for (size_t j = 0; j < i; j++)
		{
			Sample s_j = samples[j];
			s_j.target.rot = rotation * s_j.target.rot;
			s_j.target.trans = rotation * s_j.target.trans;

			auto QAi = s_i.ref.rot.transpose();
			auto QAj = s_j.ref.rot.transpose();
			auto dQA = QAj - QAi;
			auto CA = QAj * (s_j.ref.trans - s_j.target.trans) - QAi * (s_i.ref.trans - s_i.target.trans);
			deltas.push_back(std::make_pair(CA, dQA));

			auto QBi = s_i.target.rot.transpose();
			auto QBj = s_j.target.rot.transpose();
			auto dQB = QBj - QBi;
			auto CB = QBj * (s_j.ref.trans - s_j.target.trans) - QBi * (s_i.ref.trans - s_i.target.trans);
			deltas.push_back(std::make_pair(CB, dQB));
		}
	}

	Eigen::VectorXd constants(deltas.size() * 3);
	Eigen::MatrixXd coefficients(deltas.size() * 3, 3);

	for (size_t i = 0; i < deltas.size(); i++)
	{
		for (int axis = 0; axis < 3; axis++)
		{
			constants(i * 3 + axis) = deltas[i].first(axis);
			coefficients.row(i * 3 + axis) = deltas[i].second.row(axis);
		}
	}

	Eigen::Vector3d trans = coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(constants);
	auto transcm = trans * 100.0;

	char buf[256];
	snprintf(buf, sizeof buf, "Calibrated translation x=%.2f y=%.2f z=%.2f\n", transcm[0], transcm[1], transcm[2]);
	CalCtx.Log(buf);
	return transcm;
}

static const double AxisVarianceThreshold = 0.001;

static double SecondAxisVariance(const std::vector<Sample> &samples)
{
	std::vector<Eigen::Vector4d> points;
	points.reserve(samples.size());
	Eigen::Vector4d mean = Eigen::Vector4d::Zero();

	for (auto &sample : samples)
	{
		Eigen::Quaterniond q(sample.target.rot);
		if (q.w() < 0)
			q.coeffs() = -q.coeffs();

		Eigen::Vector4d point(q.w(), q.x(), q.y(), q.z());
		mean += point;
		points.push_back(point);
	}

	if (points.empty())
		return 0.0;

	mean /= (double)points.size();

	Eigen::Matrix4d cov = Eigen::Matrix4d::Zero();
	for (auto &point : points)
	{
		Eigen::Vector4d d = point - mean;
		cov += d * d.transpose();
	}
	cov /= (double)points.size();

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(cov);
	return solver.eigenvalues()(1);
}

static Eigen::Vector3d ComputeRefToTargetOffset(const std::vector<Sample> &samples, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans)
{
	Eigen::Vector3d accum = Eigen::Vector3d::Zero();

	for (auto &sample : samples)
		accum += sample.ref.rot.transpose() * (calRot * sample.target.trans + calTrans - sample.ref.trans);

	return accum / (double)samples.size();
}

static double RetargetingErrorRMS(const std::vector<Sample> &samples, const Eigen::Vector3d &hmdToTargetPos, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans)
{
	double accum = 0;

	for (auto &sample : samples)
		accum += (calRot * sample.target.trans + calTrans - (sample.ref.rot * hmdToTargetPos + sample.ref.trans)).squaredNorm();

	return std::sqrt(accum / (double)samples.size());
}

Sample CollectSample(const CalibrationContext &ctx)
{
	vr::TrackedDevicePose_t reference, target;
	reference.bPoseIsValid = false;
	target.bPoseIsValid = false;

	reference = ctx.devicePoses[0];
	target = ctx.devicePoses[ctx.targetID];

	bool ok = true;
	if (!reference.bPoseIsValid)
	{
		CalCtx.Log("Reference device is not tracking\n"); ok = false;
	}
	if (!target.bPoseIsValid)
	{
		CalCtx.Log("Target device is not tracking\n"); ok = false;
	}
	if (!ok)
	{
		CalCtx.Log("Aborting calibration!\n");
		CalCtx.state = CalibrationState::None;
		return Sample();
	}

	return Sample(
		Pose(reference.mDeviceToAbsoluteTracking),
		Pose(target.mDeviceToAbsoluteTracking)
	);
}

vr::HmdQuaternion_t VRRotationQuat(Eigen::Vector3d eulerdeg)
{
	auto euler = eulerdeg * EIGEN_PI / 180.0;

	Eigen::Quaterniond rotQuat =
		Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());

	vr::HmdQuaternion_t vrRotQuat;
	vrRotQuat.x = rotQuat.coeffs()[0];
	vrRotQuat.y = rotQuat.coeffs()[1];
	vrRotQuat.z = rotQuat.coeffs()[2];
	vrRotQuat.w = rotQuat.coeffs()[3];
	return vrRotQuat;
}

vr::HmdVector3d_t VRTranslationVec(Eigen::Vector3d transcm)
{
	auto trans = transcm * 0.01;
	vr::HmdVector3d_t vrTrans;
	vrTrans.v[0] = trans[0];
	vrTrans.v[1] = trans[1];
	vrTrans.v[2] = trans[2];
	return vrTrans;
}

void ResetAndDisableOffsets(uint32_t id)
{
	vr::HmdVector3d_t zeroV;
	zeroV.v[0] = zeroV.v[1] = zeroV.v[2] = 0;

	vr::HmdQuaternion_t zeroQ;
	zeroQ.x = 0; zeroQ.y = 0; zeroQ.z = 0; zeroQ.w = 1;

	protocol::Request req(protocol::RequestSetDeviceTransform);
	req.setDeviceTransform = { id, false, zeroV, zeroQ, 1.0 };
	Driver.SendBlocking(req);
}

void SendHmdTrackerCommand(uint32_t hmdID, uint32_t trackerID, bool enabled)
{
	protocol::Request req(protocol::RequestSetHmdTracker);
	req.setHmdTracker.hmdID = hmdID;
	req.setHmdTracker.trackerID = trackerID;
	req.setHmdTracker.enabled = enabled;
	req.setHmdTracker.native = CalCtx.enableNative;
	req.setHmdTracker.slamFallback = CalCtx.fallbackToSlam;
	req.setHmdTracker.predictionTime = CalCtx.predictionTime;
	req.setHmdTracker.disableAngVel = CalCtx.disableAngularVelocity;
	req.setHmdTracker.offsetRotation = CalCtx.relativeRotation;
	req.setHmdTracker.offsetTranslation = CalCtx.relativeTranslation;
	req.setHmdTracker.calibrationRotation = VRRotationQuat(CalCtx.calibratedRotation);
	req.setHmdTracker.calibrationTranslation = VRTranslationVec(CalCtx.calibratedTranslation);
	Driver.SendBlocking(req);
}

// https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions/27410865
void ComputeRelativeOffset(CalibrationContext &ctx, const std::vector<Sample> &samples, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans)
{
	if (samples.empty())
		return;

	Eigen::Matrix4d quatAccum = Eigen::Matrix4d::Zero();
	Eigen::Vector3d transAccum = Eigen::Vector3d::Zero();

	for (auto &sample : samples)
	{
		Eigen::Matrix3d trackerRot = calRot * sample.target.rot;
		Eigen::Vector3d trackerTrans = calRot * sample.target.trans + calTrans;

		Eigen::Matrix3d offsetRot = trackerRot.transpose() * sample.ref.rot;
		Eigen::Vector3d offsetTrans = trackerRot.transpose() * (sample.ref.trans - trackerTrans);

		Eigen::Quaterniond q(offsetRot);
		Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());
		quatAccum += v * v.transpose();
		transAccum += offsetTrans;
	}

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(quatAccum);
	Eigen::Vector4d avg = solver.eigenvectors().col(3).normalized();

	Eigen::Quaterniond q(avg(0), avg(1), avg(2), avg(3));
	q.normalize();
	if (q.w() < 0)
		q.coeffs() = -q.coeffs();

	transAccum /= (double)samples.size();

	ctx.relativeRotation.w = q.w();
	ctx.relativeRotation.x = q.x();
	ctx.relativeRotation.y = q.y();
	ctx.relativeRotation.z = q.z();
	ctx.relativeTranslation.v[0] = transAccum.x();
	ctx.relativeTranslation.v[1] = transAccum.y();
	ctx.relativeTranslation.v[2] = transAccum.z();
	ctx.validRelativeOffset = true;
}

static_assert(vr::k_unTrackedDeviceIndex_Hmd == 0, "HMD index expected to be 0");

void ScanAndApplyProfile(CalibrationContext &ctx)
{
	char buffer[vr::k_unMaxPropertyStringSize];
	ctx.enabled = ctx.validProfile;

	if (ctx.enabled)
	{
		ctx.targetID = vr::k_unTrackedDeviceIndexInvalid;
		if (!ctx.trackerSerial.empty())
		{
			for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
			{
				if (vr::VRSystem()->GetTrackedDeviceClass(id) == vr::TrackedDeviceClass_Invalid)
					continue;
				if (GetDeviceSerial(id) == ctx.trackerSerial)
				{
					ctx.targetID = id;
					break;
				}
			}
		}
	}

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;

		// The headset is driven from the tracker, so every device keeps its raw pose;
		// clear any space-warp offset that an older profile may have applied.
		ResetAndDisableOffsets(id);

		if (!ctx.enabled)
			continue;

		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);

		if (err != vr::TrackedProp_Success)
			continue;

		std::string trackingSystem(buffer);

		if (id == vr::k_unTrackedDeviceIndex_Hmd)
			continue;

		if (deviceClass == vr::TrackedDeviceClass_GenericTracker && trackingSystem == ctx.targetTrackingSystem && id == ctx.targetID)
		{
			continue;
		}

		if (trackingSystem == ctx.targetTrackingSystem) {
			protocol::Request req(protocol::RequestSetDeviceTransform);
			req.setDeviceTransform = {
				id,
				true,
				VRTranslationVec(ctx.calibratedTranslation),
				VRRotationQuat(ctx.calibratedRotation),
				ctx.calibratedScale
			};
			Driver.SendBlocking(req);
		}
	}

	bool overrideActive = ctx.enabled && ctx.validRelativeOffset && ctx.targetID != vr::k_unTrackedDeviceIndexInvalid;

	for (uint32_t id = 0; overrideActive && id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;

		bool sync = ctx.continuousSync
			&& id != vr::k_unTrackedDeviceIndex_Hmd
			&& deviceClass != vr::TrackedDeviceClass_TrackingReference;

		if (sync)
		{
			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);
			sync = err == vr::TrackedProp_Success && std::string(buffer) != ctx.targetTrackingSystem;
		}

		protocol::Request req(protocol::RequestSetSlamSync);
		req.setSlamSync = { id, sync };
		Driver.SendBlocking(req);
	}

	if (overrideActive)
	{
		SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, ctx.targetID, true);
	}
	else
	{
		SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);
	}

	if (ctx.enabled && ctx.chaperone.valid && ctx.chaperone.autoApply)
	{
		uint32_t quadCount = 0;
		vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

		// Heuristic: when SteamVR resets to a blank-ish chaperone, it uses empty geometry,
		// but manual adjustments (e.g. via a play space mover) will not touch geometry.
		if (quadCount != ctx.chaperone.geometry.size())
		{
			ApplyChaperoneBounds();
		}
	}
}

static void BeginSamplingPhase(CalibrationContext &ctx, uint32_t targetID)
{
	ctx.targetID = targetID;
	ctx.targetTrackingSystem = GetDeviceTrackingSystem(targetID);
	ctx.hmdSerial = GetDeviceSerial(vr::k_unTrackedDeviceIndex_Hmd);
	ctx.trackerSerial = GetDeviceSerial(targetID);

	char buf[256];
	snprintf(buf, sizeof buf, "Using headset tracker: %s (id %d)\n", ctx.trackerSerial.c_str(), targetID);
	ctx.Log(buf);

	ResetAndDisableOffsets(targetID);
	SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);

	ctx.state = CalibrationState::Sampling;
	ctx.wantedUpdateInterval = 0.0;
	ctx.Log("Starting calibration...\n");
}

static std::vector<Sample> collectedSamples;
static int coplanarRetries = 0;

void StartCalibration()
{
	CalCtx.state = CalibrationState::Begin;
	CalCtx.wantedUpdateInterval = 0.0;
	CalCtx.messages.clear();
	Detection.Clear();
	collectedSamples.clear();
	coplanarRetries = 0;
}

static void AbortAndRestoreProfile(CalibrationContext &ctx)
{
	if (ctx.targetID != vr::k_unTrackedDeviceIndexInvalid)
		ResetAndDisableOffsets(ctx.targetID);

	LoadProfile(ctx);
	ctx.state = CalibrationState::None;
	collectedSamples.clear();
	coplanarRetries = 0;
}

void CalibrationTick(double time)
{
	if (!vr::VRSystem())
		return;

	auto &ctx = CalCtx;
	if ((time - ctx.timeLastTick) < 0.05)
		return;

	ctx.timeLastTick = time;
	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.0f, ctx.devicePoses, vr::k_unMaxTrackedDeviceCount);

	if (ctx.state == CalibrationState::None)
	{
		ctx.wantedUpdateInterval = 1.0;

		if ((time - ctx.timeLastScan) >= 1.0)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Editing)
	{
		ctx.wantedUpdateInterval = 0.1;

		if ((time - ctx.timeLastScan) >= 0.1)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Begin)
	{
		SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);

		if (vr::VRSystem()->GetTrackedDeviceClass(vr::k_unTrackedDeviceIndex_Hmd) != vr::TrackedDeviceClass_HMD ||
			!ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
		{
			ctx.state = CalibrationState::None;
			CalCtx.Log("No tracking HMD found, aborting calibration!\n");
			return;
		}

		std::string hmdSystem = GetDeviceTrackingSystem(vr::k_unTrackedDeviceIndex_Hmd);

		Detection.Clear();
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
		{
			if (vr::VRSystem()->GetTrackedDeviceClass(id) != vr::TrackedDeviceClass_GenericTracker)
				continue;
			if (!ctx.devicePoses[id].bPoseIsValid)
				continue;

			Detection.candidates.push_back(id);
		}

		if (Detection.candidates.empty())
		{
			ctx.state = CalibrationState::None;
			CalCtx.Log("No trackers from a different tracking system detected, aborting!\n");
			return;
		}

		if (Detection.candidates.size() == 1)
		{
			ctx.targetID = Detection.candidates[0];
			BeginSamplingPhase(ctx, Detection.candidates[0]);
			return;
		}

		Detection.candidateSpeeds.resize(Detection.candidates.size());
		CalCtx.Log("Move your head around to identify the headset tracker...\n");
		ctx.state = CalibrationState::Detect;
		ctx.wantedUpdateInterval = 0.0;
		return;
	}

	if (ctx.state == CalibrationState::Detect)
	{
		if (!ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
			return;

		Eigen::Matrix3d hmdRot = Pose(ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking).rot;

		std::vector<Eigen::Matrix3d> curRot(Detection.candidates.size());
		for (size_t i = 0; i < Detection.candidates.size(); i++)
			curRot[i] = Pose(ctx.devicePoses[Detection.candidates[i]].mDeviceToAbsoluteTracking).rot;

		double dt = time - Detection.prevTime;
		if (Detection.havePrev && dt > 1e-4)
		{
			Detection.hmdSpeeds.push_back(AngularSpeedBetween(hmdRot, Detection.prevRot[0], dt));
			for (size_t i = 0; i < Detection.candidates.size(); i++)
				Detection.candidateSpeeds[i].push_back(AngularSpeedBetween(curRot[i], Detection.prevRot[i + 1], dt));

			CalCtx.Progress((int) Detection.hmdSpeeds.size(), 40);
		}

		Detection.prevRot.assign(1, hmdRot);
		Detection.prevRot.insert(Detection.prevRot.end(), curRot.begin(), curRot.end());
		Detection.prevTime = time;
		Detection.havePrev = true;

		if ((int) Detection.hmdSpeeds.size() < 40)
			return;

		double hmdPeak = 0;
		for (double s : Detection.hmdSpeeds)
			hmdPeak = max(hmdPeak, s);

		if (hmdPeak < 0.5)
		{
			Detection.Clear();
			ctx.state = CalibrationState::None;
			CalCtx.Log("Didn't detect enough head movement, aborting! Try again and move your head more.\n");
			return;
		}

		double bestCorr = -2, secondCorr = -2;
		int bestIdx = -1;
		for (size_t i = 0; i < Detection.candidates.size(); i++)
		{
			double corr = PearsonCorrelation(Detection.hmdSpeeds, Detection.candidateSpeeds[i]);
			if (corr > bestCorr)
			{
				secondCorr = bestCorr;
				bestCorr = corr;
				bestIdx = (int) i;
			}
			else if (corr > secondCorr)
			{
				secondCorr = corr;
			}
		}

		if (bestIdx == -1 || bestCorr < 0.7 || (bestCorr - secondCorr) < 0.1)
		{
			Detection.Clear();
			ctx.state = CalibrationState::None;
			CalCtx.Log("Couldn't clearly identify the headset tracker, aborting! Make sure only the headset tracker moves with your head, then try again.\n");
			return;
		}

		uint32_t targetID = Detection.candidates[bestIdx];
		Detection.Clear();
		BeginSamplingPhase(ctx, targetID);
		return;
	}

	auto sample = CollectSample(ctx);
	if (!sample.valid)
	{
		return;
	}

	auto &samples = collectedSamples;
	samples.push_back(sample);

	CalCtx.Progress(samples.size(), CalCtx.SampleCount());

	if (samples.size() >= CalCtx.SampleCount())
	{
		CalCtx.Log("\n");

		double axisVariance = SecondAxisVariance(samples);
		if (axisVariance < AxisVarianceThreshold)
		{
			if (++coplanarRetries >= 10)
			{
				CalCtx.Log("Not enough rotation variety after several attempts, aborting calibration! Previous calibration restored.\n");
				AbortAndRestoreProfile(ctx);
				return;
			}

			char buf[256];
			snprintf(buf, sizeof buf, "Head movement is too uniform (axis variance %.5f), tilt and turn your head in different directions! Collecting more samples...\n", axisVariance);
			CalCtx.Log(buf);
			samples.erase(samples.begin(), samples.begin() + samples.size() / 4);
			return;
		}
		coplanarRetries = 0;

		ctx.calibratedRotation = CalibrateRotation(samples);

		Eigen::Vector3d eulerRad = ctx.calibratedRotation * EIGEN_PI / 180.0;
		Eigen::Matrix3d calRot =
			(Eigen::AngleAxisd(eulerRad(0), Eigen::Vector3d::UnitZ()) *
			 Eigen::AngleAxisd(eulerRad(1), Eigen::Vector3d::UnitY()) *
			 Eigen::AngleAxisd(eulerRad(2), Eigen::Vector3d::UnitX())).toRotationMatrix();

		ctx.calibratedTranslation = CalibrateTranslation(samples, calRot);
		Eigen::Vector3d calTransM = ctx.calibratedTranslation * 0.01;

		Eigen::Vector3d hmdToTarget = ComputeRefToTargetOffset(samples, calRot, calTransM);
		double rmsError = RetargetingErrorRMS(samples, hmdToTarget, calRot, calTransM);

		char buf[256];
		snprintf(buf, sizeof buf, "Calibration residual error (RMS): %.1f mm\n", rmsError * 1000.0);
		CalCtx.Log(buf);

		if (rmsError > 0.1)
		{
			CalCtx.Log("Calibration quality is too low, aborting! Previous calibration restored. Try again with a slower calibration speed, moving smoothly.\n");
			AbortAndRestoreProfile(ctx);
			return;
		}

		ComputeRelativeOffset(ctx, samples, calRot, calTransM);

		ctx.validProfile = true;
		SaveProfile(ctx);
		CalCtx.Log("Finished calibration, profile saved\n");

		ctx.state = CalibrationState::None;
		samples.clear();
	}
}

void LoadChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();

	uint32_t quadCount = 0;
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

	CalCtx.chaperone.geometry.resize(quadCount);
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], &quadCount);
	vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->GetWorkingPlayAreaSize(&CalCtx.chaperone.playSpaceSize.v[0], &CalCtx.chaperone.playSpaceSize.v[1]);
	CalCtx.chaperone.valid = true;
}

void ApplyChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();
	vr::VRChaperoneSetup()->SetWorkingCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], CalCtx.chaperone.geometry.size());
	vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->SetWorkingPlayAreaSize(CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1]);
	vr::VRChaperoneSetup()->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);
}
