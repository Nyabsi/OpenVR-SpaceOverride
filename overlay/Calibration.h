#pragma once

#include <Eigen/Core>
#include <openvr.h>
#include <vector>

enum class CalibrationState
{
	None,
	Begin,
	Detect,
	Rotation,
	Translation,
	Editing,
};

struct CalibrationContext
{
	CalibrationState state = CalibrationState::None;
	uint32_t targetID;

	Eigen::Vector3d calibratedRotation;
	Eigen::Vector3d calibratedTranslation;
	double calibratedScale;

	vr::HmdQuaternion_t relativeRotation = { 1, 0, 0, 0 };
	vr::HmdVector3d_t relativeTranslation = { 0, 0, 0 };
	bool validRelativeOffset = false;

	vr::HmdMatrix34_t offsetRefRaw = {};
	vr::HmdMatrix34_t offsetTargetRaw = {};
	bool haveOffsetSample = false;

	std::string targetTrackingSystem;

	std::string hmdSerial;
	std::string trackerSerial;

	bool enabled = false;
	bool validProfile = false;
	double timeLastTick = 0, timeLastScan = 0;
	double wantedUpdateInterval = 1.0;

	bool enableNative = false;
	bool fallbackToSlam = true;
	bool disableAngularVelocity = false;

	enum Speed
	{
		FAST = 0,
		SLOW = 1,
		VERY_SLOW = 2
	};
	Speed calibrationSpeed = FAST;

	vr::TrackedDevicePose_t devicePoses[vr::k_unMaxTrackedDeviceCount];

	struct Chaperone
	{
		bool valid = false;
		bool autoApply = true;
		std::vector<vr::HmdQuad_t> geometry;
		vr::HmdMatrix34_t standingCenter;
		vr::HmdVector2_t playSpaceSize;
	} chaperone;

	void Clear()
	{
		chaperone.geometry.clear();
		chaperone.standingCenter = vr::HmdMatrix34_t();
		chaperone.playSpaceSize = vr::HmdVector2_t();
		chaperone.valid = false;

		calibratedRotation = Eigen::Vector3d();
		calibratedTranslation = Eigen::Vector3d();
		calibratedScale = 1.0;
		relativeRotation = { 1, 0, 0, 0 };
		relativeTranslation = { 0, 0, 0 };
		validRelativeOffset = false;
		haveOffsetSample = false;
		targetTrackingSystem = "";
		hmdSerial = "";
		trackerSerial = "";
		enabled = false;
		validProfile = false;
	}

	size_t SampleCount()
	{
		switch (calibrationSpeed)
		{
		case FAST:
			return 100;
		case SLOW:
			return 250;
		case VERY_SLOW:
			return 500;
		}
		return 100;
	}

	struct Message
	{
		enum Type
		{
			String,
			Progress
		} type = String;

		Message(Type type) : type(type) { }

		std::string str;
		int progress, target;
	};

	std::vector<Message> messages;

	void Log(const std::string &msg)
	{
		if (messages.empty() || messages.back().type == Message::Progress)
			messages.push_back(Message(Message::String));

		messages.back().str += msg;
		std::cerr << msg;
	}

	void Progress(int current, int target)
	{
		if (messages.empty() || messages.back().type == Message::String)
			messages.push_back(Message(Message::Progress));

		messages.back().progress = current;
		messages.back().target = target;
	}
};

extern CalibrationContext CalCtx;

void InitCalibrator();
void CalibrationTick(double time);
void StartCalibration();
void LoadChaperoneBounds();
void ApplyChaperoneBounds();