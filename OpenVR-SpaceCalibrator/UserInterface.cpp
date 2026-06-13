#include "stdafx.h"
#include "UserInterface.h"
#include "Calibration.h"
#include "Configuration.h"
#include "../Version.h"

#include <string>
#include <vector>
#include <algorithm>
#include <imgui/imgui.h>

struct VRDevice
{
	int id = -1;
	vr::TrackedDeviceClass deviceClass;
	std::string model = "";
	std::string serial = "";
	std::string trackingSystem = "";
	vr::ETrackedControllerRole controllerRole = vr::TrackedControllerRole_Invalid;
};

struct VRState
{
	std::vector<std::string> trackingSystems;
	std::vector<VRDevice> devices;
};

void TextWithWidth(const char *label, const char *text, float width);

VRState LoadVRState();
void BuildStatus(const VRState &state);
void BuildProfileEditor();
void BuildMenu(bool runningInOverlay);

static const ImGuiWindowFlags bareWindowFlags =
	ImGuiWindowFlags_NoTitleBar |
	ImGuiWindowFlags_NoResize |
	ImGuiWindowFlags_NoMove |
	ImGuiWindowFlags_NoScrollbar |
	ImGuiWindowFlags_NoScrollWithMouse;

void BuildMainWindow(bool runningInOverlay)
{
	auto &io = ImGui::GetIO();

	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
	ImGui::SetNextWindowSize(io.DisplaySize);

	if (!ImGui::Begin("MainWindow", nullptr, bareWindowFlags))
	{
		ImGui::End();
		return;
	}

	ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImGui::GetStyleColorVec4(ImGuiCol_Button));

	if (ImGui::BeginTabBar("##tabs")) {

		if (ImGui::BeginTabItem("Calibration")) {

			auto state = LoadVRState();
			BuildStatus(state);
			BuildMenu(runningInOverlay);
			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Settings")) {
			ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "NOTE: All settings below require re-calibration to be applied");

			ImGui::Checkbox("Native Override (Only For Expert Users)", &CalCtx.enableNative);
			ImGui::SetItemTooltip(
				"Discards all SLAM tracking (even as fallback) and feeds only raw tracker data with the offset applied.\n"
				"Downside: a yaw orientation mismatch may occur. This only works in Local tracking space, where\n"
				"re-centering adjusts the yaw - Stage tracking space never re-centers yaw, so the mismatch cannot\n"
				"be corrected there.\n"
				"Will not work on all devices - tested only on Pico.");

			ImGui::Checkbox("Fallback to SLAM", &CalCtx.fallbackToSlam);
			ImGui::SetItemTooltip(
				"Temporarily uses HMD (SLAM) tracking if the headset tracker loses line of sight.");

			ImGui::Checkbox("Disable Angular Velocity", &CalCtx.disableAngularVelocity);
			ImGui::SetItemTooltip(
				"Disables angular velocity reporting, as it may cause issues with some devices.");

			ImGui::Checkbox("Continuous Space Sync", &CalCtx.continuousSync);
			ImGui::SetItemTooltip(
				"Continuously re-aligns SLAM-tracked devices (controllers etc.) to the calibrated space\n"
				"in the background. The headset's raw SLAM pose is compared against the tracker-driven\n"
				"pose to measure SLAM drift, and the correction is applied gradually and automatically.\n"
				"Only SLAM devices are affected - the HMD and lighthouse devices are never touched.");

			ImGui::Text("Prediction Time");
			ImGui::SameLine();
			ImGui::SliderFloat("##prediction_time", &CalCtx.predictionTime, 0.0f, 10.0f, "%.1f");

			ImGui::SetItemTooltip(
				"How many frames of prediction SteamVR applies to the tracker.\n"
				"Some wireless solutions may need more prediction to feel smooth.");

			ImGui::EndTabItem();
		}


		ImGui::EndTabBar();
	}

	ImGui::PopStyleColor();
	ImGui::End();
}

void BuildMenu(bool runningInOverlay)
{
	auto &io = ImGui::GetIO();
	ImGuiStyle &style = ImGui::GetStyle();
	ImGui::Text("");

	if (CalCtx.state == CalibrationState::None)
	{
		float buttonWidth = ImGui::GetContentRegionAvail().x;
		if (CalCtx.validProfile)
			buttonWidth = (buttonWidth - style.ItemSpacing.x * 2.0f) / 3.0f;

		if (ImGui::Button("Calibrate", ImVec2(buttonWidth, ImGui::GetTextLineHeight() * 2)))
		{
			ImGui::OpenPopup("Calibration Progress");
			StartCalibration();
		}

		if (CalCtx.validProfile)
		{
			ImGui::SameLine();
			if (ImGui::Button("Edit Calibration", ImVec2(buttonWidth, ImGui::GetTextLineHeight() * 2)))
			{
				CalCtx.state = CalibrationState::Editing;
			}

			ImGui::SameLine();
			if (ImGui::Button("Remove Calibration", ImVec2(buttonWidth, ImGui::GetTextLineHeight() * 2)))
			{
				CalCtx.Clear();
				SaveProfile(CalCtx);
			}
		}

		float chapWidth = ImGui::GetContentRegionAvail().x;
		if (CalCtx.chaperone.valid)
			chapWidth = (chapWidth - style.ItemSpacing.x) / 2.0f;

		ImGui::Text("");
		if (ImGui::Button("Copy Chaperone Bounds to profile", ImVec2(chapWidth, ImGui::GetTextLineHeight() * 2)))
		{
			LoadChaperoneBounds();
			SaveProfile(CalCtx);
		}

		if (CalCtx.chaperone.valid)
		{
			ImGui::SameLine();
			if (ImGui::Button("Paste Chaperone Bounds", ImVec2(chapWidth, ImGui::GetTextLineHeight() * 2)))
			{
				ApplyChaperoneBounds();
			}

			if (ImGui::Checkbox(" Paste Chaperone Bounds automatically when geometry resets", &CalCtx.chaperone.autoApply))
			{
				SaveProfile(CalCtx);
			}
		}

		ImGui::Text("");
		auto speed = CalCtx.calibrationSpeed;

		ImGui::Columns(4, NULL, false);
		ImGui::Text("Calibration Speed");

		ImGui::NextColumn();
		if (ImGui::RadioButton(" Fast          ", speed == CalibrationContext::FAST))
			CalCtx.calibrationSpeed = CalibrationContext::FAST;

		ImGui::NextColumn();
		if (ImGui::RadioButton(" Slow          ", speed == CalibrationContext::SLOW))
			CalCtx.calibrationSpeed = CalibrationContext::SLOW;

		ImGui::NextColumn();
		if (ImGui::RadioButton(" Very Slow     ", speed == CalibrationContext::VERY_SLOW))
			CalCtx.calibrationSpeed = CalibrationContext::VERY_SLOW;

		ImGui::Columns(1);
	}
	else if (CalCtx.state == CalibrationState::Editing)
	{
		BuildProfileEditor();

		if (ImGui::Button("Save Profile", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetTextLineHeight() * 2)))
		{
			SaveProfile(CalCtx);
			CalCtx.state = CalibrationState::None;
		}
	}
	else
	{
		ImGui::Button("Calibration in progress...", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetTextLineHeight() * 2));
	}

	float footerHeight = ImGui::GetTextLineHeightWithSpacing() * (runningInOverlay ? 2.0f : 1.0f);
	ImGui::SetCursorPos(ImVec2(10.0f, ImGui::GetWindowHeight() - footerHeight - style.WindowPadding.y));
	ImGui::BeginChild("bottom line", ImVec2(ImGui::GetWindowWidth() - 20.0f, footerHeight), false,
		ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
	ImGui::Text("OpenVR-SpaceOverride v" SPACECAL_VERSION_STRING " - by Nyabsi (Special thanks to tach/pushrax for OpenVR-SpaceCalibrator)");
	if (runningInOverlay)
	{
		ImGui::Text("close VR overlay to use mouse");
	}
	ImGui::EndChild();

	ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f));
	ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x - 40.0f, io.DisplaySize.y - 40.0f));
	if (ImGui::BeginPopupModal("Calibration Progress", nullptr, bareWindowFlags))
	{
		ImGui::PushStyleColor(ImGuiCol_FrameBg, (ImVec4)ImColor(0, 0, 0));
		for (auto &message : CalCtx.messages)
		{
			switch (message.type)
			{
			case CalibrationContext::Message::String:
				ImGui::TextWrapped(message.str.c_str());
				break;
			case CalibrationContext::Message::Progress:
				float fraction = (float)message.progress / (float)message.target;
				ImGui::Text("");
				ImGui::ProgressBar(fraction, ImVec2(-1.0f, 0.0f), "");
				ImGui::SetCursorPosY(ImGui::GetCursorPosY() - ImGui::GetFontSize() - style.FramePadding.y * 2);
				ImGui::Text(" %d%%", (int)(fraction * 100));
				break;
			}
		}
		ImGui::PopStyleColor();

		if (CalCtx.state == CalibrationState::None)
		{
			ImGui::Text("");
			if (ImGui::Button("Close", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetTextLineHeight() * 2)))
				ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}
}

void BuildStatus(const VRState &state)
{
	const VRDevice *hmd = nullptr;
	const VRDevice *tracker = nullptr;
	for (auto &device : state.devices)
	{
		if (device.id == vr::k_unTrackedDeviceIndex_Hmd)
			hmd = &device;
		if (!CalCtx.trackerSerial.empty() && device.serial == CalCtx.trackerSerial)
			tracker = &device;
	}

	if (hmd)
		ImGui::Text("HMD: %s (%s)", hmd->serial.c_str(), hmd->trackingSystem.c_str());
	else
		ImGui::TextColored(ImColor(0.8f, 0.2f, 0.2f), "No HMD detected");

	if (!CalCtx.validProfile)
	{
		ImGui::TextColored(ImColor(0.5f, 0.5f, 0.5f), "No calibration. Press Calibrate, then move your head to identify the headset tracker.");
		return;
	}

	if (!tracker)
		ImGui::TextColored(ImColor(0.8f, 0.2f, 0.2f), "Headset tracker (%s) not connected, override disabled", CalCtx.trackerSerial.c_str());
	else if (!CalCtx.enabled)
		ImGui::TextColored(ImColor(0.8f, 0.2f, 0.2f), "Override disabled (HMD tracking system changed?)");
	else
	{
		ImGui::TextColored(ImColor(0.2f, 0.7f, 0.2f), "Override active: HMD driven by %s (%s)", tracker->serial.c_str(), tracker->trackingSystem.c_str());
	}
}

VRState LoadVRState()
{
	VRState state;
	auto &trackingSystems = state.trackingSystems;

	char buffer[vr::k_unMaxPropertyStringSize];

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;

		if (deviceClass != vr::TrackedDeviceClass_TrackingReference)
		{
			vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);

			if (err == vr::TrackedProp_Success)
			{
				std::string system(buffer);
				auto existing = std::find(trackingSystems.begin(), trackingSystems.end(), system);
				if (existing != trackingSystems.end())
				{
					if (deviceClass == vr::TrackedDeviceClass_HMD)
					{
						trackingSystems.erase(existing);
						trackingSystems.insert(trackingSystems.begin(), system);
					}
				}
				else
				{
					trackingSystems.push_back(system);
				}

				VRDevice device;
				device.id = id;
				device.deviceClass = deviceClass;
				device.trackingSystem = system;

				vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_ModelNumber_String, buffer, vr::k_unMaxPropertyStringSize, &err);
				device.model = std::string(buffer);

				vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, buffer, vr::k_unMaxPropertyStringSize, &err);
				device.serial = std::string(buffer);

				device.controllerRole = (vr::ETrackedControllerRole) vr::VRSystem()->GetInt32TrackedDeviceProperty(id, vr::Prop_ControllerRoleHint_Int32, &err);
				state.devices.push_back(device);
			}
			else
			{
				printf("failed to get tracking system name for id %d\n", id);
			}
		}
	}

	return state;
}

void BuildProfileEditor()
{
	ImGuiStyle &style = ImGui::GetStyle();
	float width = ImGui::GetContentRegionAvail().x / 3.0f - style.FramePadding.x;
	float widthF = width - style.FramePadding.x;

	TextWithWidth("YawLabel", "Yaw", width);
	ImGui::SameLine();
	TextWithWidth("PitchLabel", "Pitch", width);
	ImGui::SameLine();
	TextWithWidth("RollLabel", "Roll", width);

	ImGui::PushItemWidth(widthF);
	ImGui::InputDouble("##Yaw", &CalCtx.calibratedRotation(1), 0.1, 1.0, "%.8f");
	ImGui::SameLine();
	ImGui::InputDouble("##Pitch", &CalCtx.calibratedRotation(2), 0.1, 1.0, "%.8f");
	ImGui::SameLine();
	ImGui::InputDouble("##Roll", &CalCtx.calibratedRotation(0), 0.1, 1.0, "%.8f");

	TextWithWidth("XLabel", "X", width);
	ImGui::SameLine();
	TextWithWidth("YLabel", "Y", width);
	ImGui::SameLine();
	TextWithWidth("ZLabel", "Z", width);

	ImGui::InputDouble("##X", &CalCtx.calibratedTranslation(0), 1.0, 10.0, "%.8f");
	ImGui::SameLine();
	ImGui::InputDouble("##Y", &CalCtx.calibratedTranslation(1), 1.0, 10.0, "%.8f");
	ImGui::SameLine();
	ImGui::InputDouble("##Z", &CalCtx.calibratedTranslation(2), 1.0, 10.0, "%.8f");

	TextWithWidth("ScaleLabel", "Scale", width);

	ImGui::InputDouble("##Scale", &CalCtx.calibratedScale, 0.0001, 0.01, "%.8f");
	ImGui::PopItemWidth();
}

void TextWithWidth(const char *label, const char *text, float width)
{
	ImGui::BeginChild(label, ImVec2(width, ImGui::GetTextLineHeightWithSpacing()));
	ImGui::Text(text);
	ImGui::EndChild();
}
