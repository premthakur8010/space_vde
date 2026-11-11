// =============================================================================
// src/UiWindows.cpp
// Implements the high-level UI panels, modal dialogs, and workspace controllers:
//   - VDE & Spacecraft read-only telemetry value panels
//   - ConfigXmlDialog (XML configuration preview + UDP RunConfig re-send)
//   - ReplayDialog & drawReplayBar (CSV file selection and playback transport)
//   - VdeWindow, SpacecraftWindow, and 12-graph AnalysisWindow
//   - MainUi (main menu bar with live FPS/UDP telemetry counters & frame driver)
// =============================================================================
#include "UiWindows.hpp"

#include "UiCore.hpp"
#include "space_docking/dynamics_engine/XmlConfigUtils.hpp"
#include "space_docking/dynamics_engine/XmlTags.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commdlg.h>
#undef near
#undef far
#endif

namespace vde::ui {

namespace {

// ============================================================================
// Telemetry Value Panels (VDE & Spacecraft)
// ============================================================================

// Renders the VDE control panel displaying B_sp -> P_sp kinematics (mm, deg), platform loads (N, N*m), and travel limits.
void drawVdeValuesPanel(const VdeState& s, const VdeLimits& limits, bool& popped) {
	using namespace units;
	mutedText("Read only · values update live");
	if (smallButtonRight(popped ? "Dock back" : "Pop out")) popped = !popped;

	if (ImGui::BeginTable("##frames", 2)) {
		ImGui::TableNextColumn(); readOnlyText("From frame", "B_sp (VDE base)");
		ImGui::TableNextColumn(); readOnlyText("To frame", "P_sp (moving platform)");
		ImGui::EndTable();
	}

	sectionHeader("Position");
	valueRow("Position (mm)", scaled(s.positionM, kMmPerM));
	valueRow("Linear rate (mm/s)", scaled(s.linearRateMS, kMmPerM));

	sectionHeader("Rotation");
	valueRow("Orientation (°)", scaled(s.orientationRad, kDegPerRad));
	valueRow("Orientation rate (°/s)", scaled(s.orientationRateRadS, kDegPerRad));

	sectionHeader("Forces & torques");
	valueRow("Moving platform force (N)", s.moving.force);
	valueRow("Moving platform torque (N·m)", s.moving.torque);
	valueRow("Overhead platform force (N)", s.overhead.force);
	valueRow("Overhead platform torque (N·m)", s.overhead.torque);

	sectionHeader("Limits");
	mutedText("x %g…%g mm · y %g…%g mm · z %g…%g mm", limits.xMinM * kMmPerM, limits.xMaxM * kMmPerM,
			  limits.yMinM * kMmPerM, limits.yMaxM * kMmPerM, limits.zMinM * kMmPerM, limits.zMaxM * kMmPerM);
	mutedText("roll ±%g° · pitch ±%g° · yaw ±%g°", limits.rollMaxRad * kDegPerRad, limits.pitchMaxRad * kDegPerRad,
			  limits.yawMaxRad * kDegPerRad);
}

// UI state for the Spacecraft side panel (Chaser vs Target selector and pop-out toggle).
struct SpacecraftPanelState {
	bool showTarget = false;
	bool popped     = false;
};

// Renders the Spacecraft control panel with Chaser/Target toggle and Properties / Orbit / NGC tabs.
void drawSpacecraftValuesPanel(const SpacecraftBundle& sc, SpacecraftPanelState& state) {
	mutedText("Read only · values update live");
	if (smallButtonRight(state.popped ? "Dock back" : "Pop out")) state.popped = !state.popped;

	const float w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
	for (int i = 0; i < 2; ++i) {
		const bool sel = (i == 1) == state.showTarget;
		ImGui::PushStyleColor(ImGuiCol_Button, sel ? IM_COL32(0x3A, 0x3A, 0x3A, 255) : IM_COL32_WHITE);
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, sel ? IM_COL32(0x50, 0x50, 0x50, 255) : color::kLightBlue);
		ImGui::PushStyleColor(ImGuiCol_Text, sel ? IM_COL32_WHITE : IM_COL32(0x22, 0x22, 0x22, 255));
		if (ImGui::Button(i == 0 ? "Chaser" : "Target", ImVec2(w, 0))) state.showTarget = (i == 1);
		ImGui::PopStyleColor(3);
		if (i == 0) ImGui::SameLine();
	}

	const SpacecraftState& s = state.showTarget ? sc.target : sc.chaser;
	if (ImGui::BeginTabBar("##scTabs")) {
		if (ImGui::BeginTabItem("Properties")) {
			readOnlyText("Frame", state.showTarget ? "D_T (Target docking hardware)" : "D_C (Chaser docking hardware)");
			sectionHeader("Mass properties");
			valueRow("Mass (kg)", s.massKg);
			valueRow("Centre of gravity (m)", s.cgM);
			static const char* const kXyz[] = {"x", "y", "z"};
			for (int r = 0; r < 3; ++r) {
				const double row[] = {s.inertiaKgM2.m[r * 3 + 0], s.inertiaKgM2.m[r * 3 + 1], s.inertiaKgM2.m[r * 3 + 2]};
				valueRow(r == 0 ? "Inertia matrix (kg·m²)" : "", row, 3, r == 2 ? kXyz : nullptr);
			}
			sectionHeader("Rates");
			valueRow("Linear rate (m/s)", s.linearRateMS);
			valueRow("Orientation rate (°/s)", units::scaled(s.orientationRateRadS, units::kDegPerRad));
			ImGui::EndTabItem();
		}
		if (ImGui::BeginTabItem("Orbit")) {
			readOnlyText("Frame", "ECI (Earth-centred inertial)");
			sectionHeader("Translation");
			valueRow("Position (km)", units::scaled(s.eciPositionM, units::kKmPerM));
			valueRow("Velocity (km/h)", units::scaled(s.eciVelocityMS, units::kKmhPerMs));
			sectionHeader("Rotation");
			valueRow("Orientation (°)", units::scaled(s.eciOrientationRad, units::kDegPerRad));
			valueRow("Orientation rate (°/s)", units::scaled(s.eciOrientationRateRadS, units::kDegPerRad));
			ImGui::EndTabItem();
		}
		if (ImGui::BeginTabItem("NGC")) {
			const NgcState& n = sc.ngc;
			readOnlyText("NGC mode", n.mode.c_str());
			readOnlyText("Failure case", n.failureCase.c_str());
			sectionHeader("Chaser");
			valueRow("Chaser force (N)", n.chaserForceN);
			valueRow("Chaser torque (N·m)", n.chaserTorqueNm);
			sectionHeader("Target");
			valueRow("Target force (N)", n.targetForceN);
			valueRow("Target torque (N·m)", n.targetTorqueNm);
			sectionHeader("Environment");
			readOnlyText("Disturbances", n.disturbancesOn ? "On" : "Off");
			ImGui::EndTabItem();
		}
		ImGui::EndTabBar();
	}
}

// ============================================================================
// Dialogs (ConfigXmlDialog, ReplayDialog, ReplayBar)
// ============================================================================

// Modal popup displaying editable input boxes for all values defined in simulation_config.xml tags,
// allowing operators to dynamically update configuration on disk and transmit via UDP RunConfigPacket.
class ConfigXmlDialog {
public:
	void open(const AppState& app) {
		m_configFilePath = vde::resolveConfigPath("simulation_config.xml");
		m_defaultFilePath = vde::resolveConfigPath("default.xml");
		loadFromXmlFile(m_configFilePath, app);
		m_title = "Edit XML Configuration (" + m_configFilePath + ")###ConfigXmlDialog";
		m_statusMessage.clear();
		m_statusTimer = 0.0f;
		m_openRequested = true;
	}

	void draw(AppState& app) {
		if (m_openRequested) {
			ImGui::OpenPopup(m_title.c_str());
			m_openRequested = false;
		}

		const ImGuiViewport* vp = ImGui::GetMainViewport();
		ImGui::SetNextWindowSize(ImVec2(vp->Size.x * 0.65f, vp->Size.y * 0.78f), ImGuiCond_Appearing);
		ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + vp->Size.x * 0.5f, vp->Pos.y + vp->Size.y * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

		if (!ImGui::BeginPopupModal(m_title.c_str(), nullptr, ImGuiWindowFlags_NoSavedSettings)) return;

		// Top status / informational banner
		if (m_statusTimer > 0.0f) {
			m_statusTimer -= ImGui::GetIO().DeltaTime;
			ImGui::TextColored(ImVec4(0.2f, 0.75f, 0.3f, 1.0f), "%s", m_statusMessage.c_str());
		} else {
			mutedText("Configuration file: %s · Edit tag values in the boxes below and click 'SAVE XML & SEND RUNCONFIG (UDP)'",
			          m_configFilePath.c_str());
		}
		ImGui::Spacing();

		const float dpi = ImGui::GetStyle().FontScaleDpi;
		const float footerH = ImGui::GetFrameHeightWithSpacing() + ImGui::GetStyle().ItemSpacing.y * 2.0f;

		// Scrollable area holding the XML tag input boxes
		if (ImGui::BeginChild("##XmlBoxesScroll", ImVec2(0, -footerH), true)) {
			const float boxW = 160.0f * dpi;

			auto tagRow = [&](const char* tagLabel, char* buf, size_t bufSize, const char* unitOrDesc) {
				ImGui::TableNextRow();
				ImGui::TableNextColumn();
				ImGui::PushFont(fonts().mono, 0.0f);
				ImGui::TextColored(ImVec4(0.20f, 0.50f, 0.90f, 1.0f), "<%s>", tagLabel);
				ImGui::PopFont();

				ImGui::TableNextColumn();
				ImGui::SetNextItemWidth(boxW);
				char id[128];
				std::snprintf(id, sizeof(id), "##box_%s_%p", tagLabel, (void*)buf);
				ImGui::PushStyleColor(ImGuiCol_FrameBg, IM_COL32(0xFA, 0xFA, 0xFA, 255));
				ImGui::PushStyleColor(ImGuiCol_Border, IM_COL32(0xBD, 0xC3, 0xC7, 255));
				ImGui::InputText(id, buf, bufSize);
				ImGui::PopStyleColor(2);

				if (unitOrDesc && unitOrDesc[0] != '\0') {
					ImGui::SameLine();
					ImGui::TextDisabled("%s", unitOrDesc);
				}
			};

			auto sectionBanner = [&](const char* tagPath, const char* desc) {
				ImGui::Spacing();
				ImGui::PushFont(fonts().mono, 0.0f);
				char hdr[160];
				std::snprintf(hdr, sizeof(hdr), "<%s> · %s", tagPath, desc);
				ImGui::SeparatorText(hdr);
				ImGui::PopFont();
				ImGui::Spacing();
			};

			// 1. Chaser Spacecraft Section
			sectionBanner("spacecraft><chaser", "Chaser Spacecraft (Active Docking Vehicle)");
			if (ImGui::BeginTable("##chaserTable", 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
				ImGui::TableSetupColumn("Tag", ImGuiTableColumnFlags_WidthFixed, 180.0f * dpi);
				ImGui::TableSetupColumn("Value Box", ImGuiTableColumnFlags_WidthStretch);

				tagRow("masskg", m_chaser.massKg, sizeof(m_chaser.massKg), "kg (Spacecraft dry/wet mass)");
				tagRow("ixx", m_chaser.ixx, sizeof(m_chaser.ixx), "kg·m² (Inertia tensor X-axis)");
				tagRow("iyy", m_chaser.iyy, sizeof(m_chaser.iyy), "kg·m² (Inertia tensor Y-axis)");
				tagRow("izz", m_chaser.izz, sizeof(m_chaser.izz), "kg·m² (Inertia tensor Z-axis)");
				tagRow("x", m_chaser.posX, sizeof(m_chaser.posX), "mm (Initial separation along X-axis, e.g. -550.0)");
				tagRow("y", m_chaser.posY, sizeof(m_chaser.posY), "mm (Initial separation along Y-axis)");
				tagRow("z", m_chaser.posZ, sizeof(m_chaser.posZ), "mm (Initial separation along Z-axis)");
				tagRow("vx", m_chaser.velX, sizeof(m_chaser.velX), "m/s (Closing approach velocity Vx, e.g. 0.05)");
				tagRow("vy", m_chaser.velY, sizeof(m_chaser.velY), "m/s (Initial velocity Vy)");
				tagRow("vz", m_chaser.velZ, sizeof(m_chaser.velZ), "m/s (Initial velocity Vz)");

				ImGui::EndTable();
			}

			// 2. Target Spacecraft Section
			sectionBanner("spacecraft><target", "Target Spacecraft (Passive / Station Vehicle)");
			if (ImGui::BeginTable("##targetTable", 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
				ImGui::TableSetupColumn("Tag", ImGuiTableColumnFlags_WidthFixed, 180.0f * dpi);
				ImGui::TableSetupColumn("Value Box", ImGuiTableColumnFlags_WidthStretch);

				tagRow("masskg", m_target.massKg, sizeof(m_target.massKg), "kg (Spacecraft mass)");
				tagRow("ixx", m_target.ixx, sizeof(m_target.ixx), "kg·m² (Inertia tensor X-axis)");
				tagRow("iyy", m_target.iyy, sizeof(m_target.iyy), "kg·m² (Inertia tensor Y-axis)");
				tagRow("izz", m_target.izz, sizeof(m_target.izz), "kg·m² (Inertia tensor Z-axis)");
				tagRow("x", m_target.posX, sizeof(m_target.posX), "mm (Target position origin = 0)");
				tagRow("y", m_target.posY, sizeof(m_target.posY), "mm (Target position origin = 0)");
				tagRow("z", m_target.posZ, sizeof(m_target.posZ), "mm (Target position origin = 0)");
				tagRow("vx", m_target.velX, sizeof(m_target.velX), "m/s (Target velocity Vx = 0)");
				tagRow("vy", m_target.velY, sizeof(m_target.velY), "m/s (Target velocity Vy = 0)");
				tagRow("vz", m_target.velZ, sizeof(m_target.velZ), "m/s (Target velocity Vz = 0)");

				ImGui::EndTable();
			}

			// 3. Platform Section
			sectionBanner("platform", "Hardware-in-the-Loop Stewart Platform Mounting");
			if (ImGui::BeginTable("##platTable", 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
				ImGui::TableSetupColumn("Tag", ImGuiTableColumnFlags_WidthFixed, 180.0f * dpi);
				ImGui::TableSetupColumn("Value Box", ImGuiTableColumnFlags_WidthStretch);

				tagRow("framemapping", m_platform.frameMapping, sizeof(m_platform.frameMapping), "default (Chaser on GSP) or flipped (Target on GSP)");

				ImGui::EndTable();
			}

			// 4. Network Section
			sectionBanner("network", "UDP Network Endpoints (Dynamics Engine <-> Visualizer)");
			if (ImGui::BeginTable("##netTable", 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
				ImGui::TableSetupColumn("Tag", ImGuiTableColumnFlags_WidthFixed, 180.0f * dpi);
				ImGui::TableSetupColumn("Value Box", ImGuiTableColumnFlags_WidthStretch);

				tagRow("targethost", m_network.targetHost, sizeof(m_network.targetHost), "Destination host IP address");
				tagRow("configport", m_network.configPort, sizeof(m_network.configPort), "UDP port for RunConfig (Inbound to Dynamics Engine)");
				tagRow("stateport", m_network.statePort, sizeof(m_network.statePort), "UDP port for State telemetry (Inbound to UI Visualizer)");

				ImGui::EndTable();
			}

			// 5. Logging Section
			sectionBanner("logging", "Simulation Trajectory Output & Provision");
			if (ImGui::BeginTable("##logTable", 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
				ImGui::TableSetupColumn("Tag", ImGuiTableColumnFlags_WidthFixed, 180.0f * dpi);
				ImGui::TableSetupColumn("Value Box", ImGuiTableColumnFlags_WidthStretch);

				tagRow("enabled", m_logging.enabled, sizeof(m_logging.enabled), "true or false");
				tagRow("logdirectory", m_logging.logDirectory, sizeof(m_logging.logDirectory), "Output folder for logs and CSVs");
				tagRow("logfilename", m_logging.logFileName, sizeof(m_logging.logFileName), "CSV telemetry log filename");
				tagRow("logdecimation", m_logging.logDecimation, sizeof(m_logging.logDecimation), "Output decimation rate in steps (e.g. 10 = every 10 ms)");

				ImGui::EndTable();
			}

			ImGui::EndChild();
		}

		// Footer buttons
		const float sendW = 230.0f * dpi;
		const float reloadW = 150.0f * dpi;
		const float closeW = 110.0f * dpi;

		ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - sendW - reloadW - closeW - ImGui::GetStyle().ItemSpacing.x * 2.0f);

		if (ImGui::Button("SAVE XML & SEND (UDP)", ImVec2(sendW, 0))) {
			saveAndApply(app);
		}
		ImGui::SameLine();
		if (ImGui::Button("REVERT TO DEFAULT", ImVec2(reloadW, 0))) {
			loadFromDefaultXmlFile(app);
			m_statusMessage = "Reverted all boxes to default values from default.xml.";
			m_statusTimer = 3.5f;
		}
		ImGui::SameLine();
		if (primaryButton("CLOSE", ImVec2(closeW, 0)) || ImGui::IsKeyPressed(ImGuiKey_Escape)) {
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}

private:
	struct ChaserFields {
		char massKg[64] = "50.0";
		char ixx[64]    = "5.0";
		char iyy[64]    = "5.0";
		char izz[64]    = "10.0";
		char posX[64]   = "-550.0";
		char posY[64]   = "0.0";
		char posZ[64]   = "0.0";
		char velX[64]   = "0.05";
		char velY[64]   = "0.0";
		char velZ[64]   = "0.0";
	};

	struct TargetFields {
		char massKg[64] = "100.0";
		char ixx[64]    = "10.0";
		char iyy[64]    = "10.0";
		char izz[64]    = "20.0";
		char posX[64]   = "0.0";
		char posY[64]   = "0.0";
		char posZ[64]   = "0.0";
		char velX[64]   = "0.0";
		char velY[64]   = "0.0";
		char velZ[64]   = "0.0";
	};

	struct PlatformFields {
		char frameMapping[64] = "default";
	};

	struct NetworkFields {
		char targetHost[64] = "127.0.0.1";
		char configPort[64] = "5001";
		char statePort[64]  = "5002";
	};

	struct LoggingFields {
		char enabled[64]       = "true";
		char logDirectory[128] = "logs";
		char logFileName[128]  = "simulation_trajectory.csv";
		char logDecimation[64] = "10";
	};

	std::string    m_title;
	std::string    m_configFilePath  = "config/simulation_config.xml";
	std::string    m_defaultFilePath = "config/default.xml";
	std::string    m_rawXml;
	std::string    m_statusMessage;
	float          m_statusTimer = 0.0f;
	bool           m_openRequested = false;

	ChaserFields   m_chaser;
	TargetFields   m_target;
	PlatformFields m_platform;
	NetworkFields  m_network;
	LoggingFields  m_logging;

	void loadFromDefaultXmlFile(const AppState& app) {
		m_defaultFilePath = vde::resolveConfigPath("default.xml");
		std::ifstream file(m_defaultFilePath);
		if (file.is_open()) {
			std::stringstream ss;
			ss << file.rdbuf();
			m_rawXml = ss.str();
			parseXmlToFields(m_rawXml);
		} else {
			loadFromXmlFile(m_configFilePath, app);
		}
	}

	static void copyField(char* dest, size_t destSize, const std::string& src) {
		std::string trimmed = space_docking::dynamics::xml_utils::trim(src);
		if (!trimmed.empty()) {
			std::snprintf(dest, destSize, "%s", trimmed.c_str());
		}
	}

	static bool replaceTagValue(std::string& xml, size_t searchStart, size_t searchEnd,
	                            const std::string& tag, const std::string& newValue) {
		std::string openTag = "<" + tag + ">";
		std::string closeTag = "</" + tag + ">";

		std::string_view slice(xml.data() + searchStart, searchEnd - searchStart);
		size_t relOpen = space_docking::dynamics::xml_utils::findCaseInsensitive(slice, openTag);
		if (relOpen == std::string_view::npos) return false;

		size_t valStart = searchStart + relOpen + openTag.size();
		size_t valMaxLen = searchEnd - valStart;
		std::string_view valSlice(xml.data() + valStart, valMaxLen);
		size_t relClose = space_docking::dynamics::xml_utils::findCaseInsensitive(valSlice, closeTag);
		if (relClose == std::string_view::npos) return false;

		size_t valEnd = valStart + relClose;
		xml.replace(valStart, valEnd - valStart, newValue);
		return true;
	}

	static bool updateXmlBlock(std::string& xml, const std::string& parentTag,
	                           const std::string& childTag, const std::string& newValue) {
		std::string openParent = "<" + parentTag + ">";
		std::string closeParent = "</" + parentTag + ">";
		size_t pStart = space_docking::dynamics::xml_utils::findCaseInsensitive(xml, openParent);
		if (pStart == std::string::npos) return false;
		size_t pEnd = space_docking::dynamics::xml_utils::findCaseInsensitive(xml, closeParent, pStart);
		if (pEnd == std::string::npos) return false;
		return replaceTagValue(xml, pStart, pEnd, childTag, newValue);
	}

	static bool updateXmlSubBlock(std::string& xml, const std::string& parentTag,
	                              const std::string& subBlockTag,
	                              const std::string& childTag, const std::string& newValue) {
		std::string openParent = "<" + parentTag + ">";
		std::string closeParent = "</" + parentTag + ">";
		size_t pStart = space_docking::dynamics::xml_utils::findCaseInsensitive(xml, openParent);
		if (pStart == std::string::npos) return false;
		size_t pEnd = space_docking::dynamics::xml_utils::findCaseInsensitive(xml, closeParent, pStart);
		if (pEnd == std::string::npos) return false;

		std::string openSub = "<" + subBlockTag + ">";
		std::string closeSub = "</" + subBlockTag + ">";
		std::string_view parentSlice(xml.data() + pStart, pEnd - pStart);
		size_t relSubStart = space_docking::dynamics::xml_utils::findCaseInsensitive(parentSlice, openSub);
		if (relSubStart == std::string_view::npos) return false;
		size_t absSubStart = pStart + relSubStart;
		size_t absSubEnd = space_docking::dynamics::xml_utils::findCaseInsensitive(xml, closeSub, absSubStart);
		if (absSubEnd == std::string::npos || absSubEnd > pEnd) return false;

		return replaceTagValue(xml, absSubStart, absSubEnd, childTag, newValue);
	}

	void loadFromXmlFile(const std::string& path, const AppState& app) {
		std::ifstream file(path);
		if (file.is_open()) {
			std::stringstream ss;
			ss << file.rdbuf();
			m_rawXml = ss.str();
		} else {
			m_rawXml = app.loadConfigXml();
		}
		parseXmlToFields(m_rawXml);
	}

	void parseXmlToFields(const std::string& xml) {
		using namespace space_docking::dynamics::xml_tags;
		using space_docking::dynamics::xml_utils::extractTagContent;

		// Chaser
		std::string chaserBlock = extractTagContent(xml, kChaser);
		if (!chaserBlock.empty()) {
			copyField(m_chaser.massKg, sizeof(m_chaser.massKg), extractTagContent(chaserBlock, kMassKg));
			std::string inertia = extractTagContent(chaserBlock, kInertia);
			if (!inertia.empty()) {
				copyField(m_chaser.ixx, sizeof(m_chaser.ixx), extractTagContent(inertia, kIxx));
				copyField(m_chaser.iyy, sizeof(m_chaser.iyy), extractTagContent(inertia, kIyy));
				copyField(m_chaser.izz, sizeof(m_chaser.izz), extractTagContent(inertia, kIzz));
			}
			std::string pos = extractTagContent(chaserBlock, kInitialPosition);
			if (!pos.empty()) {
				copyField(m_chaser.posX, sizeof(m_chaser.posX), extractTagContent(pos, kX));
				copyField(m_chaser.posY, sizeof(m_chaser.posY), extractTagContent(pos, kY));
				copyField(m_chaser.posZ, sizeof(m_chaser.posZ), extractTagContent(pos, kZ));
			}
			std::string vel = extractTagContent(chaserBlock, kInitialVelocity);
			if (!vel.empty()) {
				copyField(m_chaser.velX, sizeof(m_chaser.velX), extractTagContent(vel, kVx));
				copyField(m_chaser.velY, sizeof(m_chaser.velY), extractTagContent(vel, kVy));
				copyField(m_chaser.velZ, sizeof(m_chaser.velZ), extractTagContent(vel, kVz));
			}
		}

		// Target
		std::string targetBlock = extractTagContent(xml, kTarget);
		if (!targetBlock.empty()) {
			copyField(m_target.massKg, sizeof(m_target.massKg), extractTagContent(targetBlock, kMassKg));
			std::string inertia = extractTagContent(targetBlock, kInertia);
			if (!inertia.empty()) {
				copyField(m_target.ixx, sizeof(m_target.ixx), extractTagContent(inertia, kIxx));
				copyField(m_target.iyy, sizeof(m_target.iyy), extractTagContent(inertia, kIyy));
				copyField(m_target.izz, sizeof(m_target.izz), extractTagContent(inertia, kIzz));
			}
			std::string pos = extractTagContent(targetBlock, kInitialPosition);
			if (!pos.empty()) {
				copyField(m_target.posX, sizeof(m_target.posX), extractTagContent(pos, kX));
				copyField(m_target.posY, sizeof(m_target.posY), extractTagContent(pos, kY));
				copyField(m_target.posZ, sizeof(m_target.posZ), extractTagContent(pos, kZ));
			}
			std::string vel = extractTagContent(targetBlock, kInitialVelocity);
			if (!vel.empty()) {
				copyField(m_target.velX, sizeof(m_target.velX), extractTagContent(vel, kVx));
				copyField(m_target.velY, sizeof(m_target.velY), extractTagContent(vel, kVy));
				copyField(m_target.velZ, sizeof(m_target.velZ), extractTagContent(vel, kVz));
			}
		}

		// Platform
		std::string platBlock = extractTagContent(xml, kPlatform);
		if (!platBlock.empty()) {
			copyField(m_platform.frameMapping, sizeof(m_platform.frameMapping), extractTagContent(platBlock, kFrameMapping));
		}

		// Network
		std::string netBlock = extractTagContent(xml, kNetwork);
		if (!netBlock.empty()) {
			copyField(m_network.targetHost, sizeof(m_network.targetHost), extractTagContent(netBlock, kTargetHost));
			copyField(m_network.configPort, sizeof(m_network.configPort), extractTagContent(netBlock, kConfigPort));
			copyField(m_network.statePort, sizeof(m_network.statePort), extractTagContent(netBlock, kStatePort));
		}

		// Logging
		std::string logBlock = extractTagContent(xml, kLogging);
		if (!logBlock.empty()) {
			copyField(m_logging.enabled, sizeof(m_logging.enabled), extractTagContent(logBlock, kEnabled));
			copyField(m_logging.logDirectory, sizeof(m_logging.logDirectory), extractTagContent(logBlock, kLogDirectory));
			copyField(m_logging.logFileName, sizeof(m_logging.logFileName), extractTagContent(logBlock, kLogFileName));
			copyField(m_logging.logDecimation, sizeof(m_logging.logDecimation), extractTagContent(logBlock, kLogDecimation));
		}
	}

	void saveAndApply(AppState& app) {
		using namespace space_docking::dynamics::xml_tags;

		// 1. Update XML text in memory
		updateXmlBlock(m_rawXml, kChaser, kMassKg, m_chaser.massKg);
		updateXmlSubBlock(m_rawXml, kChaser, kInertia, kIxx, m_chaser.ixx);
		updateXmlSubBlock(m_rawXml, kChaser, kInertia, kIyy, m_chaser.iyy);
		updateXmlSubBlock(m_rawXml, kChaser, kInertia, kIzz, m_chaser.izz);
		updateXmlSubBlock(m_rawXml, kChaser, kInitialPosition, kX, m_chaser.posX);
		updateXmlSubBlock(m_rawXml, kChaser, kInitialPosition, kY, m_chaser.posY);
		updateXmlSubBlock(m_rawXml, kChaser, kInitialPosition, kZ, m_chaser.posZ);
		updateXmlSubBlock(m_rawXml, kChaser, kInitialVelocity, kVx, m_chaser.velX);
		updateXmlSubBlock(m_rawXml, kChaser, kInitialVelocity, kVy, m_chaser.velY);
		updateXmlSubBlock(m_rawXml, kChaser, kInitialVelocity, kVz, m_chaser.velZ);

		updateXmlBlock(m_rawXml, kTarget, kMassKg, m_target.massKg);
		updateXmlSubBlock(m_rawXml, kTarget, kInertia, kIxx, m_target.ixx);
		updateXmlSubBlock(m_rawXml, kTarget, kInertia, kIyy, m_target.iyy);
		updateXmlSubBlock(m_rawXml, kTarget, kInertia, kIzz, m_target.izz);
		updateXmlSubBlock(m_rawXml, kTarget, kInitialPosition, kX, m_target.posX);
		updateXmlSubBlock(m_rawXml, kTarget, kInitialPosition, kY, m_target.posY);
		updateXmlSubBlock(m_rawXml, kTarget, kInitialPosition, kZ, m_target.posZ);
		updateXmlSubBlock(m_rawXml, kTarget, kInitialVelocity, kVx, m_target.velX);
		updateXmlSubBlock(m_rawXml, kTarget, kInitialVelocity, kVy, m_target.velY);
		updateXmlSubBlock(m_rawXml, kTarget, kInitialVelocity, kVz, m_target.velZ);

		updateXmlBlock(m_rawXml, kPlatform, kFrameMapping, m_platform.frameMapping);

		updateXmlBlock(m_rawXml, kNetwork, kTargetHost, m_network.targetHost);
		updateXmlBlock(m_rawXml, kNetwork, kConfigPort, m_network.configPort);
		updateXmlBlock(m_rawXml, kNetwork, kStatePort, m_network.statePort);

		updateXmlBlock(m_rawXml, kLogging, kEnabled, m_logging.enabled);
		updateXmlBlock(m_rawXml, kLogging, kLogDirectory, m_logging.logDirectory);
		updateXmlBlock(m_rawXml, kLogging, kLogFileName, m_logging.logFileName);
		updateXmlBlock(m_rawXml, kLogging, kLogDecimation, m_logging.logDecimation);

		// 2. Persist XML to file on disk
		bool wroteDisk = false;
		if (!m_configFilePath.empty()) {
			std::ofstream out(m_configFilePath);
			if (out.is_open()) {
				out << m_rawXml;
				wroteDisk = true;
			}
		}

		// 3. Update AppState RunConfig and dispatch over UDP
		RunConfig cfg = app.runConfig();
		try {
			cfg.chaserMassKg = std::stod(m_chaser.massKg);
			cfg.targetMassKg = std::stod(m_target.massKg);
			cfg.chaserInertiaKgM2 = Mat3{{{
				std::stod(m_chaser.ixx), 0.0, 0.0,
				0.0, std::stod(m_chaser.iyy), 0.0,
				0.0, 0.0, std::stod(m_chaser.izz)
			}}};
			cfg.targetInertiaKgM2 = Mat3{{{
				std::stod(m_target.ixx), 0.0, 0.0,
				0.0, std::stod(m_target.iyy), 0.0,
				0.0, 0.0, std::stod(m_target.izz)
			}}};
			cfg.alongTrackSepM = std::stod(m_chaser.posX) * 0.001; // mm to meters
			std::string fm = m_platform.frameMapping;
			std::transform(fm.begin(), fm.end(), fm.begin(), ::tolower);
			cfg.frameMapping = (fm == "flipped") ? FrameMapping::Flipped : FrameMapping::Default;
		} catch (...) {}

		app.submit(StartRunCmd{cfg});

		if (wroteDisk) {
			m_statusMessage = "Saved to " + m_configFilePath + " and dispatched RunConfig over UDP!";
		} else {
			m_statusMessage = "Dispatched RunConfig over UDP!";
		}
		m_statusTimer = 4.0f;
	}
};

// Modal popup for choosing a target workspace (VDE, Spacecraft, or Both) and selecting a recorded CSV run file.
class ReplayDialog {
public:
	void open() { m_openRequested = true; }

	void draw(AppState& app) {
		static constexpr const char* kTitle = "Replay a run";
		if (m_openRequested) { ImGui::OpenPopup(kTitle); m_openRequested = false; }
		const ImGuiViewport* vp = ImGui::GetMainViewport();
		ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + vp->Size.x * 0.5f, vp->Pos.y + vp->Size.y * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
		ImGui::SetNextWindowSize(ImVec2(560.0f * ImGui::GetStyle().FontScaleDpi, 0.0f), ImGuiCond_Appearing);
		if (!ImGui::BeginPopupModal(kTitle, nullptr, ImGuiWindowFlags_NoSavedSettings)) return;

		sectionHeader("Step 1 · Replay on");
		int target = (int)m_target;
		ImGui::RadioButton("VDE View", &target, (int)ReplayTarget::VdeView); ImGui::SameLine();
		ImGui::RadioButton("Spacecraft View", &target, (int)ReplayTarget::SpacecraftView); ImGui::SameLine();
		ImGui::RadioButton("Both views", &target, (int)ReplayTarget::Both);
		m_target = (ReplayTarget)target;

		sectionHeader("Step 2 · Run data file (CSV)");
#ifdef _WIN32
		const float browseW = ImGui::CalcTextSize("Browse…").x + ImGui::GetStyle().FramePadding.x * 2;
		ImGui::BeginGroup();
		const float boxW = ImGui::GetContentRegionAvail().x - browseW - ImGui::GetStyle().ItemSpacing.x;
		ImGui::BeginChild("##pathBox", ImVec2(boxW, ImGui::GetFrameHeight()), ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar);
		readOnlyText(nullptr, m_path.empty() ? "(no file selected)" : m_path.c_str());
		ImGui::EndChild();
		ImGui::SameLine();
		if (ImGui::Button("Browse…")) {
			char file[512] = {};
			if (!m_path.empty() && m_path.size() < sizeof(file)) std::snprintf(file, sizeof(file), "%s", m_path.c_str());
			OPENFILENAMEA ofn = {};
			ofn.lStructSize = sizeof(ofn);
			ofn.lpstrFilter = "Run data (*.csv)\0*.csv\0All files (*.*)\0*.*\0";
			ofn.lpstrFile = file; ofn.nMaxFile = sizeof(file);
			ofn.lpstrTitle = "Select run data file";
			ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
			if (GetOpenFileNameA(&ofn)) m_path = file;
		}
		ImGui::EndGroup();
#else
		ImGui::SetNextItemWidth(-1);
		if (ImGui::InputTextWithHint("##path", "path/to/run.csv", m_pathInput, sizeof(m_pathInput))) m_path = m_pathInput;
#endif
		ImGui::Spacing(); ImGui::Separator();
		const float bw = 140.0f * ImGui::GetStyle().FontScaleDpi;
		ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - bw * 2 - ImGui::GetStyle().ItemSpacing.x);
		if (ImGui::Button("CANCEL", ImVec2(bw, 0)) || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
		ImGui::SameLine();
		ImGui::BeginDisabled(m_path.empty());
		if (primaryButton("START REPLAY", ImVec2(bw, 0))) {
			app.submit(StartReplayCmd{m_path, m_target});
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndDisabled();
		ImGui::EndPopup();
	}

private:
	ReplayTarget m_target = ReplayTarget::Both;
	std::string  m_path;
	char         m_pathInput[512] = {};
	bool         m_openRequested = false;
};

// Renders the bottom overlay transport bar (Play, Pause, timeline scrubber, speed selector, Exit) inside active replay views.
void drawReplayBar(AppState& app) {
	const ReplayStatus& r = app.replayStatus();
	const ImGuiStyle& st = ImGui::GetStyle();

	statusTag("REPLAY", color::kOrange); ImGui::SameLine();
	ImGui::BeginDisabled(r.playing);  if (ImGui::Button("Play"))  app.submit(ReplayPlayCmd{});  ImGui::EndDisabled(); ImGui::SameLine();
	ImGui::BeginDisabled(!r.playing); if (ImGui::Button("Pause")) app.submit(ReplayPauseCmd{}); ImGui::EndDisabled(); ImGui::SameLine();

	char timeText[64];
	std::snprintf(timeText, sizeof(timeText), "%s / %s", formatSimTime(r.timeS).c_str(), formatSimTime(r.durationS).c_str());
	std::string file = fileNameOf(app.replayFile());
	if (file.size() > 32) file = file.substr(0, 29) + "…";
	const float speedW = ImGui::CalcTextSize("0.5x").x + ImGui::GetFrameHeight() + st.FramePadding.x * 2;
	const float exitW  = ImGui::CalcTextSize("EXIT REPLAY").x + st.FramePadding.x * 2;
	ImGui::PushFont(fonts().mono, 0.0f);
	const float timeW = ImGui::CalcTextSize(timeText).x;
	ImGui::PopFont();
	const float sliderW = ImGui::GetContentRegionAvail().x -
						  (timeW + speedW + ImGui::CalcTextSize(file.c_str()).x + exitW + st.ItemSpacing.x * 4 + st.WindowPadding.x * 2);

	float t = (float)r.timeS;
	ImGui::SetNextItemWidth(sliderW > 80.0f ? sliderW : 80.0f);
	if (ImGui::SliderFloat("##time", &t, 0.0f, (float)r.durationS, "", ImGuiSliderFlags_NoInput)) app.submit(ReplaySeekCmd{t});
	ImGui::SameLine();
	ImGui::PushFont(fonts().mono, 0.0f); ImGui::AlignTextToFramePadding(); ImGui::TextUnformatted(timeText); ImGui::PopFont();
	ImGui::SameLine();

	static const double kSpeeds[] = {0.5, 1.0, 2.0};
	static const char* kLabels[]  = {"0.5x", "1x", "2x"};
	int cur = 1;
	for (int i = 0; i < 3; ++i) if (nearlyEqual(r.speed, kSpeeds[i], 1e-6)) cur = i;
	ImGui::SetNextItemWidth(speedW);
	if (ImGui::BeginCombo("##speed", kLabels[cur])) {
		for (int i = 0; i < 3; ++i) if (ImGui::Selectable(kLabels[i], i == cur)) app.submit(ReplaySetSpeedCmd{kSpeeds[i]});
		ImGui::EndCombo();
	}
	ImGui::SameLine(); ImGui::AlignTextToFramePadding();
	ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(color::kMuted), "%s", file.c_str());
	ImGui::SameLine();
	if (primaryButton("EXIT REPLAY")) app.submit(StopReplayCmd{});
}

// ============================================================================
// Workspace Windows (VdeWindow, SpacecraftWindow, AnalysisWindow)
// ============================================================================

// Manages the VDE hardware workspace: toolbar, VDE values panel, 3D VdeScene viewport, and 6-chart GraphGrid.
class VdeWindow {
public:
	explicit VdeWindow(AppState& app) : m_app(app), m_scene(std::make_unique<render::VdeScene>()) {}

	ToolbarAction draw(const std::function<void()>& replayBar) {
		ToolbarAction action = ToolbarAction::None;
		m_graphs.sample(m_app.parameters(), m_app.vdeSnapshot());
		if (m_focusFrames > 0) { ImGui::SetNextWindowFocus(); --m_focusFrames; }
		const bool visible = ImGui::Begin(names::kVdeWindow);
		m_focused = visible && ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows | ImGuiFocusedFlags_DockHierarchy);
		if (visible) {
			ToolbarOptions opt{ViewId::Vde, "vde_view", "run", true, true, true, m_app.replayOnVde(), m_app.vdeSnapshot().timeS};
			action = drawToolbar(m_app, opt);
		}
		DockLayout::submitInner(DockLayout::vdeDockspaceId(), visible);
		ImGui::End();

		const VdeState& s = m_app.vdeSnapshot().vde;
		const VdeLimits& lim = m_app.vdeLimits();
		if (ImGui::Begin(names::kVdePanel)) {
			if (m_panelPopped) poppedOutPlaceholder(m_panelPopped);
			else drawVdeValuesPanel(s, lim, m_panelPopped);
		}
		ImGui::End();
		poppedOutWindow("Control Panel – VDE values###VdePanelPopped", m_panelPopped, ImVec2(420, 860),
						[&] { drawVdeValuesPanel(s, lim, m_panelPopped); });

		const SimSnapshot& snap = m_app.vdeSnapshot();
		Recorder* rec = &m_app.recorder(ViewId::Vde);
		auto body = [&] { m_scene.draw(snap, m_viewPopped, rec, replayBar); };
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
		const ImGuiWindowFlags flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
		if (ImGui::Begin(names::kVdeView, nullptr, flags)) {
			if (m_viewPopped) poppedOutPlaceholder(m_viewPopped);
			else body();
		}
		ImGui::End();
		poppedOutWindow("VDE View###VdeViewPopped", m_viewPopped, ImVec2(1100, 800), body, flags);
		ImGui::PopStyleVar();

		if (ImGui::Begin(names::kVdeGraphs)) m_graphs.draw(m_app.parameters());
		ImGui::End();
		m_graphs.drawPopOuts();
		return action;
	}

	bool focused() const { return m_focused; }
	void requestFocus()  { m_focusFrames = 2; }

private:
	AppState& m_app;
	SceneView m_scene;
	GraphGrid m_graphs{6, 3, "vde"};
	bool      m_panelPopped = false, m_viewPopped = false, m_focused = false;
	int       m_focusFrames = 0;
};

// Manages the orbital Spacecraft workspace: toolbar, Spacecraft/NGC panel, 3D SpacecraftScene viewport, and 6-chart GraphGrid.
class SpacecraftWindow {
public:
	explicit SpacecraftWindow(AppState& app) : m_app(app), m_scene(std::make_unique<render::SpacecraftScene>()) {}

	ToolbarAction draw(const std::function<void()>& replayBar) {
		ToolbarAction action = ToolbarAction::None;
		m_graphs.sample(m_app.parameters(), m_app.spacecraftSnapshot());
		const bool visible = ImGui::Begin(names::kSpacecraftWindow);
		m_focused = visible && ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows | ImGuiFocusedFlags_DockHierarchy);
		if (visible) {
			ToolbarOptions opt{ViewId::Spacecraft, "spacecraft_view", "spacecraft_run", true, true, true,
							   m_app.replayOnSpacecraft(), m_app.spacecraftSnapshot().timeS};
			action = drawToolbar(m_app, opt);
		}
		DockLayout::submitInner(DockLayout::spacecraftDockspaceId(), visible);
		ImGui::End();

		const SpacecraftBundle& sc = m_app.spacecraftSnapshot().spacecraft;
		if (ImGui::Begin(names::kScPanel)) {
			if (m_panel.popped) poppedOutPlaceholder(m_panel.popped);
			else drawSpacecraftValuesPanel(sc, m_panel);
		}
		ImGui::End();
		poppedOutWindow("Control Panel – Spacecraft values###ScPanelPopped", m_panel.popped, ImVec2(420, 860),
						[&] { drawSpacecraftValuesPanel(sc, m_panel); });

		const SimSnapshot& snap = m_app.spacecraftSnapshot();
		Recorder* rec = &m_app.recorder(ViewId::Spacecraft);
		auto body = [&] { m_scene.draw(snap, m_viewPopped, rec, replayBar); };
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
		const ImGuiWindowFlags flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
		if (ImGui::Begin(names::kScView, nullptr, flags)) {
			if (m_viewPopped) poppedOutPlaceholder(m_viewPopped);
			else body();
		}
		ImGui::End();
		poppedOutWindow("Spacecraft View###ScViewPopped", m_viewPopped, ImVec2(1100, 800), body, flags);
		ImGui::PopStyleVar();

		if (ImGui::Begin(names::kScGraphs)) m_graphs.draw(m_app.parameters());
		ImGui::End();
		m_graphs.drawPopOuts();
		return action;
	}

	bool focused() const { return m_focused; }

private:
	AppState&            m_app;
	SceneView            m_scene;
	GraphGrid            m_graphs{6, 3, "spacecraft"};
	SpacecraftPanelState m_panel;
	bool                 m_viewPopped = false, m_focused = false;
};

// Standalone floating analysis window hosting a 4x3 grid of 12 simultaneous telemetry graphs.
class AnalysisWindow {
public:
	explicit AnalysisWindow(AppState& app) : m_app(app) {}
	void open() { m_open = m_justOpened = true; }

	void draw() {
		m_graphs.sample(m_app.parameters(), m_app.analysisSnapshot());
		if (!m_open) return;

		const ImGuiViewport* vp = ImGui::GetMainViewport();
		ImGui::SetNextWindowSize(ImVec2(vp->Size.x * 0.85f, vp->Size.y * 0.8f), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + vp->Size.x * 0.5f, vp->Pos.y + vp->Size.y * 0.5f), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));
		if (m_justOpened) { ImGui::SetNextWindowFocus(); m_justOpened = false; }
		ImGuiWindowClass cls; cls.ViewportFlagsOverrideSet = ImGuiViewportFlags_NoAutoMerge;
		ImGui::SetNextWindowClass(&cls);

		if (ImGui::Begin(names::kAnalysis, &m_open)) {
			ImGui::AlignTextToFramePadding();
			mutedText(m_app.replayActive() ? "Graphs read from the replay run" : "Graphs read live UDP stream");
			if (smallButtonRight("Close")) m_open = false;
			m_graphs.draw(m_app.parameters());
		}
		ImGui::End();
		m_graphs.drawPopOuts();
	}

private:
	AppState& m_app;
	GraphGrid m_graphs{12, 4, "analysis"};
	bool      m_open = false, m_justOpened = false;
};

} // namespace

// ============================================================================
// MainUi Implementation
// ============================================================================

// Private state holding the dock layout, workspace windows, dialogs, and menu bar renderer.
struct MainUi::Impl {
	explicit Impl(AppState& app) : app(app), vde(app), spacecraft(app), analysis(app) {}

	AppState&        app;
	DockLayout       layout;
	VdeWindow        vde;
	SpacecraftWindow spacecraft;
	AnalysisWindow   analysis;
	ConfigXmlDialog  configDialog;
	ReplayDialog     replayDialog;
	std::string      osTitle = names::kVdeWindow;

	// Draws the top menu bar (`Simulation` & `View` menus) and right-aligned FPS + UDP packet/sequence counters.
	void drawMenuBar() {
		if (!ImGui::BeginMainMenuBar()) return;
		if (ImGui::BeginMenu("Simulation")) {
			if (ImGui::MenuItem("Edit XML Configuration...")) configDialog.open(app);
			if (ImGui::MenuItem("Initialize Run (Send RunConfig)")) app.submit(StartRunCmd{app.runConfig()});
			bool flipped = app.runConfig().frameMapping == FrameMapping::Flipped;
			if (ImGui::MenuItem("Flipped Frame Mapping", nullptr, &flipped)) {
				RunConfig cfg = app.runConfig();
				cfg.frameMapping = flipped ? FrameMapping::Flipped : FrameMapping::Default;
				app.submit(StartRunCmd{cfg});
			}
			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu("View")) {
			if (ImGui::MenuItem("Reset layout")) layout.requestReset();
			ImGui::EndMenu();
		}

		const ImGuiIO& io     = ImGui::GetIO();
		const ClientStats& cs = app.clientStats();
		char stats[192];
		std::snprintf(stats, sizeof(stats), "%.0f FPS (%.1f ms) · Engine UDP %s:%u / :%u · seq #%llu · pkts %llu",
					  io.Framerate, io.DeltaTime * 1000.0f, cs.host.c_str(),
					  (unsigned)cs.configPort, (unsigned)cs.statePort,
					  (unsigned long long)cs.lastSequence, (unsigned long long)cs.packetsReceived);
		ImGui::SameLine(ImGui::GetWindowWidth() - ImGui::CalcTextSize(stats).x - 16);
		ImGui::TextDisabled("%s", stats);
		ImGui::EndMainMenuBar();
	}
};

MainUi::MainUi(AppState& app) : m_impl(std::make_unique<Impl>(app)) {}
MainUi::~MainUi() = default;
const std::string& MainUi::osWindowTitle() const { return m_impl->osTitle; }

// Executes one full UI pass: menu bar, dockspaces, VDE & Spacecraft workspaces, Analysis window, and modal popups.
void MainUi::frame() {
	Impl& m = *m_impl;
	m.drawMenuBar();
	m.layout.beginFrame();
	if (m.layout.builtThisFrame()) m.vde.requestFocus();

	const auto replayBar = [&m] { drawReplayBar(m.app); };
	using BarFn = std::function<void()>;

	ToolbarAction vdeAction = m.vde.draw(m.app.replayOnVde() ? BarFn(replayBar) : BarFn());
	ToolbarAction scAction  = m.spacecraft.draw(m.app.replayOnSpacecraft() ? BarFn(replayBar) : BarFn());
	ToolbarAction action    = (vdeAction != ToolbarAction::None) ? vdeAction : scAction;

	switch (action) {
	case ToolbarAction::OpenAnalysis:  m.analysis.open(); break;
	case ToolbarAction::Replay:        m.replayDialog.open(); break;
	case ToolbarAction::InitialConfig: m.configDialog.open(m.app); break;
	case ToolbarAction::None:          break;
	}
	m.analysis.draw();
	m.configDialog.draw(m.app);
	m.replayDialog.draw(m.app);

	if (m.vde.focused()) m.osTitle = names::kVdeWindow;
	if (m.spacecraft.focused()) m.osTitle = names::kSpacecraftWindow;
}

} // namespace vde::ui
