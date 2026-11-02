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

#include <cstdio>
#include <functional>

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

// Modal popup displaying the active XML run configuration and allowing manual re-transmission over UDP.
class ConfigXmlDialog {
public:
	void open(const AppState& app) {
		m_title = "Initial configuration – " + app.configSourceName() + "###ConfigXmlDialog";
		m_xml = app.loadConfigXml();
		m_openRequested = true;
	}

	void draw(AppState& app) {
		if (m_openRequested) { ImGui::OpenPopup(m_title.c_str()); m_openRequested = false; }
		const ImGuiViewport* vp = ImGui::GetMainViewport();
		ImGui::SetNextWindowSize(ImVec2(vp->Size.x * 0.55f, vp->Size.y * 0.65f), ImGuiCond_Appearing);
		ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + vp->Size.x * 0.5f, vp->Pos.y + vp->Size.y * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
		if (!ImGui::BeginPopupModal(m_title.c_str(), nullptr, ImGuiWindowFlags_NoSavedSettings)) return;

		mutedText("RunConfig sent from VisualizationClient to DynamicsEngine over UDP port %u", app.clientStats().configPort);
		const float footer = ImGui::GetFrameHeightWithSpacing() + ImGui::GetStyle().ItemSpacing.y;
		ImGui::PushFont(fonts().mono, 0.0f);
		ImGui::PushStyleColor(ImGuiCol_FrameBg, IM_COL32(0xFA, 0xFA, 0xFA, 255));
		ImGui::InputTextMultiline("##xml", m_xml.data(), m_xml.size() + 1, ImVec2(-1, -footer), ImGuiInputTextFlags_ReadOnly);
		ImGui::PopStyleColor();
		ImGui::PopFont();

		const float dpi = ImGui::GetStyle().FontScaleDpi, sendW = 190.0f * dpi, closeW = 120.0f * dpi;
		ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - sendW - closeW - ImGui::GetStyle().ItemSpacing.x);
		if (ImGui::Button("RE-SEND RUNCONFIG (UDP)", ImVec2(sendW, 0))) app.submit(StartRunCmd{app.runConfig()});
		ImGui::SameLine();
		if (primaryButton("CLOSE", ImVec2(closeW, 0)) || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
		ImGui::EndPopup();
	}

private:
	std::string m_title, m_xml;
	bool        m_openRequested = false;
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

	void draw(const std::function<void()>& replayBar) {
		m_graphs.sample(m_app.parameters(), m_app.spacecraftSnapshot());
		const bool visible = ImGui::Begin(names::kSpacecraftWindow);
		m_focused = visible && ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows | ImGuiFocusedFlags_DockHierarchy);
		if (visible) {
			ToolbarOptions opt{ViewId::Spacecraft, "spacecraft_view", "spacecraft_run", false, false, false,
							   m_app.replayOnSpacecraft(), m_app.spacecraftSnapshot().timeS};
			drawToolbar(m_app, opt);
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

	switch (m.vde.draw(m.app.replayOnVde() ? BarFn(replayBar) : BarFn())) {
	case ToolbarAction::OpenAnalysis:  m.analysis.open(); break;
	case ToolbarAction::Replay:        m.replayDialog.open(); break;
	case ToolbarAction::InitialConfig: m.configDialog.open(m.app); break;
	case ToolbarAction::None:          break;
	}
	m.spacecraft.draw(m.app.replayOnSpacecraft() ? BarFn(replayBar) : BarFn());
	m.analysis.draw();
	m.configDialog.draw(m.app);
	m.replayDialog.draw(m.app);

	if (m.vde.focused()) m.osTitle = names::kVdeWindow;
	if (m.spacecraft.focused()) m.osTitle = names::kSpacecraftWindow;
}

} // namespace vde::ui
