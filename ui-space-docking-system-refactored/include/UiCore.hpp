// =============================================================================
// include/UiCore.hpp
// Declares reusable Dear ImGui / ImPlot UI primitives and layout components:
// window identifiers, color palette, DPI-scaled light theme and font loader,
// read-only telemetry display widgets, pop-out OS window helpers, programmatic
// multi-workspace DockLayout, status/action Toolbar, interactive 3D SceneView,
// and scrolling real-time telemetry GraphWidget / GraphGrid.
// =============================================================================
#pragma once
#include "Client.hpp"
#include "Renderer.hpp"

#include <imgui.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace vde::ui {

// Stable Dear ImGui window titles and internal IDs used to bind panes into DockLayout slots.
namespace names {
inline constexpr const char* kVdeWindow        = "VDE Docking Simulator – VDE";
inline constexpr const char* kSpacecraftWindow = "VDE Docking Simulator – Spacecraft";
inline constexpr const char* kVdePanel         = "Control Panel – VDE values";
inline constexpr const char* kVdeView          = "VDE View";
inline constexpr const char* kVdeGraphs        = "Graphs###VdeGraphs";
inline constexpr const char* kScPanel          = "Control Panel – Spacecraft values";
inline constexpr const char* kScView           = "Spacecraft View";
inline constexpr const char* kScGraphs         = "Graphs###SpacecraftGraphs";
inline constexpr const char* kAnalysis         = "VDE Docking Simulator – Analysis";
} // namespace names

// Shared RGBA color constants for buttons, read-only value boxes, badges, and XYZ axis hints.
namespace color {
inline constexpr ImU32 kBlue      = IM_COL32(0x3B, 0x8E, 0xE8, 255);
inline constexpr ImU32 kLightBlue = IM_COL32(0xA9, 0xCD, 0xF0, 255);
inline constexpr ImU32 kValueBg   = IM_COL32(0xE8, 0xE8, 0xE8, 255);
inline constexpr ImU32 kValueEdge = IM_COL32(0xCC, 0xCC, 0xCC, 255);
inline constexpr ImU32 kMuted     = IM_COL32(0x6E, 0x6E, 0x6E, 255);
inline constexpr ImU32 kOrange    = IM_COL32(0xF2, 0x8C, 0x28, 255);
inline constexpr ImU32 kRecordRed = IM_COL32(0xD9, 0x3D, 0x3D, 255);
inline constexpr ImU32 kAxisX     = IM_COL32(0xE0, 0x30, 0x30, 255);
inline constexpr ImU32 kAxisY     = IM_COL32(0x2E, 0xA0, 0x43, 255);
inline constexpr ImU32 kAxisZ     = IM_COL32(0x2F, 0x6F, 0xE0, 255);
} // namespace color

// Holds loaded proportional UI font and fixed-width monospace font handles.
struct Fonts {
	ImFont* ui   = nullptr;
	ImFont* mono = nullptr;
};

// Loads TrueType fonts and configures Dear ImGui + ImPlot light theme styles scaled for monitor DPI.
void setupTheme(float dpiScale);
const Fonts& fonts();

// Read-only telemetry display widgets used in the VDE and Spacecraft side panels.
void readOnlyText(const char* caption, const char* text);
void valueRow(const char* caption, const double* values, int count, const char* const* subLabels);
void valueRow(const char* caption, const Vec3& v);
void valueRow(const char* caption, const Rpy& v);
void valueRow(const char* caption, double v);
void sectionHeader(const char* text);
void mutedText(const char* fmt, ...);

// Styled button and status-badge helpers used across toolbars, headers, and modal dialogs.
bool toolbarButton(const char* label, bool active = false, ImU32 activeColor = 0);
bool primaryButton(const char* label, ImVec2 size = ImVec2(0, 0));
bool smallButtonRight(const char* label);
void statusTag(const char* text, ImU32 bg, ImU32 fg = IM_COL32_WHITE);

// Helpers that detach any docked pane or chart into its own OS viewport window and show a "Dock back" stub.
void poppedOutWindow(const char* title, bool& open, ImVec2 defaultSize,
					 const std::function<void()>& content, ImGuiWindowFlags extraFlags = 0);
void poppedOutPlaceholder(bool& open);

// Programmatically builds and resets the two-level Dear ImGui docking hierarchy:
// an outer tab bar switching between VDE and Spacecraft workspaces, each containing
// a left telemetry panel, a top-right 3D viewport, and a bottom-right live graph grid.
class DockLayout {
public:
	void beginFrame();
	void requestReset()         { m_resetRequested = true; }
	bool builtThisFrame() const { return m_builtThisFrame; }

	static ImGuiID vdeDockspaceId();
	static ImGuiID spacecraftDockspaceId();
	static void submitInner(ImGuiID id, bool visible);

private:
	static constexpr float kPanelWidth = 400.0f;
	static constexpr float kViewHeight = 640.0f;

	static void buildMain(ImGuiID id, const ImGuiViewport* vp);
	static void buildInner(ImGuiID id, ImVec2 size, const char* panel, const char* view, const char* graphs);

	bool m_resetRequested = false;
	bool m_builtThisFrame = false;
};

// User actions returned by the workspace toolbar that open top-level dialogs or floating windows.
enum class ToolbarAction { None, OpenAnalysis, Replay, InitialConfig };

// Per-workspace configuration passed into drawToolbar().
struct ToolbarOptions {
	ViewId      view         = ViewId::Vde;
	const char* recordTag    = "view";
	const char* exportPrefix = "run";
	bool        showAnalysis = false;
	bool        showReplay   = false;
	bool        showConfig   = false;
	bool        replayMode   = false;
	double      timeS        = 0.0;
};

// Renders the horizontal workspace action bar, simulation clock, recording indicator, and connection status.
ToolbarAction drawToolbar(AppState& app, const ToolbarOptions& opt);

// Embeds an off-screen OpenGL IScene inside a Dear ImGui pane with mouse camera controls,
// projected 2D coordinate/object labels, view-reset and pop-out buttons, and optional bottom bar.
class SceneView {
public:
	explicit SceneView(std::unique_ptr<render::IScene> scene);
	void draw(const SimSnapshot& snapshot, bool& popped, Recorder* recorder,
			  const std::function<void()>& bottomBar = {});

private:
	void handleInput(ImVec2 size);
	void drawLabels(ImVec2 origin, ImVec2 size, const glm::mat4& viewProj);
	void drawHint(ImVec2 bottomLeft);
	void pushRecorderFrame(Recorder* recorder);

	std::unique_ptr<render::IScene>  m_scene;
	std::unique_ptr<render::Painter> m_painter;
	render::Framebuffer              m_fbo;
	render::OrbitCamera              m_camera;
	std::vector<std::uint8_t>        m_pixels;
	bool                             m_darkBackground = false;
};

// Single scrolling 30-second time-series chart backed by a fixed-capacity ring buffer,
// searchable parameter selector combo box, and optional detached pop-out window.
class GraphWidget {
public:
	GraphWidget(int number, std::string scope);
	void sample(const ParameterRegistry& registry, const SimSnapshot& snapshot);
	void draw(const ParameterRegistry& registry);
	void drawPopOut();

private:
	void drawHeader(const ParameterRegistry& registry);
	void drawPlot(const char* id, bool compact);

	static constexpr int    kCapacity = 8192;
	static constexpr double kWindowS  = 30.0;

	int                 m_number;
	std::string         m_scope, m_param, m_label;
	std::vector<double> m_times, m_values;
	int                 m_offset  = 0;
	double              m_now     = 0.0;
	bool                m_hasTime = false;
	bool                m_popped  = false;
	char                m_filter[64] = {};
};

// Arranges `count` independent GraphWidget instances into an evenly sized multi-column grid.
class GraphGrid {
public:
	GraphGrid(int count, int columns, const char* scope);
	void sample(const ParameterRegistry& registry, const SimSnapshot& snapshot);
	void draw(const ParameterRegistry& registry);
	void drawPopOuts();

private:
	int m_columns;
	std::vector<GraphWidget> m_graphs;
};

} // namespace vde::ui
