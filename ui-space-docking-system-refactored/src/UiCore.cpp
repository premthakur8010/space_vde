// =============================================================================
// src/UiCore.cpp
// Implements TrueType font loading, DPI-aware light theme styling, read-only
// numeric/text boxes, detached pop-out windows, DockBuilder workspace layout,
// workspace Toolbar, 3D SceneView viewport widget (off-screen FBO + 2D projected
// leader-line overlays), and scrolling ImPlot GraphWidget / GraphGrid charts.
// =============================================================================
#include "UiCore.hpp"

#include <imgui_internal.h>
#include <implot.h>

#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstdarg>
#include <cstdio>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace vde::ui {

namespace {
Fonts g_fonts;

// Resolves the directory containing the running executable so bundled fonts can be found regardless of CWD.
std::string exeDir() {
	char buf[4096] = {};
#ifdef _WIN32
	GetModuleFileNameA(nullptr, buf, sizeof(buf) - 1);
#else
	if (readlink("/proc/self/exe", buf, sizeof(buf) - 1) < 0) return ".";
#endif
	std::string p(buf);
	std::size_t slash = p.find_last_of("/\\");
	return slash == std::string::npos ? "." : p.substr(0, slash);
}

// Loads a TrueType font from `<exe>/fonts/` first, falling back to the source-tree font directory.
ImFont* loadFont(const char* file, float size) {
	const std::string candidates[] = {exeDir() + "/fonts/" + file, std::string(VDE_FONT_DIR) + "/" + file};
	for (const std::string& path : candidates) {
		if (FILE* f = std::fopen(path.c_str(), "rb")) {
			std::fclose(f);
			return ImGui::GetIO().Fonts->AddFontFromFileTTF(path.c_str(), size);
		}
	}
	return nullptr;
}

ImVec4 rgb(int r, int g, int b, float a = 1.0f) { return ImVec4(r / 255.f, g / 255.f, b / 255.f, a); }

// Draws the subtle filled rectangle and border used by read-only telemetry cells.
void drawBox(ImDrawList* dl, ImVec2 min, ImVec2 max) {
	dl->AddRectFilled(min, max, color::kValueBg, 2.0f);
	dl->AddRect(min, max, color::kValueEdge, 2.0f);
}

void drawCaption(const char* text) {
	ImGui::PushStyleColor(ImGuiCol_Text, color::kMuted);
	ImGui::TextUnformatted(text);
	ImGui::PopStyleColor();
}

// Case-insensitive substring filter used inside the graph parameter combo box.
bool containsNoCase(const std::string& text, const char* needle) {
	if (!needle || !*needle) return true;
	auto it = std::search(text.begin(), text.end(), needle, needle + std::char_traits<char>::length(needle),
						  [](char a, char b) { return std::tolower((unsigned char)a) == std::tolower((unsigned char)b); });
	return it != text.end();
}
} // namespace

// ============================================================================
// Theme, Read-Only Value Widgets & Pop-Out Helpers
// ============================================================================

const Fonts& fonts() { return g_fonts; }

// Loads Roboto (UI) and Cousine (monospace) fonts and applies a high-contrast light engineering theme.
void setupTheme(float dpiScale) {
	g_fonts.ui   = loadFont("Roboto-Medium.ttf", 15.0f);
	g_fonts.mono = loadFont("Cousine-Regular.ttf", 14.0f);
	if (!g_fonts.ui) g_fonts.ui = ImGui::GetIO().Fonts->AddFontDefault();
	if (!g_fonts.mono) g_fonts.mono = g_fonts.ui;

	ImGui::StyleColorsLight();
	ImGuiStyle& s = ImGui::GetStyle();
	s.WindowPadding = ImVec2(10, 8); s.FramePadding = ImVec2(8, 4); s.ItemSpacing = ImVec2(8, 6);
	s.WindowRounding = 0.0f; s.FrameRounding = s.TabRounding = s.GrabRounding = 2.0f;
	s.WindowBorderSize = s.FrameBorderSize = 1.0f; s.ScrollbarSize = 12.0f; s.DockingSeparatorSize = 3.0f;

	ImVec4* c = s.Colors;
	c[ImGuiCol_Text]                = rgb(0x22, 0x22, 0x22);
	c[ImGuiCol_TextDisabled]        = rgb(0x8A, 0x8A, 0x8A);
	c[ImGuiCol_WindowBg]            = rgb(0xF0, 0xF0, 0xF0);
	c[ImGuiCol_ChildBg]             = rgb(0xF0, 0xF0, 0xF0, 0.0f);
	c[ImGuiCol_PopupBg]             = rgb(0xFA, 0xFA, 0xFA);
	c[ImGuiCol_Border]              = rgb(0xC8, 0xC8, 0xC8);
	c[ImGuiCol_FrameBg]             = rgb(0xFF, 0xFF, 0xFF);
	c[ImGuiCol_FrameBgHovered]      = rgb(0xE6, 0xF0, 0xFB);
	c[ImGuiCol_FrameBgActive]       = rgb(0xD4, 0xE6, 0xF8);
	c[ImGuiCol_TitleBg]             = rgb(0xD8, 0xD8, 0xD8);
	c[ImGuiCol_TitleBgActive]       = rgb(0xC6, 0xDB, 0xF2);
	c[ImGuiCol_TitleBgCollapsed]    = rgb(0xD8, 0xD8, 0xD8);
	c[ImGuiCol_MenuBarBg]           = rgb(0xE2, 0xE2, 0xE2);
	c[ImGuiCol_Button]              = rgb(0xA9, 0xCD, 0xF0);
	c[ImGuiCol_ButtonHovered]       = rgb(0x84, 0xB8, 0xEC);
	c[ImGuiCol_ButtonActive]        = rgb(0x3B, 0x8E, 0xE8);
	c[ImGuiCol_Header]              = rgb(0xA9, 0xCD, 0xF0, 0.7f);
	c[ImGuiCol_HeaderHovered]       = rgb(0xA9, 0xCD, 0xF0);
	c[ImGuiCol_HeaderActive]        = rgb(0x3B, 0x8E, 0xE8, 0.8f);
	c[ImGuiCol_CheckMark]           = rgb(0x3B, 0x8E, 0xE8);
	c[ImGuiCol_SliderGrab]          = rgb(0x3B, 0x8E, 0xE8);
	c[ImGuiCol_SliderGrabActive]    = rgb(0x2A, 0x70, 0xC0);
	c[ImGuiCol_Separator]           = rgb(0xC8, 0xC8, 0xC8);
	c[ImGuiCol_SeparatorHovered]    = rgb(0x3B, 0x8E, 0xE8, 0.7f);
	c[ImGuiCol_SeparatorActive]     = rgb(0x3B, 0x8E, 0xE8);
	c[ImGuiCol_Tab]                 = rgb(0xDC, 0xDC, 0xDC);
	c[ImGuiCol_TabHovered]          = rgb(0xA9, 0xCD, 0xF0);
	c[ImGuiCol_TabSelected]         = rgb(0xF0, 0xF0, 0xF0);
	c[ImGuiCol_TabSelectedOverline] = rgb(0x3B, 0x8E, 0xE8);
	c[ImGuiCol_TabDimmed]           = rgb(0xDC, 0xDC, 0xDC);
	c[ImGuiCol_TabDimmedSelected]   = rgb(0xEA, 0xEA, 0xEA);
	c[ImGuiCol_DockingPreview]      = rgb(0x3B, 0x8E, 0xE8, 0.5f);
	c[ImGuiCol_DockingEmptyBg]      = rgb(0xE4, 0xE4, 0xE4);
	c[ImGuiCol_ResizeGrip]          = rgb(0xA9, 0xCD, 0xF0, 0.4f);
	c[ImGuiCol_ResizeGripHovered]   = rgb(0x3B, 0x8E, 0xE8, 0.7f);
	c[ImGuiCol_ResizeGripActive]    = rgb(0x3B, 0x8E, 0xE8);
	c[ImGuiCol_ModalWindowDimBg]    = rgb(0x30, 0x30, 0x30, 0.35f);

	s.ScaleAllSizes(dpiScale);
	s.FontScaleDpi = dpiScale;

	ImPlotStyle& ps = ImPlot::GetStyle();
	ImPlot::StyleColorsLight(&ps);
	ps.PlotPadding = ImVec2(6, 6); ps.LabelPadding = ImVec2(4, 2);
	ps.Colors[ImPlotCol_FrameBg] = rgb(0xFF, 0xFF, 0xFF, 0.0f);
	ps.Colors[ImPlotCol_PlotBg]  = rgb(0xFF, 0xFF, 0xFF);
}

// Draws a full-width non-editable text field preceded by an optional muted caption.
void readOnlyText(const char* cap, const char* text) {
	if (cap && *cap) drawCaption(cap);
	ImDrawList* dl = ImGui::GetWindowDrawList();
	const ImVec2 p = ImGui::GetCursorScreenPos(), size(ImGui::GetContentRegionAvail().x, ImGui::GetFrameHeight());
	drawBox(dl, p, ImVec2(p.x + size.x, p.y + size.y));
	const ImGuiStyle& st = ImGui::GetStyle();
	dl->PushClipRect(p, ImVec2(p.x + size.x - st.FramePadding.x, p.y + size.y), true);
	dl->AddText(ImVec2(p.x + st.FramePadding.x, p.y + st.FramePadding.y), ImGui::GetColorU32(ImGuiCol_Text), text);
	dl->PopClipRect();
	ImGui::Dummy(size);
}

// Draws `count` side-by-side right-aligned numeric cells formatted to 3 decimal places with optional sub-labels.
void valueRow(const char* cap, const double* values, int count, const char* const* subLabels) {
	if (cap && *cap) drawCaption(cap);
	ImDrawList* dl = ImGui::GetWindowDrawList();
	const ImGuiStyle& st = ImGui::GetStyle();
	const ImVec2 p = ImGui::GetCursorScreenPos();
	const float avail = ImGui::GetContentRegionAvail().x, gap = st.ItemSpacing.x;
	const float w = (avail - gap * (count - 1)) / count, h = ImGui::GetFrameHeight();
	const float labelH = subLabels ? ImGui::GetTextLineHeight() : 0.0f;

	for (int i = 0; i < count; ++i) {
		const ImVec2 min(p.x + i * (w + gap), p.y), max(min.x + w, min.y + h);
		drawBox(dl, min, max);
		char buf[48];
		std::snprintf(buf, sizeof(buf), "%.3f", values[i]);
		const ImVec2 ts = ImGui::CalcTextSize(buf);
		dl->PushClipRect(min, max, true);
		dl->AddText(ImVec2(max.x - st.FramePadding.x - ts.x, min.y + st.FramePadding.y), ImGui::GetColorU32(ImGuiCol_Text), buf);
		dl->PopClipRect();
		if (subLabels) {
			const ImVec2 ls = ImGui::CalcTextSize(subLabels[i]);
			dl->AddText(ImVec2(min.x + (w - ls.x) * 0.5f, max.y + 1.0f), color::kMuted, subLabels[i]);
		}
	}
	ImGui::Dummy(ImVec2(avail, h + labelH + 1.0f));
}

void valueRow(const char* cap, const Vec3& v) {
	static const char* const kXyz[] = {"x", "y", "z"};
	const double vals[] = {v.x, v.y, v.z};
	valueRow(cap, vals, 3, kXyz);
}
void valueRow(const char* cap, const Rpy& v) {
	static const char* const kRpy[] = {"roll", "pitch", "yaw"};
	const double vals[] = {v.roll, v.pitch, v.yaw};
	valueRow(cap, vals, 3, kRpy);
}
void valueRow(const char* cap, double v)      { valueRow(cap, &v, 1, nullptr); }
void sectionHeader(const char* text)          { ImGui::Spacing(); ImGui::SeparatorText(text); }

void mutedText(const char* fmt, ...) {
	va_list args; va_start(args, fmt);
	ImGui::PushStyleColor(ImGuiCol_Text, color::kMuted);
	ImGui::TextV(fmt, args);
	ImGui::PopStyleColor();
	va_end(args);
}

bool toolbarButton(const char* label, bool active, ImU32 activeColor) {
	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(16, 8));
	ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
	if (active) {
		ImGui::PushStyleColor(ImGuiCol_Button, activeColor);
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, activeColor);
		ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32_WHITE);
	}
	const bool pressed = ImGui::Button(label);
	if (active) ImGui::PopStyleColor(3);
	ImGui::PopStyleVar(2);
	return pressed;
}

bool primaryButton(const char* label, ImVec2 size) {
	ImGui::PushStyleColor(ImGuiCol_Button, color::kBlue);
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(0x2F, 0x7D, 0xD6, 255));
	ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(0x25, 0x68, 0xB8, 255));
	ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32_WHITE);
	const bool pressed = ImGui::Button(label, size);
	ImGui::PopStyleColor(4);
	return pressed;
}

bool smallButtonRight(const char* label) {
	const float w = ImGui::CalcTextSize(label, nullptr, true).x + ImGui::GetStyle().FramePadding.x * 2.0f;
	ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - w);
	ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32_WHITE);
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, color::kLightBlue);
	const bool pressed = ImGui::SmallButton(label);
	ImGui::PopStyleColor(2);
	return pressed;
}

void statusTag(const char* text, ImU32 bg, ImU32 fg) {
	const ImGuiStyle& st = ImGui::GetStyle();
	const ImVec2 ts = ImGui::CalcTextSize(text), pad(st.FramePadding.x, st.FramePadding.y * 0.5f);
	const ImVec2 p = ImGui::GetCursorScreenPos(), size(ts.x + pad.x * 2, ts.y + pad.y * 2);
	ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), bg, 3.0f);
	ImGui::GetWindowDrawList()->AddText(ImVec2(p.x + pad.x, p.y + pad.y), fg, text);
	ImGui::Dummy(size);
}

// Opens a standalone non-docking window on its own OS viewport so the user can drag it onto secondary monitors.
void poppedOutWindow(const char* title, bool& open, ImVec2 defaultSize,
					 const std::function<void()>& content, ImGuiWindowFlags extraFlags) {
	if (!open) return;
#ifndef __linux__
	ImGuiWindowClass cls; cls.ViewportFlagsOverrideSet = ImGuiViewportFlags_NoAutoMerge;
	ImGui::SetNextWindowClass(&cls);
#endif
	const ImGuiViewport* vp = ImGui::GetMainViewport();
	ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + vp->Size.x * 0.5f, vp->Pos.y + vp->Size.y * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
	ImGui::SetNextWindowSize(defaultSize, ImGuiCond_Appearing);
	if (ImGui::Begin(title, &open, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoCollapse | extraFlags)) content();
	ImGui::End();
}

// Renders a centred placeholder inside the original dock slot when a panel or 3D view is popped out.
void poppedOutPlaceholder(bool& open) {
	const ImVec2 avail = ImGui::GetContentRegionAvail(), start = ImGui::GetCursorPos();
	const char* msg = "Shown in a separate window";
	const ImVec2 ts = ImGui::CalcTextSize(msg);
	ImGui::SetCursorPos(ImVec2(start.x + (avail.x - ts.x) * 0.5f, start.y + avail.y * 0.45f - ts.y));
	ImGui::PushStyleColor(ImGuiCol_Text, color::kMuted);
	ImGui::TextUnformatted(msg);
	ImGui::PopStyleColor();
	const char* btn = "Dock back";
	const float bw = ImGui::CalcTextSize(btn).x + ImGui::GetStyle().FramePadding.x * 2;
	ImGui::SetCursorPosX(start.x + (avail.x - bw) * 0.5f);
	if (ImGui::Button(btn)) open = false;
}

// ============================================================================
// DockLayout & Toolbar
// ============================================================================

ImGuiID DockLayout::vdeDockspaceId()        { return ImHashStr("VdeInnerDock_v2"); }
ImGuiID DockLayout::spacecraftDockspaceId() { return ImHashStr("ScInnerDock_v2"); }
void DockLayout::submitInner(ImGuiID id, bool visible) {
	ImGui::DockSpace(id, ImVec2(0, 0), visible ? ImGuiDockNodeFlags_None : ImGuiDockNodeFlags_KeepAliveOnly);
}

// Initializes or rebuilds the outer tab dockspace and both inner 3-pane workspace dockspaces on first frame or reset.
void DockLayout::beginFrame() {
	const ImGuiViewport* vp = ImGui::GetMainViewport();
	const ImGuiID mainId = ImHashStr("VdeMainDock_v2");
	m_builtThisFrame = false;

	if (ImGui::DockBuilderGetNode(mainId) == nullptr || m_resetRequested) {
		buildMain(mainId, vp);
		m_builtThisFrame = true;
	}
	const ImVec2 inner(vp->WorkSize.x - 20.0f, vp->WorkSize.y - 90.0f);
	if (m_builtThisFrame || !ImGui::DockBuilderGetNode(vdeDockspaceId()))
		buildInner(vdeDockspaceId(), inner, names::kVdePanel, names::kVdeView, names::kVdeGraphs);
	if (m_builtThisFrame || !ImGui::DockBuilderGetNode(spacecraftDockspaceId()))
		buildInner(spacecraftDockspaceId(), inner, names::kScPanel, names::kScView, names::kScGraphs);
	m_resetRequested = false;

	ImGui::DockSpaceOverViewport(mainId, vp);
}

void DockLayout::buildMain(ImGuiID id, const ImGuiViewport* vp) {
	ImGui::DockBuilderRemoveNode(id);
	ImGui::DockBuilderAddNode(id, ImGuiDockNodeFlags_DockSpace);
	ImGui::DockBuilderSetNodeSize(id, vp->WorkSize);
	ImGui::DockBuilderDockWindow(names::kVdeWindow, id);
	ImGui::DockBuilderDockWindow(names::kSpacecraftWindow, id);
	ImGui::DockBuilderFinish(id);
}

// Splits a workspace dockspace into a left control column and a right column divided into 3D view (top) and graphs (bottom).
void DockLayout::buildInner(ImGuiID id, ImVec2 size, const char* panel, const char* view, const char* graphs) {
	ImGui::DockBuilderRemoveNode(id);
	ImGui::DockBuilderAddNode(id, ImGuiDockNodeFlags_DockSpace);
	ImGui::DockBuilderSetNodeSize(id, size);
	const float dpi = ImGui::GetStyle().FontScaleDpi;
	ImGuiID left = 0, right = 0, top = 0, bottom = 0;
	ImGui::DockBuilderSplitNode(id, ImGuiDir_Left, std::min(kPanelWidth * dpi / size.x, 0.4f), &left, &right);
	ImGui::DockBuilderSplitNode(right, ImGuiDir_Up, std::min(kViewHeight * dpi / size.y, 0.55f), &top, &bottom);
	ImGui::DockBuilderDockWindow(panel, left);
	ImGui::DockBuilderDockWindow(view, top);
	ImGui::DockBuilderDockWindow(graphs, bottom);
	ImGui::DockBuilderFinish(id);
}

// Draws action buttons on the left and right-aligns the T+ clock, 120 Hz rate indicator, REC badge, and LIVE/WAITING status.
ToolbarAction drawToolbar(AppState& app, const ToolbarOptions& opt) {
	ToolbarAction action = ToolbarAction::None;
	ImGui::Dummy(ImVec2(0, 2));

	const bool recording = app.isRecording(opt.view);
	if (toolbarButton(recording ? "STOP RECORDING" : "RECORD VIDEO", recording, color::kRecordRed))
		app.submit(ToggleRecordingCmd{opt.view, opt.recordTag});
	ImGui::SameLine();
	if (toolbarButton("EXPORT DATA")) app.submit(ExportDataCmd{opt.exportPrefix});
	if (opt.showAnalysis) { ImGui::SameLine(); if (toolbarButton("OPEN ANALYSIS")) action = ToolbarAction::OpenAnalysis; }
	if (opt.showReplay)   { ImGui::SameLine(); if (toolbarButton("REPLAY")) action = ToolbarAction::Replay; }
	if (opt.showConfig)   { ImGui::SameLine(); if (toolbarButton("INITIAL CONFIG (XML)")) action = ToolbarAction::InitialConfig; }
	const float rowY = ImGui::GetItemRectMin().y, rowH = ImGui::GetItemRectSize().y;

	char clock[64];
	std::snprintf(clock, sizeof(clock), "T+ %s · %.0f Hz", formatSimTime(opt.timeS).c_str(), kUiRateHz);
	const char* status = opt.replayMode ? "REPLAY MODE" : (app.clientStats().hasState ? "Status: LIVE" : "Status: WAITING FOR ENGINE");
	const ImGuiStyle& st = ImGui::GetStyle();

	ImGui::PushFont(g_fonts.mono, 0.0f);
	const float clockW = ImGui::CalcTextSize(clock).x;
	ImGui::PopFont();
	const float recW = recording ? ImGui::CalcTextSize("REC").x + st.FramePadding.x * 2 + st.ItemSpacing.x * 2 : 0.0f;
	const float statusW = ImGui::CalcTextSize(status).x + (opt.replayMode ? st.FramePadding.x * 2 : 0.0f);

	ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - (clockW + recW + st.ItemSpacing.x * 3 + statusW));
	const float textY = rowY + (rowH - ImGui::GetTextLineHeight()) * 0.5f;
	ImGui::SetCursorScreenPos(ImVec2(ImGui::GetCursorScreenPos().x, textY));
	ImGui::PushFont(g_fonts.mono, 0.0f);
	ImGui::TextUnformatted(clock);
	ImGui::PopFont();
	if (recording) {
		ImGui::SameLine(0, st.ItemSpacing.x * 2);
		ImGui::SetCursorScreenPos(ImVec2(ImGui::GetCursorScreenPos().x, textY - st.FramePadding.y * 0.5f));
		statusTag("REC", color::kRecordRed);
	}
	ImGui::SameLine(0, st.ItemSpacing.x * 3);
	if (opt.replayMode) {
		ImGui::SetCursorScreenPos(ImVec2(ImGui::GetCursorScreenPos().x, textY - st.FramePadding.y * 0.5f));
		statusTag(status, color::kOrange);
	} else {
		ImGui::SetCursorScreenPos(ImVec2(ImGui::GetCursorScreenPos().x, textY));
		ImGui::TextUnformatted(status);
	}
	ImGui::SetCursorScreenPos(ImVec2(ImGui::GetWindowPos().x + st.WindowPadding.x, rowY + rowH + st.ItemSpacing.y));
	ImGui::Dummy(ImVec2(0, 0));
	return action;
}

// ============================================================================
// SceneView (Interactive 3D Viewport Widget)
// ============================================================================

SceneView::SceneView(std::unique_ptr<render::IScene> scene)
	: m_scene(std::move(scene)), m_painter(std::make_unique<render::Painter>()) {
	m_camera = m_scene->defaultCamera();
	const glm::vec3 bg = m_scene->clearColor();
	m_darkBackground = (bg.r + bg.g + bg.b) < 1.5f;
}

// Renders `m_scene` into the 4x MSAA off-screen FBO, displays the resolved texture inside ImGui,
// captures mouse orbit/pan/zoom input, overlays 2D projected labels, and draws HUD controls.
void SceneView::draw(const SimSnapshot& snapshot, bool& popped, Recorder* recorder,
					 const std::function<void()>& bottomBar) {
	const ImVec2 size = ImGui::GetContentRegionAvail();
	if (size.x < 16.0f || size.y < 16.0f) return;
	const ImVec2 origin = ImGui::GetCursorScreenPos();

	float fbScale = ImGui::GetWindowViewport()->FramebufferScale.x;
	if (fbScale <= 0.0f) fbScale = 1.0f;
	m_fbo.resize((int)(size.x * fbScale), (int)(size.y * fbScale));
	m_fbo.bind();
	const glm::vec3 bg = m_scene->clearColor();
	glClearColor(bg.r, bg.g, bg.b, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	const glm::mat4 view = m_camera.view(), proj = m_camera.projection(size.x / size.y);
	m_painter->begin(view, proj, m_camera.position());
	m_scene->render(*m_painter, snapshot);
	m_fbo.resolve();
	pushRecorderFrame(recorder);

	ImGui::Image((ImTextureID)(std::intptr_t)m_fbo.texture(), size, ImVec2(0, 1), ImVec2(1, 0));
	ImGui::SetCursorScreenPos(origin);
	ImGui::SetNextItemAllowOverlap();
	ImGui::InvisibleButton("##sceneInput", size,
						   ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
	handleInput(size);
	drawLabels(origin, size, proj * view);

	const ImGuiStyle& st = ImGui::GetStyle();
	const float pad = 8.0f;
	const char* popLabel = popped ? "Dock back" : "Pop out";
	const float popW = ImGui::CalcTextSize(popLabel).x + st.FramePadding.x * 2;
	const float resetW = ImGui::CalcTextSize("Reset view").x + st.FramePadding.x * 2;
	ImGui::SetCursorScreenPos(ImVec2(origin.x + size.x - popW - resetW - st.ItemSpacing.x - pad, origin.y + pad));
	ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(255, 255, 255, 230));
	if (ImGui::Button("Reset view")) m_camera = m_scene->defaultCamera();
	ImGui::SameLine();
	if (ImGui::Button(popLabel)) popped = !popped;
	ImGui::PopStyleColor();

	float bottom = origin.y + size.y;
	if (bottomBar) {
		bottom -= ImGui::GetFrameHeight() + st.WindowPadding.y * 2;
		ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(origin.x, bottom), ImVec2(origin.x + size.x, origin.y + size.y), IM_COL32(250, 250, 250, 235));
		ImGui::GetWindowDrawList()->AddLine(ImVec2(origin.x, bottom), ImVec2(origin.x + size.x, bottom), color::kOrange, 2.0f);
		ImGui::SetCursorScreenPos(ImVec2(origin.x + st.WindowPadding.x, bottom + st.WindowPadding.y));
		ImGui::PushID("bottomBar"); bottomBar(); ImGui::PopID();
	}
	drawHint(ImVec2(origin.x + pad, bottom - pad));
}

// Translates mouse wheel steps and LMB/RMB/MMB drags over the viewport into OrbitCamera updates.
void SceneView::handleInput(ImVec2 size) {
	const ImGuiIO& io = ImGui::GetIO();
	if (ImGui::IsItemHovered() && !nearlyEqual(io.MouseWheel, 0.0f, 1e-6f)) m_camera.zoom(io.MouseWheel);
	if (!ImGui::IsItemActive()) return;
	if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) m_camera.rotate(io.MouseDelta.x, io.MouseDelta.y);
	else if (ImGui::IsMouseDown(ImGuiMouseButton_Right) || ImGui::IsMouseDown(ImGuiMouseButton_Middle))
		m_camera.pan(io.MouseDelta.x, io.MouseDelta.y, size.y);
}

// Projects 3D world-space label anchors through `viewProj` into 2D screen coordinates and renders
// pill badges for coordinate frames and horizontal leader lines for hardware/spacecraft callouts.
void SceneView::drawLabels(ImVec2 origin, ImVec2 size, const glm::mat4& viewProj) {
	ImDrawList* dl = ImGui::GetWindowDrawList();
	dl->PushClipRect(origin, ImVec2(origin.x + size.x, origin.y + size.y), true);
	for (const render::Label& l : m_painter->labels()) {
		const glm::vec4 clip = viewProj * glm::vec4(l.world, 1.0f);
		if (clip.w <= 0.01f) continue;
		const glm::vec3 ndc = glm::vec3(clip) / clip.w;
		const ImVec2 at(origin.x + (ndc.x * 0.5f + 0.5f) * size.x, origin.y + (0.5f - ndc.y * 0.5f) * size.y);
		const ImVec2 ts = ImGui::CalcTextSize(l.text.c_str());
		if (l.kind == render::LabelKind::Frame) {
			const ImVec2 p(at.x + 6.0f, at.y + 6.0f);
			dl->AddRectFilled(ImVec2(p.x - 3, p.y - 1), ImVec2(p.x + ts.x + 3, p.y + ts.y + 1),
							  m_darkBackground ? IM_COL32(20, 24, 36, 200) : IM_COL32(255, 255, 255, 210), 3.0f);
			dl->AddText(p, m_darkBackground ? IM_COL32(235, 238, 245, 255) : IM_COL32(30, 30, 30, 255), l.text.c_str());
		} else {
			const ImU32 ink = m_darkBackground ? IM_COL32(200, 210, 230, 255) : IM_COL32(70, 74, 82, 255);
			const ImVec2 end(at.x + l.side * 70.0f * ImGui::GetStyle().FontScaleDpi, at.y);
			dl->AddCircleFilled(at, 2.5f, ink);
			dl->AddLine(at, end, ink, 1.0f);
			dl->AddText(ImVec2(l.side > 0 ? end.x + 4.0f : end.x - 4.0f - ts.x, end.y - ts.y * 0.5f), ink, l.text.c_str());
		}
	}
	dl->PopClipRect();
}

// Draws the bottom-left mouse navigation legend and color-coded X / Y / Z axis key.
void SceneView::drawHint(ImVec2 bottomLeft) {
	ImDrawList* dl = ImGui::GetWindowDrawList();
	const char* text = "Mouse: LMB rotate · RMB pan · wheel zoom  ";
	const ImVec2 ts = ImGui::CalcTextSize(text), pad(8, 4);
	const ImVec2 min(bottomLeft.x, bottomLeft.y - ts.y - pad.y * 2);
	const ImVec2 max(min.x + ts.x + ImGui::CalcTextSize("X Y Z").x + pad.x * 2, bottomLeft.y);
	dl->AddRectFilled(min, max, IM_COL32(255, 255, 255, 220), 3.0f);
	dl->AddRect(min, max, color::kValueEdge, 3.0f);
	ImVec2 p(min.x + pad.x, min.y + pad.y);
	dl->AddText(p, IM_COL32(40, 40, 40, 255), text); p.x += ts.x;
	const ImU32 cols[3] = {color::kAxisX, color::kAxisY, color::kAxisZ};
	const char* letters[3] = {"X ", "Y ", "Z"};
	for (int i = 0; i < 3; ++i) { dl->AddText(p, cols[i], letters[i]); p.x += ImGui::CalcTextSize(letters[i]).x; }
}

void SceneView::pushRecorderFrame(Recorder* recorder) {
	if (!recorder || !recorder->isRecording() || !recorder->wantsFrames()) return;
	m_pixels.resize((std::size_t)m_fbo.width() * m_fbo.height() * 4);
	m_fbo.readPixels(m_pixels.data());
	recorder->pushFrame(m_pixels.data(), m_fbo.width(), m_fbo.height());
}

// ============================================================================
// GraphWidget & GraphGrid (Live Scrolling Time-Series Plots)
// ============================================================================

GraphWidget::GraphWidget(int number, std::string scope) : m_number(number), m_scope(std::move(scope)) {
	m_times.reserve(kCapacity);
	m_values.reserve(kCapacity);
}

// Appends a new `(time, value)` sample whenever simulation time advances, resetting the ring buffer if time rewinds.
void GraphWidget::sample(const ParameterRegistry& registry, const SimSnapshot& snapshot) {
	const double t = snapshot.timeS;
	if (m_hasTime && t < m_now) { m_times.clear(); m_values.clear(); m_offset = 0; }
	const bool advanced = !m_hasTime || t > m_now;
	m_now = t; m_hasTime = true;
	if (!m_param.empty() && advanced) {
		const double v = registry.value(m_param, snapshot);
		if ((int)m_times.size() < kCapacity) { m_times.push_back(t); m_values.push_back(v); }
		else { m_times[m_offset] = t; m_values[m_offset] = v; m_offset = (m_offset + 1) % kCapacity; }
	}
}

void GraphWidget::draw(const ParameterRegistry& registry) {
	ImGui::PushID(m_scope.c_str()); ImGui::PushID(m_number);
	drawHeader(registry);
	drawPlot("##plot", true);
	ImGui::PopID(); ImGui::PopID();
}

// Renders an enlarged full-axis version of this chart in its own OS window when popped out.
void GraphWidget::drawPopOut() {
	if (!m_popped) return;
	const ImGuiViewport* vp = ImGui::GetMainViewport();
	const std::string title = "Graph " + std::to_string(m_number) + " – " +
							  (m_param.empty() ? std::string("[select parameter]") : m_label) +
							  "###GraphPop_" + m_scope + std::to_string(m_number);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(4, 4));
	poppedOutWindow(title.c_str(), m_popped, ImVec2(vp->Size.x * 0.75f, vp->Size.y * 0.7f), [&] {
		ImGui::PushID(m_scope.c_str()); ImGui::PushID(m_number);
		drawPlot("##bigPlot", false);
		ImGui::PopID(); ImGui::PopID();
	});
	ImGui::PopStyleVar();
}

// Draws the graph title, searchable parameter combo box, and Pop out / Close pop out toggle button.
void GraphWidget::drawHeader(const ParameterRegistry& registry) {
	ImGui::AlignTextToFramePadding();
	ImGui::Text("Graph %d", m_number);
	ImGui::SameLine();
	const char* popLabel = m_popped ? "Close pop out" : "Pop out";
	const float popW = ImGui::CalcTextSize(popLabel).x + ImGui::GetStyle().FramePadding.x * 2;
	ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - popW - ImGui::GetStyle().ItemSpacing.x);
	ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32_WHITE);
	if (ImGui::BeginCombo("##param", m_param.empty() ? "[select parameter]" : m_label.c_str(), ImGuiComboFlags_HeightLarge)) {
		if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
		ImGui::SetNextItemWidth(-1);
		ImGui::InputTextWithHint("##filter", "filter parameters…", m_filter, sizeof(m_filter));
		if (ImGui::Selectable("[select parameter]", m_param.empty())) { m_param.clear(); m_times.clear(); m_values.clear(); m_offset = 0; }
		for (const ParameterInfo& p : registry.parameters()) {
			const std::string label = p.label();
			if (!containsNoCase(label, m_filter)) continue;
			const bool sel = p.name == m_param;
			if (ImGui::Selectable(label.c_str(), sel) && !sel) {
				m_param = p.name; m_label = label; m_times.clear(); m_values.clear(); m_offset = 0;
			}
			if (sel) ImGui::SetItemDefaultFocus();
		}
		ImGui::EndCombo();
	}
	ImGui::PopStyleColor();
	ImGui::SameLine();
	if (ImGui::Button(popLabel)) m_popped = !m_popped;
}

// Plots the circular sample buffer over a sliding [now - 30s, now] X window with auto-fitting Y limits.
void GraphWidget::drawPlot(const char* id, bool compact) {
	const bool has = !m_param.empty();
	const char* yLabel = has ? m_label.c_str() : "[parameter, unit]";
	const float footerH = compact ? ImGui::GetTextLineHeightWithSpacing() : 0.0f;
	if (compact) ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(4, 4));
	const bool open = ImPlot::BeginPlot(id, ImVec2(-1, compact ? -footerH : -1.0f),
										ImPlotFlags_NoLegend | ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect);
	if (compact) ImPlot::PopStyleVar();
	if (!open) return;

	ImPlot::SetupAxes(compact ? nullptr : "time (s)", compact ? nullptr : yLabel, ImPlotAxisFlags_None,
					  has ? ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit : ImPlotAxisFlags_None);
	ImPlot::SetupAxisLimits(ImAxis_X1, std::max(0.0, m_now - kWindowS), std::max(kWindowS, m_now), ImGuiCond_Always);
	if (!has) ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, 1.0, ImGuiCond_Always);

	if (has && !m_times.empty()) {
		ImPlotSpec spec;
		spec.LineColor = ImGui::ColorConvertU32ToFloat4(color::kBlue); spec.LineWeight = 2.0f; spec.Offset = m_offset;
		ImPlot::PlotLine(m_param.c_str(), m_times.data(), m_values.data(), (int)m_times.size(), spec);
	} else if (!has) {
		const ImPlotRect lim = ImPlot::GetPlotLimits();
		ImPlot::PlotText("Select a parameter above", (lim.X.Min + lim.X.Max) * 0.5, 0.5);
	}
	ImPlot::EndPlot();

	if (compact) {
		ImGui::PushStyleColor(ImGuiCol_Text, color::kMuted);
		ImGui::Text("y: %s", yLabel);
		const char* x = "x: time (s)";
		ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize(x).x);
		ImGui::TextUnformatted(x);
		ImGui::PopStyleColor();
	}
}

GraphGrid::GraphGrid(int count, int columns, const char* scope) : m_columns(columns) {
	m_graphs.reserve(count);
	for (int i = 0; i < count; ++i) m_graphs.emplace_back(i + 1, scope);
}

void GraphGrid::sample(const ParameterRegistry& r, const SimSnapshot& s) { for (auto& g : m_graphs) g.sample(r, s); }
void GraphGrid::drawPopOuts()                                            { for (auto& g : m_graphs) g.drawPopOut(); }

// Lays out all child GraphWidgets in an evenly proportioned `m_columns` grid filling the available region.
void GraphGrid::draw(const ParameterRegistry& registry) {
	const ImGuiStyle& st = ImGui::GetStyle();
	const int rows = ((int)m_graphs.size() + m_columns - 1) / m_columns;
	const ImVec2 avail = ImGui::GetContentRegionAvail();
	const ImVec2 cell((avail.x - st.ItemSpacing.x * (m_columns - 1)) / m_columns,
					  std::max(110.0f * st.FontScaleDpi, (avail.y - st.ItemSpacing.y * (rows - 1)) / rows));

	ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(250, 250, 250, 255));
	for (int i = 0; i < (int)m_graphs.size(); ++i) {
		if (i % m_columns != 0) ImGui::SameLine();
		ImGui::PushID(i);
		if (ImGui::BeginChild("##cell", cell, ImGuiChildFlags_Borders, ImGuiWindowFlags_NoScrollbar))
			m_graphs[i].draw(registry);
		ImGui::EndChild();
		ImGui::PopID();
	}
	ImGui::PopStyleColor();
}

} // namespace vde::ui
