// =============================================================================
// include/UiWindows.hpp
// Declares the top-level MainUi controller that owns and orchestrates the menu
// bar, docking layout, VDE and Spacecraft workspaces, 12-plot Analysis window,
// and modal configuration/replay dialogs each render frame.
// =============================================================================
#pragma once

#include <memory>
#include <string>

namespace vde { class AppState; }

namespace vde::ui {

// Top-level GUI facade hiding all workspace windows and modal dialog states behind a Pimpl pointer.
class MainUi {
public:
	explicit MainUi(AppState& app);
	~MainUi();

	// Renders one complete UI frame across all workspaces, pop-outs, and dialogs.
	void frame();

	// Returns the active workspace title used to update the native OS window title bar.
	const std::string& osWindowTitle() const;

private:
	struct Impl;
	std::unique_ptr<Impl> m_impl;
};

} // namespace vde::ui
