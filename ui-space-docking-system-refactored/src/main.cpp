// =============================================================================
// src/main.cpp
// Application entry point for the VDE Docking Simulator UI client.
// Resolves UDP endpoints from simulation_config.xml with CLI overrides,
// initializes GLFW, OpenGL 4.5 (via GLAD), Dear ImGui (docking + multi-viewports),
// and ImPlot, connects AppState to the external Dynamics Engine over UDP, and
// drives the 120 Hz (8.333 ms) render and telemetry polling loop.
// =============================================================================

#include "Client.hpp"
#include "UiCore.hpp"
#include "UiWindows.hpp"
#include "space_docking/dynamics_engine/XmlConfigUtils.hpp"

#include <glad/gl.h>  // must precede GLFW
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <thread>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#include <mmsystem.h>
#endif

namespace {

// Command-line network configuration specifying where the external Dynamics Engine listens and streams.
struct CliOptions {
	std::string   host       = vde::net::kDefaultHost;
	std::uint16_t configPort = vde::net::kDefaultConfigPort;
	std::uint16_t statePort  = vde::net::kDefaultStatePort;
};

// Loads UDP endpoint configuration from simulation_config.xml, then applies explicit CLI overrides
// (`--host <ip>`, `--config-port <port>`, `--state-port <port>`, `--config <xml>`).
CliOptions resolveNetworkOptions(int argc, char** argv) {
	CliOptions opt;
	std::string configPath;
	std::optional<std::string>   cliHost;
	std::optional<std::uint16_t> cliConfigPort;
	std::optional<std::uint16_t> cliStatePort;

	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--host") == 0 && i + 1 < argc)
			cliHost = argv[++i];
		else if (std::strcmp(argv[i], "--config-port") == 0 && i + 1 < argc) {
			std::uint16_t port = 0;
			if (space_docking::dynamics::xml_utils::parsePort(argv[++i], port)) cliConfigPort = port;
		} else if (std::strcmp(argv[i], "--state-port") == 0 && i + 1 < argc) {
			std::uint16_t port = 0;
			if (space_docking::dynamics::xml_utils::parsePort(argv[++i], port)) cliStatePort = port;
		} else if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
			configPath = argv[++i];
		}
	}

	const std::string resolvedConfig = vde::resolveConfigPath("simulation_config.xml", configPath);
	std::string unusedTargetHost = opt.host;
	space_docking::dynamics::xml_utils::loadNetworkEndpointsFromFile(
	    resolvedConfig, unusedTargetHost, opt.configPort, opt.statePort);

	if (cliHost)       opt.host       = *cliHost;
	if (cliConfigPort) opt.configPort = *cliConfigPort;
	if (cliStatePort)  opt.statePort  = *cliStatePort;
	return opt;
}

// Parents detached ImGui multi-viewport windows to the main Win32 HWND so pop-outs stay above the main window.
void keepPopOutsAboveMainWindow(GLFWwindow* mainWindow) {
#ifdef _WIN32
	const HWND owner = glfwGetWin32Window(mainWindow);
	const ImGuiPlatformIO& pio = ImGui::GetPlatformIO();
	for (int i = 1; i < pio.Viewports.Size; ++i) {
		const HWND hwnd = (HWND)pio.Viewports[i]->PlatformHandleRaw;
		if (hwnd && (HWND)GetWindowLongPtr(hwnd, GWLP_HWNDPARENT) != owner)
			SetWindowLongPtr(hwnd, GWLP_HWNDPARENT, (LONG_PTR)owner);
	}
#else
	(void)mainWindow;
#endif
}

// Logs GLFW windowing or OpenGL context creation errors to stderr.
void glfwErrorCallback(int code, const char* desc) {
	std::fprintf(stderr, "[GLFW] error %d: %s\n", code, desc);
}

} // namespace

int main(int argc, char** argv) {
	const CliOptions cli = resolveNetworkOptions(argc, argv);

	glfwSetErrorCallback(glfwErrorCallback);
	if (!glfwInit()) return 1;

	// Request a maximized OpenGL 4.5 Core Profile window scaled to the primary monitor's DPI.
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);
	const float dpiScale = ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor());
	GLFWwindow* window = glfwCreateWindow((int)(1600 * dpiScale), (int)(1000 * dpiScale),
										  "VDE Docking Simulator – VDE", nullptr, nullptr);
	if (!window) {
		glfwTerminate();
		return 1;
	}
	glfwMakeContextCurrent(window);
	glfwSwapInterval(0);  // Disable monitor VSync clamping so the render loop runs freely at 120 Hz
	if (!gladLoadGL(glfwGetProcAddress)) {
		glfwDestroyWindow(window);
		glfwTerminate();
		return 1;
	}

	// Initialize Dear ImGui and ImPlot with keyboard navigation, docking, and multi-viewport OS windows enabled.
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImPlot::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_DockingEnable;
#ifndef __linux__
	io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
#endif
	io.ConfigDpiScaleFonts     = true;
	io.ConfigDpiScaleViewports = true;
	vde::ui::setupTheme(dpiScale);
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 450 core");

#ifdef _WIN32
	// Request 1 ms Windows scheduler timer granularity for accurate 120 Hz sleep pacing.
	timeBeginPeriod(1);
#endif

	{
		using Clock = std::chrono::steady_clock;
		constexpr auto kFramePeriod = std::chrono::nanoseconds(8'333'333);  // 120 Hz (8.333 ms per frame)
		auto nextFrame = Clock::now();

		// Create central application state, bind UDP client sockets, and construct the UI controller.
		vde::AppState app;
		app.connectClient(cli.host, cli.configPort, cli.statePort);
		vde::ui::MainUi ui(app);
		std::string title;

		while (!glfwWindowShouldClose(window)) {
			glfwPollEvents();
			if (glfwGetWindowAttrib(window, GLFW_ICONIFIED)) {
				ImGui_ImplGlfw_Sleep(10);
				nextFrame = Clock::now();
				continue;
			}

			ImGui_ImplOpenGL3_NewFrame();
			ImGui_ImplGlfw_NewFrame();
			ImGui::NewFrame();

			// Poll non-blocking UDP state stream and build all UI widgets and 3D off-screen viewports.
			app.update(io.DeltaTime);
			ui.frame();

			// Composite Dear ImGui draw lists onto the main window's default framebuffer.
			ImGui::Render();
			int w = 0, h = 0;
			glfwGetFramebufferSize(window, &w, &h);
			glBindFramebuffer(GL_FRAMEBUFFER, 0);
			glViewport(0, 0, w, h);
			glClearColor(0.94f, 0.94f, 0.94f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT);
			ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

			// Update and render any detached pop-out OS viewport windows.
			if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
				GLFWwindow* backup = glfwGetCurrentContext();
				ImGui::UpdatePlatformWindows();
				keepPopOutsAboveMainWindow(window);
				ImGui::RenderPlatformWindowsDefault();
				glfwMakeContextCurrent(backup);
			}
			glfwSwapBuffers(window);

			if (title != ui.osWindowTitle()) {
				title = ui.osWindowTitle();
				glfwSetWindowTitle(window, title.c_str());
			}

			// Hybrid sleep + short spin-yield frame limiter keeping steady 120 Hz frame pacing.
			nextFrame += kFramePeriod;
			const auto now = Clock::now();
			if (nextFrame > now) {
				if (nextFrame - now > std::chrono::milliseconds(2))
					std::this_thread::sleep_until(nextFrame - std::chrono::milliseconds(1));
				while (Clock::now() < nextFrame) std::this_thread::yield();
			} else {
				nextFrame = now;
			}
		}
	}

#ifdef _WIN32
	timeEndPeriod(1);
#endif
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImPlot::DestroyContext();
	ImGui::DestroyContext();
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}
