// =============================================================================
// include/Client.hpp
// Declares simulation math primitives, SI-to-display unit conversions, telemetry
// snapshot structures, the binary UDP packet protocol, the non-blocking UDP
// VisualizationClient, the plottable ParameterRegistry, replay/recording helpers,
// and the central AppState coordinator.
// =============================================================================
#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <numbers>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace vde {

// ============================================================================
// Core types & math (all internal physics states are stored in SI units)
// ============================================================================

// Target UI refresh rate in Hertz (8.333 ms frame period).
inline constexpr double kUiRateHz = 120.0;

// Status codes returned by UDP socket setup and packet transmission calls.
enum class ErrorCode : std::int32_t { Ok = 0, SocketError, BindError, SendError, InvalidConfig };

// Selects standard or inverted coordinate-frame mapping between spacecraft and VDE hardware.
enum class FrameMapping : std::int32_t { Default = 0, Flipped = 1 };

// 3D Cartesian vector used for positions, velocities, forces, torques, and centres of gravity.
struct Vec3 { double x = 0.0, y = 0.0, z = 0.0; };

// Intrinsic Roll-Pitch-Yaw Euler angles (stored in radians internally, displayed in degrees).
struct Rpy  { double roll = 0.0, pitch = 0.0, yaw = 0.0; };

// 3x3 matrix in row-major order (linear array of 9 doubles, matching space_vde).
struct Mat3 {
	std::array<double, 9> m{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	constexpr Mat3() noexcept = default;
	constexpr Mat3(const std::array<double, 9>& elements) noexcept : m(elements) {}
};

// Combined 6-DOF Torque and Force structure (torque stored first, matching space_vde).
struct ForceTorque {
	Vec3 torque{};
	Vec3 force{};
};

// Unit quaternion following standard space_vde scalar-first convention: q(w, x, y, z).
struct Quaternion {
	double w = 1.0, x = 0.0, y = 0.0, z = 0.0; // scalar-first (w, x, y, z)

	// Converts ZYX Roll-Pitch-Yaw Euler angles (radians) into a unit quaternion.
	static Quaternion fromRpy(const Rpy& r) {
		const double cr = std::cos(r.roll * 0.5),  sr = std::sin(r.roll * 0.5);
		const double cp = std::cos(r.pitch * 0.5), sp = std::sin(r.pitch * 0.5);
		const double cy = std::cos(r.yaw * 0.5),   sy = std::sin(r.yaw * 0.5);
		return {cr * cp * cy + sr * sp * sy,
		        sr * cp * cy - cr * sp * sy,
		        cr * sp * cy + sr * cp * sy,
		        cr * cp * sy - sr * sp * cy};
	}

	// Extracts Roll-Pitch-Yaw Euler angles (radians) from this unit quaternion, clamping pitch to avoid NaN.
	Rpy toRpy() const {
		Rpy r;
		r.yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
		const double sinp = 2.0 * (w * y - z * x);
		if (std::abs(sinp) >= 1.0)
			r.pitch = std::copysign(std::numbers::pi / 2.0, sinp);
		else
			r.pitch = std::asin(sinp);
		r.roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
		return r;
	}
};

// Relative 6-DOF pose and rate state between the Chaser and Target docking frames.
struct RelativeState {
	Quaternion orientation;
	Vec3       positionM;
	Quaternion orientationRate;
	Vec3       velocityMS;
	Rpy toEuler() const { return orientation.toRpy(); }
};

// Conversion factors and scaling helpers between internal SI units and UI display units.
namespace units {
inline constexpr double kMmPerM    = 1000.0;
inline constexpr double kKmPerM    = 1.0e-3;
inline constexpr double kKmhPerMs  = 3.6;
inline constexpr double kDegPerRad = 180.0 / std::numbers::pi;
inline constexpr double kRadPerDeg = std::numbers::pi / 180.0;
inline constexpr double kMPerMm    = 1.0e-3;

constexpr Vec3 scaled(const Vec3& v, double f) { return {v.x * f, v.y * f, v.z * f}; }
constexpr Rpy  scaled(const Rpy& r, double f)  { return {r.roll * f, r.pitch * f, r.yaw * f}; }
} // namespace units

// Relative floating-point comparison helper tolerant to magnitude scaling.
template <std::floating_point T>
bool nearlyEqual(T a, T b, T eps) {
	return std::abs(a - b) <= eps * std::max({T(1), std::abs(a), std::abs(b)});
}

// Formatting helpers for simulation clock strings, output filenames, and path display.
std::string formatSimTime(double seconds);        // Formats elapsed seconds as "MM:SS.mmm"
std::string fileTimestamp();                      // Formats local wall-clock time as "YYYY-MM-DD_HHMM"
std::string fileNameOf(const std::string& path);  // Extracts the base filename from a full path

// ============================================================================
// Telemetry Snapshots & Run Configuration
// ============================================================================

// Kinematics and contact/actuator loads of the physical VDE Stewart platform and overhead gantry.
struct VdeState {
	Vec3        positionM, linearRateMS;
	Rpy         orientationRad, orientationRateRadS;
	ForceTorque moving;   // torque, then force (matching space_vde)
	ForceTorque overhead; // torque, then force (matching space_vde)
};

// Mass properties, body-frame rates, and Earth-Centred Inertial (ECI) orbital state of a spacecraft.
struct SpacecraftState {
	double massKg = 0.0;
	Vec3   cgM;
	Mat3   inertiaKgM2;
	Vec3   linearRateMS;
	Rpy    orientationRateRadS;
	Vec3   eciPositionM, eciVelocityMS;
	Rpy    eciOrientationRad, eciOrientationRateRadS;
};

// Navigation, Guidance, and Control (NGC) mode, fault state, and commanded forces/torques.
struct NgcState {
	std::string mode;
	std::string failureCase;
	Vec3 chaserForceN, chaserTorqueNm;
	Vec3 targetForceN, targetTorqueNm;
	bool disturbancesOn = false;
};

// Groups both spacecraft telemetry states together with the shared NGC telemetry state.
struct SpacecraftBundle {
	SpacecraftState chaser, target;
	NgcState        ngc;
};

// Complete time-stamped telemetry snapshot consumed by 3D scenes, value panels, and live plots.
struct SimSnapshot {
	double           timeS = 0.0;
	RelativeState    relative;
	VdeState         vde;
	SpacecraftBundle spacecraft;
};

// Safe travel and angular workspace envelope of the VDE hexapod platform (stored in metres and radians).
struct VdeLimits {
	double xMinM = -0.100, xMaxM = 0.100;
	double yMinM = -0.100, yMaxM = 0.100;
	double zMinM = 0.0,    zMaxM = 0.550;
	double rollMaxRad  = 5.0 * units::kRadPerDeg;
	double pitchMaxRad = 5.0 * units::kRadPerDeg;
	double yawMaxRad   = 10.0 * units::kRadPerDeg;
};

// Initial simulation parameters sent to the external Dynamics Engine when initializing a run.
struct RunConfig {
	double       chaserMassKg      = 2150.0;
	double       targetMassKg      = 7800.0;
	Mat3         chaserInertiaKgM2 = {{{2042.5, 21.5, 0.0, 21.5, 2365.0, 0.0, 0.0, 0.0, 1505.0}}};
	Mat3         targetInertiaKgM2 = {{{7410.0, 78.0, 0.0, 78.0, 8580.0, 0.0, 0.0, 0.0, 5460.0}}};
	double       orbitRadiusM      = 6778.0e3;
	double       inclinationRad    = 25.0 * units::kRadPerDeg;
	double       alongTrackSepM    = -12.0;
	FrameMapping frameMapping      = FrameMapping::Default;
	bool         disturbancesOn    = true;
	std::string  ngcMode           = "FINAL APPROACH (closing)";
	VdeLimits    vdeLimits;
};

// ============================================================================
// UDP Wire Protocol & Non-Blocking VisualizationClient
// ============================================================================

namespace net {
inline constexpr const char*   kDefaultHost       = "127.0.0.1";
inline constexpr std::uint16_t kDefaultConfigPort = 5001; // Outbound: UI Client -> Dynamics Engine
inline constexpr std::uint16_t kDefaultStatePort  = 5002; // Inbound:  Dynamics Engine -> UI Client
inline constexpr std::uint32_t kProtocolMagic     = 0x56444531u; // ASCII "VDE1" packet identifier
inline constexpr std::uint32_t kProtocolVersion   = 1u;

// Fixed-layout binary datagram sent from the UI client to the Dynamics Engine to configure/start a run.
struct RunConfigPacket {
	std::uint32_t magic          = kProtocolMagic;
	std::uint32_t version        = kProtocolVersion;
	double        chaserMassKg   = 2150.0;
	double        targetMassKg   = 7800.0;
	Mat3          chaserInertia;
	Mat3          targetInertia;
	double        orbitRadiusM   = 6778.0e3;
	double        inclinationRad = 0.0;
	double        alongTrackSepM = -12.0;
	std::int32_t  frameMapping   = 0;
	std::uint8_t  disturbancesOn = 1;
	char          ngcMode[64]    = {};
};

// Fixed-layout binary datagram streamed from the Dynamics Engine to the UI client each physics step.
struct StatePacket {
	std::uint32_t   magic          = kProtocolMagic;
	std::uint32_t   version        = kProtocolVersion;
	std::uint64_t   sequence       = 0;
	double          timeS          = 0.0;
	RelativeState   relative;
	VdeState        vde;
	SpacecraftState chaser;
	SpacecraftState target;
	ForceTorque     ngcChaser; // torque first, force second (matching space_vde)
	ForceTorque     ngcTarget; // torque first, force second (matching space_vde)
	std::uint8_t    disturbancesOn = 1;
	char            ngcMode[64]        = {};
	char            ngcFailureCase[64] = {};
};
} // namespace net

// Manages non-blocking UDP communication with the external Dynamics Engine server.
// Sends RunConfigPacket datagrams on the config port and drains incoming StatePacket
// datagrams on the state port so the 120 Hz UI always renders the freshest sequence number.
class VisualizationClient {
public:
	VisualizationClient();
	~VisualizationClient();
	VisualizationClient(const VisualizationClient&) = delete;
	VisualizationClient& operator=(const VisualizationClient&) = delete;

	// Opens non-blocking TX/RX UDP sockets and binds the local state receiver port.
	ErrorCode connect(const std::string& host = net::kDefaultHost,
					  std::uint16_t configPort = net::kDefaultConfigPort,
					  std::uint16_t statePort  = net::kDefaultStatePort);

	// Serializes and transmits a RunConfigPacket to the Dynamics Engine's config port.
	ErrorCode sendRunConfig(const RunConfig& config);

	// Drains all queued UDP state packets and writes the newest snapshot into `out`.
	bool      getLatestState(SimSnapshot& out);

	std::uint64_t packetsReceived() const { return m_packetsReceived; }
	std::uint64_t lastSequence() const    { return m_lastSequence; }
	bool          hasState() const        { return m_hasState; }

private:
	void closeSockets();

	std::string   m_host       = net::kDefaultHost;
	std::uint16_t m_configPort = net::kDefaultConfigPort;
	std::uint16_t m_statePort  = net::kDefaultStatePort;
	std::uintptr_t m_txHandle;
	std::uintptr_t m_rxHandle;

	SimSnapshot   m_latest;
	bool          m_hasState        = false;
	std::uint64_t m_packetsReceived = 0;
	std::uint64_t m_lastSequence    = 0;
};

// ============================================================================
// Plottable Parameters, Video Recorder, Replay Player & AppState
// ============================================================================

// Metadata for a single scalar telemetry channel available in graph dropdown menus.
struct ParameterInfo {
	std::string name, unit;
	std::string label() const { return name + " (" + unit + ")"; }
};

// Catalog of all scalar telemetry channels that can be plotted in real time.
// Maps human-readable channel names to extractor lambdas that convert SI snapshot fields to display units.
class ParameterRegistry {
public:
	ParameterRegistry();
	const std::vector<ParameterInfo>& parameters() const { return m_params; }
	double value(const std::string& name, const SimSnapshot& snapshot) const;

private:
	using Getter = std::function<double(const SimSnapshot&)>;
	template <class T> using Ref = std::function<const T&(const SimSnapshot&)>;

	void add(const std::string& name, const std::string& unit, Getter getter);
	void addVec3(const std::string& prefix, const std::string& unit, double toDisplay, Ref<Vec3> get);
	void addRpy(const std::string& prefix, const std::string& unit, double toDisplay, Ref<Rpy> get);
	void addCraft(const std::string& who, Ref<SpacecraftState> get);

	std::vector<ParameterInfo>                   m_params;
	std::vector<Getter>                          m_getters;
	std::unordered_map<std::string, std::size_t> m_index;
};

// Tracks viewport video-recording state and receives RGBA framebuffers when active encoding is enabled.
class Recorder {
public:
	void startRecording(const std::string& path) { m_path = path; m_recording = true; }
	void stop()                                  { m_recording = false; }
	bool isRecording() const                     { return m_recording; }
	bool wantsFrames() const                     { return false; }
	void pushFrame(const std::uint8_t*, int, int) {}

private:
	std::string m_path;
	bool        m_recording = false;
};

// Drives offline playback transport (play, pause, seek, speed multiplier) and synthesizes replay snapshots.
class ReplayPlayer {
public:
	bool   open(const std::string&) { m_time = 0.0; m_playing = false; return true; }
	void   close()                  { m_playing = false; }
	double duration() const         { return kDurationS; }
	double time() const             { return m_time; }
	void   seek(double t)           { m_time = std::clamp(t, 0.0, kDurationS); }
	void   play()                   { if (m_time >= kDurationS) m_time = 0.0; m_playing = true; }
	void   pause()                  { m_playing = false; }
	bool   isPlaying() const        { return m_playing; }
	void   setSpeed(double s)       { m_speed = s; }
	double speed() const            { return m_speed; }
	void   update(double dtS);
	SimSnapshot snapshot(const RunConfig& cfg) const;

private:
	static constexpr double kDurationS = 90.0;
	double m_time = 0.0, m_speed = 1.0;
	bool   m_playing = false;
};

// Identifies which workspace(s) should switch from live UDP telemetry to offline replay data.
enum class ReplayTarget { VdeView = 0, SpacecraftView = 1, Both = 2 };

// Identifies one of the two primary 3D workspaces (VDE hardware view or orbital Spacecraft view).
enum class ViewId { Vde, Spacecraft };

// UI-to-application command messages queued during widget rendering and applied cleanly on the next frame.
struct StartRunCmd        { RunConfig config; };
struct StartReplayCmd     { std::string csvPath; ReplayTarget target = ReplayTarget::Both; };
struct StopReplayCmd      {};
struct ReplayPlayCmd      {};
struct ReplayPauseCmd     {};
struct ReplaySeekCmd      { double timeS = 0.0; };
struct ReplaySetSpeedCmd  { double factor = 1.0; };
struct ToggleRecordingCmd { ViewId view = ViewId::Vde; std::string fileTag; };
struct ExportDataCmd      { std::string filePrefix; };

using Command = std::variant<StartRunCmd, StartReplayCmd, StopReplayCmd, ReplayPlayCmd, ReplayPauseCmd,
							 ReplaySeekCmd, ReplaySetSpeedCmd, ToggleRecordingCmd, ExportDataCmd>;

// Read-only playback progress exposed to the replay transport bar.
struct ReplayStatus {
	bool   playing   = false;
	double timeS     = 0.0, durationS = 0.0, speed = 1.0;
};

// Live UDP connection and packet counters displayed in the main menu bar and toolbar.
struct ClientStats {
	bool          connected       = false;
	bool          hasState        = false;
	std::string   host            = net::kDefaultHost;
	std::uint16_t configPort      = net::kDefaultConfigPort;
	std::uint16_t statePort       = net::kDefaultStatePort;
	std::uint64_t packetsReceived = 0;
	std::uint64_t lastSequence    = 0;
};

// Central application state container owned by main().
// Coordinates the UDP VisualizationClient, command queue, live/replay snapshots, recorders, and XML preview.
class AppState {
public:
	explicit AppState(RunConfig initialConfig = {});

	// Connects the UDP client to the Dynamics Engine endpoint and sends the initial RunConfig.
	ErrorCode connectClient(const std::string& host = net::kDefaultHost,
							std::uint16_t configPort = net::kDefaultConfigPort,
							std::uint16_t statePort  = net::kDefaultStatePort);

	// Processes queued UI commands, polls the latest UDP state packet, and advances replay playback.
	void update(double frameDtS);

	// Enqueues a UI action to be executed at the start of the next update() tick.
	void submit(Command command) { m_pending.push_back(std::move(command)); }

	const RunConfig&         runConfig() const   { return m_runConfig; }
	const VdeLimits&         vdeLimits() const   { return m_runConfig.vdeLimits; }
	const ParameterRegistry& parameters() const  { return m_parameters; }
	const ClientStats&       clientStats() const { return m_clientStats; }

	std::string configSourceName() const { return "config.xml"; }
	std::string loadConfigXml() const;

	Recorder& recorder(ViewId view)       { return view == ViewId::Vde ? m_vdeRecorder : m_scRecorder; }
	bool      isRecording(ViewId v) const { return (v == ViewId::Vde ? m_vdeRecorder : m_scRecorder).isRecording(); }

	// Routes either the live UDP snapshot or the replay snapshot to each workspace depending on ReplayTarget.
	const SimSnapshot& vdeSnapshot() const        { return replayOnVde() ? m_replay : m_live; }
	const SimSnapshot& spacecraftSnapshot() const { return replayOnSpacecraft() ? m_replay : m_live; }
	const SimSnapshot& analysisSnapshot() const   { return m_replayActive ? m_replay : m_live; }

	bool                replayActive() const       { return m_replayActive; }
	bool                replayOnVde() const        { return m_replayActive && m_replayTarget != ReplayTarget::SpacecraftView; }
	bool                replayOnSpacecraft() const { return m_replayActive && m_replayTarget != ReplayTarget::VdeView; }
	const std::string&  replayFile() const         { return m_replayFile; }
	const ReplayStatus& replayStatus() const       { return m_replayStatus; }

private:
	void apply(const Command& command);

	RunConfig            m_runConfig;
	VisualizationClient  m_client;
	ClientStats          m_clientStats;
	ParameterRegistry    m_parameters;
	ReplayPlayer         m_replayPlayer;
	Recorder             m_vdeRecorder, m_scRecorder;
	std::vector<Command> m_pending;

	SimSnapshot  m_live, m_replay;
	bool         m_replayActive = false;
	ReplayTarget m_replayTarget = ReplayTarget::Both;
	std::string  m_replayFile;
	ReplayStatus m_replayStatus;
};

} // namespace vde
