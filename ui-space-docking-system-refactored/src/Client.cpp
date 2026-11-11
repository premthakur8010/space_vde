// =============================================================================
// src/Client.cpp
// Implements time/path string formatters, standard BSD non-blocking UDP sockets
// (native POSIX on Ubuntu/Linux via <sys/socket.h>, compatibility shim on Windows),
// VisualizationClient packet TX/RX,
// ParameterRegistry channel registration, ReplayPlayer trajectory evaluation,
// and AppState command processing.
// =============================================================================
#include "Client.hpp"
#include "space_docking/dynamics_engine/XmlConfigUtils.hpp"

#include <cassert>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <sstream>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace vde {

// ============================================================================
// Formatting helpers
// ============================================================================

// Formats simulation seconds as zero-padded "MM:SS.mmm" for toolbar and replay clocks.
std::string formatSimTime(double seconds) {
	if (seconds < 0.0) seconds = 0.0;
	const long long ms = (long long)std::llround(seconds * 1000.0);
	char buf[32];
	std::snprintf(buf, sizeof(buf), "%02lld:%02lld.%03lld", ms / 60000, (ms / 1000) % 60, ms % 1000);
	return buf;
}

// Formats local wall-clock date and time for timestamping recorded video filenames.
std::string fileTimestamp() {
	std::time_t now = std::time(nullptr);
	char buf[32];
	std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H%M", std::localtime(&now));
	return buf;
}

// Strips any leading directory components (`/` or `\`) from a file path.
std::string fileNameOf(const std::string& path) {
	const std::size_t slash = path.find_last_of("/\\");
	return slash == std::string::npos ? path : path.substr(slash + 1);
}

// Resolves the directory containing the running executable so bundled fonts/configs work from any CWD.
std::string executableDir() {
	char buf[4096] = {};
#ifdef _WIN32
	GetModuleFileNameA(nullptr, buf, sizeof(buf) - 1);
#else
	if (readlink("/proc/self/exe", buf, sizeof(buf) - 1) < 0) return ".";
#endif
	const std::string p(buf);
	const std::size_t slash = p.find_last_of("/\\");
	return slash == std::string::npos ? "." : p.substr(0, slash);
}

// Resolves config XML paths dynamically from CWD or executable ancestors without hardcoded folder names.
std::string resolveConfigPath(std::string_view fileName, std::string_view explicitPath) {
	return space_docking::dynamics::xml_utils::resolveConfigFilePath(fileName, explicitPath, executableDir());
}

// ============================================================================
// Non-blocking UDP socket helpers & VisualizationClient
// ============================================================================

namespace {
// Thin platform adapter so all networking code below uses standard BSD socket calls directly
// (native POSIX sockets on Ubuntu/Linux; mapped onto Windows sockets where POSIX lacks <sys/socket.h>).
#ifdef _WIN32
using NativeSocket = SOCKET;
constexpr std::uintptr_t kBadSocket = static_cast<std::uintptr_t>(INVALID_SOCKET);
struct SocketSubsystemInit {
	SocketSubsystemInit()  { WSADATA w = {}; WSAStartup(MAKEWORD(2, 2), &w); }
	~SocketSubsystemInit() { WSACleanup(); }
};
void ensureSockets() { static SocketSubsystemInit s_init; }
bool isInvalidNative(NativeSocket s) { return s == INVALID_SOCKET; }
void closeNative(NativeSocket s) { ::closesocket(s); }
bool setNonBlockingNative(NativeSocket s) { u_long nb = 1; return ::ioctlsocket(s, FIONBIO, &nb) == 0; }
#else
using NativeSocket = int;
constexpr std::uintptr_t kBadSocket = ~std::uintptr_t{0};
void ensureSockets() {}
bool isInvalidNative(NativeSocket s) { return s < 0; }
void closeNative(NativeSocket s) { ::close(s); }
bool setNonBlockingNative(NativeSocket s) {
	const int flags = ::fcntl(s, F_GETFL, 0);
	return flags >= 0 && ::fcntl(s, F_SETFL, flags | O_NONBLOCK) == 0;
}
#endif

NativeSocket toNative(std::uintptr_t h) { return static_cast<NativeSocket>(h); }
void closeHandle(std::uintptr_t& h) {
	if (h != kBadSocket) { closeNative(toNative(h)); h = kBadSocket; }
}

// Builds an IPv4 socket address structure from a dotted-quad IP string and host-order port.
sockaddr_in makeAddr(const char* host, std::uint16_t port) {
	sockaddr_in a = {};
	a.sin_family = AF_INET;
	a.sin_port   = htons(port);
	inet_pton(AF_INET, host, &a.sin_addr);
	return a;
}

// Creates an IPv4 UDP socket, puts it into non-blocking mode, and enlarges the OS receive buffer
// to 256 KB so high-rate physics packets do not drop between 120 Hz UI frames.
std::uintptr_t openUdpNonBlocking() {
	ensureSockets();
	const NativeSocket s = ::socket(AF_INET, SOCK_DGRAM, 0);
	if (isInvalidNative(s)) return kBadSocket;
	if (!setNonBlockingNative(s)) { closeNative(s); return kBadSocket; }
	const int rcvBuf = 256 * 1024;
	::setsockopt(s, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char*>(&rcvBuf), sizeof(rcvBuf));
	return static_cast<std::uintptr_t>(s);
}

// Copies a validated binary StatePacket received off the wire into an application SimSnapshot.
SimSnapshot unpackState(const net::StatePacket& p) {
	SimSnapshot s;
	s.timeS                         = p.timeS;
	s.relative                      = p.relative;
	s.vde                           = p.vde;
	s.spacecraft.chaser             = p.chaser;
	s.spacecraft.target             = p.target;
	s.spacecraft.ngc.chaserForceN   = p.ngcChaser.force;
	s.spacecraft.ngc.chaserTorqueNm = p.ngcChaser.torque;
	s.spacecraft.ngc.targetForceN   = p.ngcTarget.force;
	s.spacecraft.ngc.targetTorqueNm = p.ngcTarget.torque;
	s.spacecraft.ngc.disturbancesOn = p.disturbancesOn != 0;
	s.spacecraft.ngc.mode           = p.ngcMode;
	s.spacecraft.ngc.failureCase    = p.ngcFailureCase;
	return s;
}
} // namespace

VisualizationClient::VisualizationClient() : m_txHandle(kBadSocket), m_rxHandle(kBadSocket) {}
VisualizationClient::~VisualizationClient() { closeSockets(); }

void VisualizationClient::closeSockets() {
	closeHandle(m_txHandle);
	closeHandle(m_rxHandle);
}

// Opens TX and RX non-blocking UDP sockets and binds RX to 0.0.0.0:statePort to accept local or LAN streams.
ErrorCode VisualizationClient::connect(const std::string& host, std::uint16_t configPort, std::uint16_t statePort) {
	closeSockets();
	m_host       = host;
	m_configPort = configPort;
	m_statePort  = statePort;

	m_txHandle = openUdpNonBlocking();
	m_rxHandle = openUdpNonBlocking();
	if (m_txHandle == kBadSocket || m_rxHandle == kBadSocket) {
		closeSockets();
		return ErrorCode::SocketError;
	}

	const sockaddr_in bindAddr = makeAddr("0.0.0.0", m_statePort);
	const int reuse = 1;
	::setsockopt(toNative(m_rxHandle), SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&reuse), sizeof(reuse));
	if (::bind(toNative(m_rxHandle), reinterpret_cast<const sockaddr*>(&bindAddr), sizeof(bindAddr)) != 0)
		return ErrorCode::BindError;
	return ErrorCode::Ok;
}

// Packs RunConfig fields into a fixed-size RunConfigPacket and sends it to <m_host>:<m_configPort>.
ErrorCode VisualizationClient::sendRunConfig(const RunConfig& cfg) {
	if (m_txHandle == kBadSocket) return ErrorCode::SocketError;
	net::RunConfigPacket p;
	p.chaserMassKg   = cfg.chaserMassKg;
	p.targetMassKg   = cfg.targetMassKg;
	p.chaserInertia  = cfg.chaserInertiaKgM2;
	p.targetInertia  = cfg.targetInertiaKgM2;
	p.orbitRadiusM   = cfg.orbitRadiusM;
	p.inclinationRad = cfg.inclinationRad;
	p.alongTrackSepM = cfg.alongTrackSepM;
	p.frameMapping   = (std::int32_t)cfg.frameMapping;
	p.disturbancesOn = cfg.disturbancesOn ? 1u : 0u;
	std::snprintf(p.ngcMode, sizeof(p.ngcMode), "%s", cfg.ngcMode.c_str());

	const sockaddr_in dst = makeAddr(m_host.c_str(), m_configPort);
	const long long sent = static_cast<long long>(
		::sendto(toNative(m_txHandle), reinterpret_cast<const char*>(&p), static_cast<int>(sizeof(p)), 0,
		         reinterpret_cast<const sockaddr*>(&dst), sizeof(dst)));
	return sent == static_cast<long long>(sizeof(p)) ? ErrorCode::Ok : ErrorCode::SendError;
}

// Reads all pending UDP datagrams from the receive buffer without blocking, validates magic/version/size,
// discards stale sequence numbers, and updates `out` with the highest-sequence telemetry snapshot.
bool VisualizationClient::getLatestState(SimSnapshot& out) {
	if (m_rxHandle == kBadSocket) return m_hasState;
	net::StatePacket pkt, newest;
	bool gotNew = false;

	for (;;) {
		const long long n = static_cast<long long>(
			::recvfrom(toNative(m_rxHandle), reinterpret_cast<char*>(&pkt), static_cast<int>(sizeof(pkt)), 0, nullptr, nullptr));
		if (n <= 0) break;
		if (n == static_cast<long long>(sizeof(pkt)) && pkt.magic == net::kProtocolMagic && pkt.version == net::kProtocolVersion) {
			if (!m_hasState || pkt.sequence >= m_lastSequence) {
				newest         = pkt;
				m_lastSequence = pkt.sequence;
				gotNew         = true;
			}
			++m_packetsReceived;
		}
	}

	if (gotNew) {
		m_latest   = unpackState(newest);
		m_hasState = true;
	}
	if (m_hasState) out = m_latest;
	return m_hasState;
}

// ============================================================================
// ParameterRegistry & ReplayPlayer
// ============================================================================

// Registers all plottable VDE, Chaser, Target, and NGC scalar channels along with display unit conversions.
ParameterRegistry::ParameterRegistry() {
	using S = SimSnapshot;
	using namespace units;

	addVec3("VDE Position", "mm", kMmPerM, [](const S& s) -> const Vec3& { return s.vde.positionM; });
	addVec3("VDE Linear rate", "mm/s", kMmPerM, [](const S& s) -> const Vec3& { return s.vde.linearRateMS; });
	addRpy("VDE Orientation", "deg", kDegPerRad, [](const S& s) -> const Rpy& { return s.vde.orientationRad; });
	addRpy("VDE Orientation rate", "deg/s", kDegPerRad, [](const S& s) -> const Rpy& { return s.vde.orientationRateRadS; });
	addVec3("VDE Moving platform force", "N", 1.0, [](const S& s) -> const Vec3& { return s.vde.moving.force; });
	addVec3("VDE Moving platform torque", "N·m", 1.0, [](const S& s) -> const Vec3& { return s.vde.moving.torque; });
	addVec3("VDE Overhead platform force", "N", 1.0, [](const S& s) -> const Vec3& { return s.vde.overhead.force; });
	addVec3("VDE Overhead platform torque", "N·m", 1.0, [](const S& s) -> const Vec3& { return s.vde.overhead.torque; });

	addCraft("Chaser", [](const S& s) -> const SpacecraftState& { return s.spacecraft.chaser; });
	addCraft("Target", [](const S& s) -> const SpacecraftState& { return s.spacecraft.target; });

	addVec3("NGC Chaser force", "N", 1.0, [](const S& s) -> const Vec3& { return s.spacecraft.ngc.chaserForceN; });
	addVec3("NGC Chaser torque", "N·m", 1.0, [](const S& s) -> const Vec3& { return s.spacecraft.ngc.chaserTorqueNm; });
	addVec3("NGC Target force", "N", 1.0, [](const S& s) -> const Vec3& { return s.spacecraft.ngc.targetForceN; });
	addVec3("NGC Target torque", "N·m", 1.0, [](const S& s) -> const Vec3& { return s.spacecraft.ngc.targetTorqueNm; });
}

// Evaluates the named channel's getter against `snapshot`, returning 0.0 if the name is unknown.
double ParameterRegistry::value(const std::string& name, const SimSnapshot& snapshot) const {
	auto it = m_index.find(name);
	return it == m_index.end() ? 0.0 : m_getters[it->second](snapshot);
}

void ParameterRegistry::add(const std::string& name, const std::string& unit, Getter getter) {
	m_index[name] = m_params.size();
	m_params.push_back({name, unit});
	m_getters.push_back(std::move(getter));
}

// Expands a Vec3 field into three individual X / Y / Z plottable channels.
void ParameterRegistry::addVec3(const std::string& prefix, const std::string& unit, double toDisplay, Ref<Vec3> get) {
	add(prefix + " X", unit, [get, toDisplay](const SimSnapshot& s) { return get(s).x * toDisplay; });
	add(prefix + " Y", unit, [get, toDisplay](const SimSnapshot& s) { return get(s).y * toDisplay; });
	add(prefix + " Z", unit, [get, toDisplay](const SimSnapshot& s) { return get(s).z * toDisplay; });
}

// Expands an Rpy field into three individual Roll / Pitch / Yaw plottable channels.
void ParameterRegistry::addRpy(const std::string& prefix, const std::string& unit, double toDisplay, Ref<Rpy> get) {
	add(prefix + " Roll", unit, [get, toDisplay](const SimSnapshot& s) { return get(s).roll * toDisplay; });
	add(prefix + " Pitch", unit, [get, toDisplay](const SimSnapshot& s) { return get(s).pitch * toDisplay; });
	add(prefix + " Yaw", unit, [get, toDisplay](const SimSnapshot& s) { return get(s).yaw * toDisplay; });
}

// Registers mass, centre of gravity, body rates, and ECI orbital channels for a single spacecraft.
void ParameterRegistry::addCraft(const std::string& who, Ref<SpacecraftState> get) {
	using namespace units;
	add(who + " Mass", "kg", [get](const SimSnapshot& s) { return get(s).massKg; });
	addVec3(who + " CG", "m", 1.0, [get](const SimSnapshot& s) -> const Vec3& { return get(s).cgM; });
	addVec3(who + " Linear rate", "m/s", 1.0, [get](const SimSnapshot& s) -> const Vec3& { return get(s).linearRateMS; });
	addRpy(who + " Orientation rate", "deg/s", kDegPerRad, [get](const SimSnapshot& s) -> const Rpy& { return get(s).orientationRateRadS; });
	addVec3(who + " Position", "km", kKmPerM, [get](const SimSnapshot& s) -> const Vec3& { return get(s).eciPositionM; });
	addVec3(who + " Velocity", "km/h", kKmhPerMs, [get](const SimSnapshot& s) -> const Vec3& { return get(s).eciVelocityMS; });
	addRpy(who + " Orientation", "deg", kDegPerRad, [get](const SimSnapshot& s) -> const Rpy& { return get(s).eciOrientationRad; });
}

// Advances the replay playhead by frameDtS scaled by playback speed, auto-pausing at the end of the run.
void ReplayPlayer::update(double dtS) {
	if (!m_playing) return;
	m_time = std::min(m_time + dtS * m_speed, kDurationS);
	if (m_time >= kDurationS) m_playing = false;
}

// Evaluates a smooth demonstration trajectory at the current replay timestamp.
SimSnapshot ReplayPlayer::snapshot(const RunConfig& cfg) const {
	const double t = 37.0 + 1.6 * m_time;
	SimSnapshot s;
	s.timeS = m_time;
	s.vde.positionM      = {0.06 * std::sin(0.21 * t), 0.045 * std::sin(0.17 * t + 1.0), 0.275 + 0.20 * std::sin(0.11 * t)};
	s.vde.orientationRad = {3.0 * units::kRadPerDeg * std::sin(0.31 * t),
							2.5 * units::kRadPerDeg * std::sin(0.23 * t + 0.5),
							7.0 * units::kRadPerDeg * std::sin(0.13 * t)};
	s.spacecraft.chaser.massKg      = cfg.chaserMassKg;
	s.spacecraft.chaser.inertiaKgM2 = cfg.chaserInertiaKgM2;
	s.spacecraft.target.massKg      = cfg.targetMassKg;
	s.spacecraft.target.inertiaKgM2 = cfg.targetInertiaKgM2;
	s.spacecraft.ngc.mode           = cfg.ngcMode;
	s.spacecraft.ngc.failureCase    = "Replay";
	s.spacecraft.ngc.disturbancesOn = cfg.disturbancesOn;

	// Synchronize synthetic replay trajectory across relative and spacecraft states
	const double replayGap = 0.9 + 0.5 * std::sin(0.25 * m_time);
	s.relative.pose.position = {-replayGap, s.vde.positionM.x, s.vde.positionM.y};
	s.spacecraft.chaser.eciOrientationRad = s.vde.orientationRad;
	s.spacecraft.target.eciOrientationRad = {0.0, 0.0, 0.0};
	return s;
}

// ============================================================================
// AppState
// ============================================================================

namespace {
// Helper template for visiting std::variant commands with inline lambdas.
template <class... Fs> struct Overloaded : Fs... { using Fs::operator()...; };
}

// Seeds default static mass/NGC fields into `m_live` so the UI displays sensible defaults before the engine connects.
AppState::AppState(RunConfig initialConfig) : m_runConfig(std::move(initialConfig)) {
	m_live.spacecraft.chaser.massKg      = m_runConfig.chaserMassKg;
	m_live.spacecraft.chaser.inertiaKgM2 = m_runConfig.chaserInertiaKgM2;
	m_live.spacecraft.target.massKg      = m_runConfig.targetMassKg;
	m_live.spacecraft.target.inertiaKgM2 = m_runConfig.targetInertiaKgM2;
	m_live.spacecraft.ngc.mode           = m_runConfig.ngcMode;
	m_live.spacecraft.ngc.failureCase    = "None (nominal)";
	m_live.spacecraft.ngc.disturbancesOn = m_runConfig.disturbancesOn;
	m_live.relative.pose.position.x      = m_runConfig.alongTrackSepM;
}

// Opens the UDP sockets and immediately transmits the initial RunConfig packet to the Dynamics Engine.
ErrorCode AppState::connectClient(const std::string& host, std::uint16_t configPort, std::uint16_t statePort) {
	m_clientStats.host       = host;
	m_clientStats.configPort = configPort;
	m_clientStats.statePort  = statePort;
	const ErrorCode err      = m_client.connect(host, configPort, statePort);
	m_clientStats.connected  = (err == ErrorCode::Ok);
	if (m_clientStats.connected) m_client.sendRunConfig(m_runConfig);
	return err;
}

// Per-frame tick: flushes queued UI commands, drains latest UDP telemetry, and updates offline replay if active.
void AppState::update(double frameDtS) {
	assert(std::isfinite(frameDtS) && frameDtS >= 0.0);

	std::vector<Command> pending;
	pending.swap(m_pending);
	for (const Command& c : pending) apply(c);

	if (m_clientStats.connected) {
		m_client.getLatestState(m_live);
		m_clientStats.hasState        = m_client.hasState();
		m_clientStats.packetsReceived = m_client.packetsReceived();
		m_clientStats.lastSequence    = m_client.lastSequence();
	}

	if (m_replayActive) {
		m_replayPlayer.update(frameDtS);
		m_replay       = m_replayPlayer.snapshot(m_runConfig);
		m_replayStatus = {m_replayPlayer.isPlaying(), m_replayPlayer.time(), m_replayPlayer.duration(), m_replayPlayer.speed()};
	} else {
		m_replayStatus = {};
	}
}

// Dispatches a single UI command variant to update configuration, replay state, or recording state.
void AppState::apply(const Command& command) {
	std::visit(Overloaded{
		[&](const StartRunCmd& c) {
			m_runConfig = c.config;
			if (m_clientStats.connected) m_client.sendRunConfig(m_runConfig);
		},
		[&](const StartReplayCmd& c) {
			if (!m_replayPlayer.open(c.csvPath)) return;
			m_replayActive = true;
			m_replayTarget = c.target;
			m_replayFile   = c.csvPath;
			m_replayPlayer.seek(0.0);
			m_replayPlayer.play();
			m_replay = m_replayPlayer.snapshot(m_runConfig);
		},
		[&](const StopReplayCmd&) {
			m_replayPlayer.pause();
			m_replayPlayer.close();
			m_replayActive = false;
		},
		[&](const ReplayPlayCmd&)            { if (m_replayActive) m_replayPlayer.play(); },
		[&](const ReplayPauseCmd&)           { if (m_replayActive) m_replayPlayer.pause(); },
		[&](const ReplaySeekCmd& c)          { if (m_replayActive) { m_replayPlayer.seek(c.timeS); m_replay = m_replayPlayer.snapshot(m_runConfig); } },
		[&](const ReplaySetSpeedCmd& c)      { if (m_replayActive) m_replayPlayer.setSpeed(c.factor); },
		[&](const ToggleRecordingCmd& c) {
			Recorder& rec = recorder(c.view);
			if (rec.isRecording()) rec.stop();
			else rec.startRecording(c.fileTag + "_" + fileTimestamp() + ".mp4");
		},
		[&](const ExportDataCmd&) {},
	}, command);
}

// Loads simulation_config.xml from disk if accessible, or falls back to human-readable XML summary.
std::string AppState::loadConfigXml() const {
	std::ifstream f(resolveConfigPath("simulation_config.xml"));
	if (f.is_open()) {
		std::stringstream ss;
		ss << f.rdbuf();
		return ss.str();
	}

	char buf[2048];
	std::snprintf(buf, sizeof(buf),
		R"XML(<?xml version="1.0" encoding="UTF-8"?>
<simulation engine_rate_hz="1000" render_rate_hz="120"
            server_host="%s" config_port="%u" state_port="%u">
  <spacecraft id="Chaser" mass_kg="%.1f" model="chaser_model.step"/>
  <spacecraft id="Target" mass_kg="%.1f" model="target_model.step"/>
  <orbit frame="ECI" radius_km="%.1f" inclination_deg="%.1f" along_track_sep_m="%.1f"/>
  <ngc mode="%s" disturbances="%s"/>
  <vde from_frame="B_sp" to_frame="P_sp" frame_mapping="%s">
    <limits x_mm="-100..100" y_mm="-100..100" z_mm="0..550" roll_deg="±5" pitch_deg="±5" yaw_deg="±10"/>
  </vde>
</simulation>
)XML",
		m_clientStats.host.c_str(), m_clientStats.configPort, m_clientStats.statePort,
		m_runConfig.chaserMassKg, m_runConfig.targetMassKg,
		m_runConfig.orbitRadiusM * units::kKmPerM, m_runConfig.inclinationRad * units::kDegPerRad,
		m_runConfig.alongTrackSepM, m_runConfig.ngcMode.c_str(),
		m_runConfig.disturbancesOn ? "true" : "false",
		m_runConfig.frameMapping == FrameMapping::Flipped ? "Flipped" : "Default");
	return buf;
}

} // namespace vde
