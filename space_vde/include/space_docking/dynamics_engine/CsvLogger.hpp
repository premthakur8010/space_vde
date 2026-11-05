/**
 * @file CsvLogger.hpp
 * @brief High-frequency asynchronous CSV telemetry and visualization replay logger.
 *
 * PURPOSE AND ARCHITECTURE:
 * -------------------------
 * This module records simulation trajectory states into CSV format for post-simulation analysis
 * and accurate 3D visualization replay (at 60 Hz or higher).
 *
 * Key Architectural Highlights:
 * 1. Zero Real-Time Latency Jitter:
 *    Writing directly to disk inside the 1 kHz (1 ms) physics loop can cause unexpected 5-50 ms
 *    OS stalls (due to disk cache flushes, NTFS journaling, and background virus scanners).
 *    To maintain deterministic 1 ms timing, CsvLogger dispatches telemetry data to a low-priority
 *    asynchronous worker thread via a fast in-memory queue (<50 ns latency per push).
 *
 * 2. 10 ms Decimation and True Net State:
 *    The physics engine integrates dynamics at 1 kHz (every 1 ms). Every 10 ms (100 Hz), the
 *    TelemetryAccumulator samples the cumulative integrated state (positions, quaternions, velocities).
 *    Because this state is recorded at the 10th millisecond, it represents the exact net accumulation
 *    of the 10 preceding physics steps—providing full kinematic fidelity for 3D visualization replay.
 *
 * 3. Self-Contained Dependency:
 *    This module depends exclusively on Types.hpp and standard C++ libraries, allowing it to be
 *    deployed or pushed independently without requiring the DynamicsEngine coordinator.
 */

#pragma once

#include "Types.hpp"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <ostream>
#include <queue>
#include <string>
#include <thread>
#include <vector>

namespace space_docking::dynamics
{
	/**
	 * @struct TelemetryLogEntry
	 * @brief Plain-Old-Data snapshot containing the exact kinematic states required for 3D visualization replay.
	 *
	 * Each entry maps directly to one row in the output trajectory CSV file, enabling 3D renderers
	 * (e.g. OpenGL 4.5 / ImGui) to animate the Chaser and Target meshes at 60 FPS.
	 */
	struct TelemetryLogEntry
	{
		double timestamp{0.0};       ///< Simulation elapsed time in seconds (s)
		uint64_t cycleCount{0};      ///< Simulation cycle counter (10, 20, 30, ...)

		// Chaser 3D Transform in Target/Inertial Reference Frame (for Chaser mesh placement)
		Pose chaserPose{};           ///< Chaser position (m) and orientation quaternion q(w, x, y, z)

		// Target 3D Transform in Inertial Reference Frame (for Target mesh placement)
		Pose targetPose{};           ///< Target position (m) and orientation quaternion q(w, x, y, z)

		// Relative Docking Kinematics (for camera tracking and docking corridor verification)
		Pose relativePose{};         ///< Relative position (m) and relative orientation quaternion
		Vector3 relativeVelocity{};  ///< Relative linear closing velocity (m/s)

		// Relative Attitude in Euler Angles (for HUD telemetry gauges: Yaw, Pitch, Roll in degrees)
		EulerAngles relativeEulerDeg{};
	};

	/**
	 * @class TelemetryAccumulator
	 * @brief Decimation filter that captures the net physical state over each 10 ms window.
	 *
	 * Operates synchronously inside the 1 ms physics loop:
	 * - Receives high-rate 1 ms state updates from spacecraft dynamics.
	 * - Every 10th step, computes the net integrated state and flags readiness for CSV logging.
	 */
	class TelemetryAccumulator
	{
	public:
		/**
		 * @brief Construct accumulator with a specified decimation rate.
		 * @param decimation Number of physics cycles between log entries (e.g., 10 for 100 Hz output from 1 kHz loop).
		 */
		explicit TelemetryAccumulator(uint32_t decimation = 10);

		/**
		 * @brief Reset accumulator state and initialize window reference pose.
		 * @param initialRelativePose Starting relative pose between Chaser and Target.
		 */
		void reset(const Pose& initialRelativePose) noexcept;

		/**
		 * @brief Record a 1 ms simulation step sample.
		 * @param chaser Current Chaser body state (pose, velocity).
		 * @param target Current Target body state (pose, velocity).
		 * @param relative Current Relative docking state vector.
		 */
		void addSample(const BodyState& chaser,
		               const BodyState& target,
		               const RelativeState& relative) noexcept;

		/**
		 * @brief Returns true when the decimation window is complete (e.g. at the 10th sample).
		 */
		[[nodiscard]] bool isReady() const noexcept;

		/**
		 * @brief Computes the net cumulative entry for visualization replay and advances to next window.
		 * @param timestamp Current simulation time in seconds.
		 * @param cycleCount Current physics cycle count.
		 * @return TelemetryLogEntry Populated snapshot ready for asynchronous disk dispatch.
		 */
		[[nodiscard]] TelemetryLogEntry computeNetEntryAndAdvance(double timestamp, uint64_t cycleCount) noexcept;

		/// Number of steps in the decimation window (default: 10)
		[[nodiscard]] uint32_t getDecimation() const noexcept { return m_decimation; }

		/// Current sample count within the active window (0 to 10)
		[[nodiscard]] uint32_t getSampleCount() const noexcept { return m_sampleCount; }

	private:
		uint32_t m_decimation{10};    ///< Window size in cycles (10 = 10 ms)
		uint32_t m_sampleCount{0};    ///< Number of samples accumulated in current window
		BodyState m_latestChaser{};   ///< Most recent Chaser state (at sample 10)
		BodyState m_latestTarget{};   ///< Most recent Target state (at sample 10)
		RelativeState m_latestRelative{}; ///< Most recent Relative state (at sample 10)
	};

	/**
	 * @class CsvLogger
	 * @brief Non-blocking, asynchronous CSV logger for recording simulation trajectory files.
	 *
	 * Features:
	 * - Spawns a background thread that manages file I/O completely off the real-time physics thread.
	 * - Uses an in-memory queue protected by a mutex and condition variable with batch flushing.
	 * - Guarantees clean flush on shutdown without data loss.
	 * - Provides writeRow(vector<double>) for backward compatibility with the Software Design Document (Figure 3).
	 */
	class CsvLogger
	{
	public:
		CsvLogger();

		/**
		 * @brief Construct logger and immediately open destination file.
		 * @param filename File path for the output CSV (parent folders created automatically).
		 */
		explicit CsvLogger(const std::string& filename);

		~CsvLogger();

		// Non-copyable, movable
		CsvLogger(const CsvLogger&) = delete;
		CsvLogger& operator=(const CsvLogger&) = delete;
		CsvLogger(CsvLogger&&) noexcept;
		CsvLogger& operator=(CsvLogger&&) noexcept;

		/**
		 * @brief Open output file, start asynchronous background worker thread, and write CSV header.
		 * @param filename Destination CSV file path.
		 * @return ErrorCode::OK on success, or ErrorCode::FILE_OPEN_FAILED on failure.
		 */
		ErrorCode open(const std::string& filename);

		/**
		 * @brief Signal worker thread to finish writing queued data and cleanly close file handle.
		 */
		void close();

		/**
		 * @brief Enqueue a high-rate telemetry snapshot for background writing.
		 * Execution time: <50 ns (non-blocking). Safe to invoke from 1 kHz real-time loops.
		 * @param entry The kinematic snapshot to write.
		 */
		ErrorCode log(const TelemetryLogEntry& entry);

		/**
		 * @brief Generic double-vector row writer for SDD interface compatibility (SDD Figure 3).
		 * @param values Vector of double precision values to write comma-separated.
		 */
		ErrorCode writeRow(const std::vector<double>& values);

		/**
		 * @brief Check if logger is actively running with an open file.
		 */
		[[nodiscard]] bool isOpen() const noexcept;

		/**
		 * @brief Block until all currently queued entries are fully flushed to disk.
		 */
		void flush();

		/**
		 * @brief Total rows committed to disk since file open.
		 */
		[[nodiscard]] uint64_t getWrittenRowCount() const noexcept;

		/**
		 * @brief Current number of items pending in memory queue.
		 */
		[[nodiscard]] size_t getQueuedRowCount() const noexcept;

	private:
		void workerLoop();
		void writeHeader();
		static void writeEntryToStream(std::ostream& os, const TelemetryLogEntry& entry);
		static void writeValuesToStream(std::ostream& os, const std::vector<double>& values);

		std::string m_filePath;
		std::ofstream m_file;
		std::thread m_workerThread;
		mutable std::mutex m_mutex;
		std::condition_variable m_cv;
		std::atomic<bool> m_running{false};
		std::atomic<bool> m_isOpen{false};
		std::atomic<uint64_t> m_writtenRows{0};

		enum class QueueItemType : uint8_t { TelemetryEntry, RawValues };
		struct QueueItem
		{
			QueueItemType type{QueueItemType::TelemetryEntry};
			TelemetryLogEntry entry{};
			std::vector<double> values{};
		};

		std::queue<QueueItem> m_queue;
		static constexpr size_t kMaxQueueCapacity = 20000; ///< Queue size limit to prevent memory exhaustion
	};
}
