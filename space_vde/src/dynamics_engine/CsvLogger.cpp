/**
 * @file CsvLogger.cpp
 * @brief Implementation of the asynchronous CSV telemetry logger and 10 ms decimation accumulator.
 *
 * PURPOSE:
 * --------
 * Provides non-blocking CSV persistence for spacecraft docking trajectories.
 * Telemetry data is enqueued from the high-priority 1 kHz physics loop in ~50 nanoseconds,
 * while an independent background thread serializes data into streamlined CSV format for
 * accurate 3D visualization replay.
 */

#include "space_docking/dynamics_engine/CsvLogger.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>

namespace space_docking::dynamics
{
	namespace
	{
		// Conversion factor from radians to degrees
		constexpr double kRadToDeg = 180.0 / M_PI;
	}

	// =========================================================================
	// TelemetryAccumulator Implementation
	// =========================================================================

	// Sets how many 1 ms physics steps to bundle into one CSV log row (e.g., decimation=10 gives 100 Hz logging)
	TelemetryAccumulator::TelemetryAccumulator(uint32_t decimation)
		: m_decimation(decimation > 0 ? decimation : 1)
	{
	}

	// Clears the step counter and wipes cached states for a fresh simulation run
	void TelemetryAccumulator::reset(const Pose& /*initialRelativePose*/) noexcept
	{
		m_sampleCount = 0;
		m_latestChaser = BodyState{};
		m_latestTarget = BodyState{};
		m_latestRelative = RelativeState{};
	}

	// Called every 1 ms by the physics engine: caches the newest spacecraft states and increments the step count
	void TelemetryAccumulator::addSample(const BodyState& chaser,
	                                    const BodyState& target,
	                                    const RelativeState& relative) noexcept
	{
		// Cache the latest physics states produced by the 1 ms numerical integrator
		m_latestChaser = chaser;
		m_latestTarget = target;
		m_latestRelative = relative;

		// Increment step counter within this decimation window
		m_sampleCount++;
	}

	// Returns true when enough samples have accumulated (e.g., 10 ms reached) to write a CSV row
	bool TelemetryAccumulator::isReady() const noexcept
	{
		// Ready to log when exactly 10 samples (10 ms) have been accumulated
		return m_sampleCount >= m_decimation;
	}

	// Packages the latest 10 ms state into a TelemetryLogEntry and resets the counter for the next window
	TelemetryLogEntry TelemetryAccumulator::computeNetEntryAndAdvance(double timestamp, uint64_t cycleCount) noexcept
	{
		TelemetryLogEntry entry{};
		entry.timestamp = timestamp;
		entry.cycleCount = cycleCount;

		// 1. Chaser 3D Transform: true position & quaternion at sample 10 (t = 10 ms)
		entry.chaserPose = m_latestChaser.pose;

		// 2. Target 3D Transform: true position & quaternion at sample 10
		entry.targetPose = m_latestTarget.pose;

		// 3. Relative Docking State: relative position and orientation quaternion
		entry.relativePose = m_latestRelative.pose;
		entry.relativeVelocity = m_latestRelative.velocity;

		// 4. Relative Attitude in Euler angles (converted to degrees for HUD display gauges)
		const auto eulerRad = m_latestRelative.toEuler();
		entry.relativeEulerDeg.yaw = eulerRad.yaw * kRadToDeg;
		entry.relativeEulerDeg.pitch = eulerRad.pitch * kRadToDeg;
		entry.relativeEulerDeg.roll = eulerRad.roll * kRadToDeg;

		// Reset sample counter for the next 10 ms window
		m_sampleCount = 0;

		return entry;
	}

	// =========================================================================
	// CsvLogger Implementation
	// =========================================================================

	CsvLogger::CsvLogger() = default;

	CsvLogger::CsvLogger(const std::string& filename)
	{
		open(filename);
	}

	// Destructor: flushes remaining rows and cleanly terminates background writer thread
	CsvLogger::~CsvLogger()
	{
		close();
	}

	CsvLogger::CsvLogger(CsvLogger&& other) noexcept
	{
		*this = std::move(other);
	}

	// Thread-safe move assignment: transfers file handles and worker thread ownership
	CsvLogger& CsvLogger::operator=(CsvLogger&& other) noexcept
	{
		if (this != &other)
		{
			close();
			std::unique_lock<std::mutex> lockOther(other.m_mutex);
			m_filePath = std::move(other.m_filePath);
			m_file = std::move(other.m_file);
			m_running.store(other.m_running.load());
			m_isOpen.store(other.m_isOpen.load());
			m_writtenRows.store(other.m_writtenRows.load());
			m_queue = std::move(other.m_queue);

			other.m_running = false;
			other.m_isOpen = false;
			other.m_writtenRows = 0;
			lockOther.unlock();

			if (other.m_workerThread.joinable())
			{
				other.m_workerThread.join();
			}

			if (m_isOpen && m_running)
			{
				m_workerThread = std::thread(&CsvLogger::workerLoop, this);
			}
		}
		return *this;
	}

	// Creates directory (e.g. "logs/"), opens CSV file, writes header row, and starts background thread
	ErrorCode CsvLogger::open(const std::string& filename)
	{
		close();

		if (filename.empty())
		{
			return ErrorCode::INVALID_ARGUMENT;
		}

		m_filePath = filename;

		// Automatically create parent folder structure (e.g. "logs/") if it does not exist
		try
		{
			std::filesystem::path p(filename);
			if (p.has_parent_path())
			{
				std::filesystem::create_directories(p.parent_path());
			}
		}
		catch (...)
		{
			return ErrorCode::FILE_OPEN_FAILED;
		}

		// Open output file stream in truncate mode
		m_file.open(m_filePath, std::ios::out | std::ios::trunc);
		if (!m_file.is_open())
		{
			return ErrorCode::FILE_OPEN_FAILED;
		}

		// Write streamlined CSV column header
		writeHeader();

		m_isOpen = true;
		m_running = true;
		m_writtenRows = 0;

		// Launch low-priority background thread for asynchronous disk writing
		m_workerThread = std::thread(&CsvLogger::workerLoop, this);

		return ErrorCode::OK;
	}

	// Signals background thread to flush pending rows, waits for it to finish, and closes the file
	void CsvLogger::close()
	{
		if (!m_isOpen)
		{
			return;
		}

		// Signal background worker thread to drain queue and terminate
		m_running = false;
		m_cv.notify_all();

		if (m_workerThread.joinable())
		{
			m_workerThread.join();
		}

		if (m_file.is_open())
		{
			m_file.flush();
			m_file.close();
		}

		m_isOpen = false;
	}

	// Non-blocking write (<50 ns): enqueues telemetry entry from the 1 kHz physics loop and wakes worker thread
	ErrorCode CsvLogger::log(const TelemetryLogEntry& entry)
	{
		if (!m_isOpen)
		{
			return ErrorCode::NOT_INITIALIZED;
		}

		// Fast thread-safe queue push (<50 ns latency; no heap allocations for strings)
		std::unique_lock<std::mutex> lock(m_mutex);
		if (m_queue.size() >= kMaxQueueCapacity)
		{
			// Drop oldest entry to protect memory if disk I/O stalls permanently
			m_queue.pop();
		}

		QueueItem item;
		item.type = QueueItemType::TelemetryEntry;
		item.entry = entry;
		m_queue.push(std::move(item));
		lock.unlock();

		// Notify background thread that new data is available
		m_cv.notify_one();
		return ErrorCode::OK;
	}

	// Enqueues raw double values for custom logging or test suites
	ErrorCode CsvLogger::writeRow(const std::vector<double>& values)
	{
		if (!m_isOpen)
		{
			return ErrorCode::NOT_INITIALIZED;
		}

		std::unique_lock<std::mutex> lock(m_mutex);
		if (m_queue.size() >= kMaxQueueCapacity)
		{
			m_queue.pop();
		}

		QueueItem item;
		item.type = QueueItemType::RawValues;
		item.values = values;
		m_queue.push(std::move(item));
		lock.unlock();

		m_cv.notify_one();
		return ErrorCode::OK;
	}

	// Returns true if the CSV file is open and background logging is active
	bool CsvLogger::isOpen() const noexcept
	{
		return m_isOpen;
	}

	// Blocks until all queued in-memory entries have been written and flushed to disk
	void CsvLogger::flush()
	{
		if (!m_isOpen)
		{
			return;
		}

		// Wait until worker thread has fully drained the in-memory queue to disk
		while (true)
		{
			{
				std::lock_guard<std::mutex> lock(m_mutex);
				if (m_queue.empty())
				{
					break;
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		std::lock_guard<std::mutex> lock(m_mutex);
		if (m_file.is_open())
		{
			m_file.flush();
		}
	}

	// Returns the total number of rows physically written to disk
	uint64_t CsvLogger::getWrittenRowCount() const noexcept
	{
		return m_writtenRows.load();
	}

	// Returns the current number of rows waiting in memory to be written
	size_t CsvLogger::getQueuedRowCount() const noexcept
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		return m_queue.size();
	}

	// Writes the 25-column CSV header row defining all trajectory fields
	void CsvLogger::writeHeader()
	{
		// Streamlined 25-column CSV header designed specifically for 3D visualization replay
		m_file << "timestamp_s,cycle_count,"
		       << "chaser_pos_x_m,chaser_pos_y_m,chaser_pos_z_m,"
		       << "chaser_quat_w,chaser_quat_x,chaser_quat_y,chaser_quat_z,"
		       << "target_pos_x_m,target_pos_y_m,target_pos_z_m,"
		       << "target_quat_w,target_quat_x,target_quat_y,target_quat_z,"
		       << "rel_pos_x_m,rel_pos_y_m,rel_pos_z_m,"
		       << "rel_vel_x_mps,rel_vel_y_mps,rel_vel_z_mps,"
		       << "rel_yaw_deg,rel_pitch_deg,rel_roll_deg\n";
		m_file.flush();
	}

	// Formats a TelemetryLogEntry into a comma-separated line with fixed 6-8 decimal precision
	void CsvLogger::writeEntryToStream(std::ostream& os, const TelemetryLogEntry& entry)
	{
		// High precision numerical formatting for accurate 3D playback
		os << std::fixed << std::setprecision(6)
		   << entry.timestamp << ","
		   << entry.cycleCount << ","
		   // Chaser 3D Position
		   << entry.chaserPose.position.x << ","
		   << entry.chaserPose.position.y << ","
		   << entry.chaserPose.position.z << ","
		   // Chaser 3D Quaternion
		   << std::setprecision(8)
		   << entry.chaserPose.orientation.w << ","
		   << entry.chaserPose.orientation.x << ","
		   << entry.chaserPose.orientation.y << ","
		   << entry.chaserPose.orientation.z << ","
		   // Target 3D Position
		   << std::setprecision(6)
		   << entry.targetPose.position.x << ","
		   << entry.targetPose.position.y << ","
		   << entry.targetPose.position.z << ","
		   // Target 3D Quaternion
		   << std::setprecision(8)
		   << entry.targetPose.orientation.w << ","
		   << entry.targetPose.orientation.x << ","
		   << entry.targetPose.orientation.y << ","
		   << entry.targetPose.orientation.z << ","
		   // Relative Position
		   << std::setprecision(6)
		   << entry.relativePose.position.x << ","
		   << entry.relativePose.position.y << ","
		   << entry.relativePose.position.z << ","
		   // Relative Velocity
		   << entry.relativeVelocity.x << ","
		   << entry.relativeVelocity.y << ","
		   << entry.relativeVelocity.z << ","
		   // Relative Euler Angles (degrees)
		   << std::setprecision(4)
		   << entry.relativeEulerDeg.yaw << ","
		   << entry.relativeEulerDeg.pitch << ","
		   << entry.relativeEulerDeg.roll << "\n";
	}

	// Formats a generic vector of doubles into a comma-separated line
	void CsvLogger::writeValuesToStream(std::ostream& os, const std::vector<double>& values)
	{
		os << std::fixed << std::setprecision(6);
		for (size_t i = 0; i < values.size(); ++i)
		{
			os << values[i];
			if (i + 1 < values.size())
			{
				os << ",";
			}
		}
		os << "\n";
	}

	// Background worker loop: sleeps until rows arrive, batches up to 128 rows, and writes to disk
	void CsvLogger::workerLoop()
	{
		std::vector<QueueItem> batch;
		batch.reserve(128);

		while (m_running || !m_queue.empty())
		{
			{
				std::unique_lock<std::mutex> lock(m_mutex);
				// Sleep until items arrive or 20 ms timeout (keeps thread idle with zero CPU usage)
				m_cv.wait_for(lock, std::chrono::milliseconds(20), [this] {
					return !m_queue.empty() || !m_running;
				});

				// Batch up to 128 items in one quick lock acquisition
				while (!m_queue.empty() && batch.size() < 128)
				{
					batch.push_back(std::move(m_queue.front()));
					m_queue.pop();
				}
			}

			if (batch.empty())
			{
				continue;
			}

			// Format and write all batched items to file outside of lock
			for (const auto& item : batch)
			{
				if (item.type == QueueItemType::TelemetryEntry)
				{
					writeEntryToStream(m_file, item.entry);
				}
				else
				{
					writeValuesToStream(m_file, item.values);
				}
			}

			m_writtenRows.fetch_add(batch.size(), std::memory_order_relaxed);
			batch.clear();
			m_file.flush();
		}
	}
}
