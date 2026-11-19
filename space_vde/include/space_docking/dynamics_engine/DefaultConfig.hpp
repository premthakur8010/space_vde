/**
 * @file DefaultConfig.hpp
 * @brief Compiled-in, tamper-proof factory defaults for the simulation configuration.
 *
 * WHY THIS EXISTS:
 * - config/default.xml is the "factory reset" reference used by the UI's REVERT TO DEFAULT
 *   action and by operators as a known-good template.
 * - Because it is a plain text file, it can be edited (accidentally or deliberately).
 * - The single source of truth is therefore the string kDefaultConfigXml below, compiled
 *   into every binary. default.xml on disk is only a generated mirror of it.
 *
 * TAMPER PROTECTION:
 * - ensureDefaultConfigFile() is called on startup by both physics_engine and vde_sim.
 * - If default.xml is missing or its contents differ from kDefaultConfigXml, the file is
 *   rewritten from the compiled-in copy and a warning is printed.
 * - Line endings are ignored during comparison (CRLF vs LF), so Git autocrlf checkouts on
 *   Windows are not reported as tampering.
 *
 * TO CHANGE THE DEFAULTS: edit kDefaultConfigXml below and rebuild. Editing default.xml
 * directly has no lasting effect; it is regenerated on the next launch.
 *
 * Header-only so it can be shared by the Dynamics Engine and the UI client without
 * additional link dependencies.
 */

#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <string_view>
#include <system_error>

namespace space_docking::dynamics::default_config
{
	/// File name of the generated factory-defaults mirror (lives next to simulation_config.xml).
	inline constexpr const char* kDefaultConfigFileName = "default.xml";

	/// Canonical factory-default configuration. This is the ONLY place default values are defined.
	inline constexpr std::string_view kDefaultConfigXml = R"VDEXML(<?xml version="1.0" encoding="UTF-8"?>
<!--
    Space Docking Simulation Configuration (VDE / Dynamics Engine)
    
    This configuration file sets up all physical properties, initial conditions,
    hardware-in-the-loop (HIL) platform frame mappings, and output logging destinations
    without requiring recompilation of C++ binaries.
-->
<simulationconfig>
    <!-- ===================================================================== -->
    <!-- Spacecraft Physical Properties & Initial Flight Conditions            -->
    <!-- ===================================================================== -->
    <spacecraft>
        <!-- Chaser Spacecraft (Active Docking Vehicle) -->
        <chaser>
            <!-- Spacecraft dry/wet mass in kilograms (loaded dynamically via setup) -->
            <masskg>50.0</masskg>

            <!-- Principal moments of inertia in kg*m^2 (diagonal tensor [Ixx, Iyy, Izz]) -->
            <inertia>
                <ixx>5.0</ixx>
                <iyy>5.0</iyy>
                <izz>10.0</izz>
            </inertia>

            <!-- Initial position in millimetres (mm), expressed in the VDE base platform (B_sp = 0,0,0) reference frame -->
            <!-- Chaser starts at 100 mm (0.10 m) above the base platform; Target is at 650 mm (550 mm docking stroke) -->
            <initialposition>
                <x>100.0</x>
                <y>0.0</y>
                <z>0.0</z>
            </initialposition>

            <!-- Initial linear velocity in m/s -->
            <!-- Positive Vx represents closing approach velocity towards target -->
            <initialvelocity>
                <vx>0.05</vx>
                <vy>0.0</vy>
                <vz>0.0</vz>
            </initialvelocity>
        </chaser>

        <!-- Target Spacecraft (Passive / Station Vehicle) -->
        <target>
            <!-- Spacecraft mass in kilograms -->
            <masskg>100.0</masskg>

            <!-- Principal moments of inertia in kg*m^2 -->
            <inertia>
                <ixx>10.0</ixx>
                <iyy>10.0</iyy>
                <izz>20.0</izz>
            </inertia>

            <!-- Target position in mm relative to VDE base platform (B_sp = 0,0,0) -->
            <!-- Placed at 650 mm (0.65 m) so initial separation from Chaser (100 mm) is 550 mm -->
            <initialposition>
                <x>650.0</x>
                <y>0.0</y>
                <z>0.0</z>
            </initialposition>

            <!-- Target initial linear velocity in m/s (station-keeping at 0 m/s) -->
            <initialvelocity>
                <vx>0.0</vx>
                <vy>0.0</vy>
                <vz>0.0</vz>
            </initialvelocity>
        </target>
    </spacecraft>

    <!-- ===================================================================== -->
    <!-- Hardware-in-the-Loop (HIL) Stewart Platform Mounting Mapping          -->
    <!-- ===================================================================== -->
    <platform>
        <!-- 
            Frame Mapping Mode:
            - default: Chaser mounted on Ground Stewart Platform (GSP), 
                       Target mounted on Flight Stewart Platform (FSP).
            - flipped: Target mounted on Ground Stewart Platform (GSP), 
                       Chaser mounted on Flight Stewart Platform (FSP).
        -->
        <framemapping>default</framemapping>
    </platform>

    <!-- ===================================================================== -->
    <!-- UDP Network Endpoints (Dynamics Engine <-> Visualization Client)      -->
    <!-- ===================================================================== -->
    <network>
        <!-- Destination IP address for UDP telemetry broadcast (e.g. 127.0.0.1 for local workstation) -->
        <targethost>127.0.0.1</targethost>

        <!-- UDP port on which Dynamics Engine receives RunConfigPacket from UI -->
        <configport>5001</configport>

        <!-- UDP port to which Dynamics Engine streams StatePacket telemetry to UI -->
        <stateport>5002</stateport>

        <!-- Hardware-in-the-Loop (HIL) VDE hardware / Hardware Emulator endpoints -->
        <hardwareenabled>true</hardwareenabled>
        <hardwarehost>127.0.0.1</hardwarehost>
        <!-- UDP port on which VDE Hardware / Emulator receives HardwareCommandPacket (105 bytes) -->
        <hardwarecommandport>5006</hardwarecommandport>
        <!-- UDP port on which Dynamics Engine receives HardwareTelemetryPacket (225 bytes) -->
        <hardwaretelemetryport>5005</hardwaretelemetryport>
    </network>

    <!-- ===================================================================== -->
    <!-- Simulation Logging & Output Provision                                 -->
    <!-- ===================================================================== -->
    <!-- 
        Configures where simulation state outputs (relative pose, velocities,
        thruster forces, and timestamped telemetry) will be written.
    -->
    <logging>
        <!-- Enable or disable disk logging (true / false) -->
        <enabled>true</enabled>

        <!-- Output directory where log files will be saved -->
        <logdirectory>logs</logdirectory>

        <!-- Primary output log file name for trajectory state recording -->
        <logfilename>simulation_trajectory.csv</logfilename>

        <!-- 
            Logging decimation rate: 
            Specifies how often output is recorded from the 1000 Hz physics loop.
            e.g. 10 = records every 10 ms (100 Hz output data rate).
        -->
        <logdecimation>10</logdecimation>
    </logging>
</simulationconfig>
)VDEXML";

	/// Outcome of a default.xml integrity check.
	enum class DefaultFileStatus
	{
		Intact,          // File matched the compiled-in defaults; nothing written
		RestoredMissing, // File did not exist and was created
		RestoredTampered,// File differed from the compiled-in defaults and was overwritten
		WriteFailed      // File was missing/tampered but could not be (re)written
	};

	/// Removes carriage returns so CRLF and LF files compare equal.
	[[nodiscard]] inline std::string normalizeLineEndings(std::string_view text)
	{
		std::string out;
		out.reserve(text.size());
		for (const char c : text)
		{
			if (c != '\r')
			{
				out.push_back(c);
			}
		}
		return out;
	}

	/**
	 * @brief Verifies default.xml at @p path against kDefaultConfigXml and regenerates it if needed.
	 * @param path    Full path to default.xml.
	 * @param quiet   When false, prints a console warning whenever the file is restored.
	 */
	inline DefaultFileStatus ensureDefaultConfigFile(const std::filesystem::path& path, bool quiet = false)
	{
		namespace fs = std::filesystem;
		std::error_code ec;

		const bool existed = fs::is_regular_file(path, ec);
		if (existed)
		{
			std::ifstream in(path, std::ios::binary);
			const std::string current((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
			if (normalizeLineEndings(current) == normalizeLineEndings(kDefaultConfigXml))
			{
				return DefaultFileStatus::Intact;
			}
		}

		if (path.has_parent_path())
		{
			fs::create_directories(path.parent_path(), ec);
		}

		// default.xml may have been made read-only; restore write permission before overwriting.
		if (existed)
		{
			fs::permissions(path, fs::perms::owner_write, fs::perm_options::add, ec);
		}

		std::ofstream out(path, std::ios::binary | std::ios::trunc);
		if (!out.is_open())
		{
			if (!quiet)
			{
				std::cerr << "[DEFAULT CONFIG ERROR] Could not write factory defaults to " << path.string() << std::endl;
			}
			return DefaultFileStatus::WriteFailed;
		}
		out.write(kDefaultConfigXml.data(), static_cast<std::streamsize>(kDefaultConfigXml.size()));
		out.close();

		if (!quiet)
		{
			if (existed)
			{
				std::cerr << "[DEFAULT CONFIG WARNING] Tampering detected: " << path.string()
				          << " did not match the built-in factory defaults and has been restored." << std::endl;
			}
			else
			{
				std::cout << "[DEFAULT CONFIG] " << path.string()
				          << " was missing and has been regenerated from the built-in factory defaults." << std::endl;
			}
		}
		return existed ? DefaultFileStatus::RestoredTampered : DefaultFileStatus::RestoredMissing;
	}

	/// Convenience overload: checks default.xml in the same directory as simulation_config.xml.
	inline DefaultFileStatus ensureDefaultConfigNextTo(const std::filesystem::path& simulationConfigPath, bool quiet = false)
	{
		const std::filesystem::path dir = simulationConfigPath.has_parent_path()
		    ? simulationConfigPath.parent_path()
		    : std::filesystem::path(".");
		return ensureDefaultConfigFile(dir / kDefaultConfigFileName, quiet);
	}
}
