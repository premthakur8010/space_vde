/**
 * @file generate_default_xml.cpp
 * @brief Standalone C++ generator that verifies and writes the tamper-proof config/default.xml file.
 *
 * Can be run directly:
 *   ./build/generate_default_xml [optional/path/to/default.xml]
 *
 * Also runs automatically inside physics_engine and vde_sim on startup via DefaultConfig.hpp.
 */

#include "space_docking/dynamics_engine/DefaultConfig.hpp"
#include "space_docking/dynamics_engine/XmlConfigUtils.hpp"

#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
	using namespace space_docking::dynamics;

	std::filesystem::path targetPath;
	if (argc > 1 && argv[1][0] != '\0')
	{
		targetPath = argv[1];
	}
	else
	{
		std::error_code ec;
		const std::filesystem::path exeDir = std::filesystem::weakly_canonical(std::filesystem::path(argv[0]), ec).parent_path();
		const std::string simConfig = xml_utils::resolveConfigFilePath("simulation_config.xml", {}, exeDir.string());
		const std::filesystem::path simPath(simConfig);
		targetPath = (simPath.has_parent_path() ? simPath.parent_path() : std::filesystem::path("config"))
		             / default_config::kDefaultConfigFileName;
	}

	const auto status = default_config::ensureDefaultConfigFile(targetPath, /*quiet=*/false);
	switch (status)
	{
	case default_config::DefaultFileStatus::Intact:
		std::cout << "[OK] " << targetPath.string() << " verified intact (matches compiled-in factory defaults).\n";
		return 0;
	case default_config::DefaultFileStatus::RestoredMissing:
	case default_config::DefaultFileStatus::RestoredTampered:
		std::cout << "[OK] " << targetPath.string() << " written with compiled-in factory default values.\n";
		return 0;
	case default_config::DefaultFileStatus::WriteFailed:
		return 1;
	}
	return 0;
}
