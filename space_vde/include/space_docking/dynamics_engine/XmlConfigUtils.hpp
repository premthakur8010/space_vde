/**
 * @file XmlConfigUtils.hpp
 * @brief Lightweight ISO C++20 XML parsing helpers shared by the Dynamics Engine and UI client.
 */

#pragma once

#include "XmlTags.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>

namespace space_docking::dynamics::xml_utils
{
	/**
	 * @brief Trims leading and trailing whitespace while preserving original character case.
	 */
	[[nodiscard]] inline std::string trim(std::string_view sv)
	{
		const auto start = sv.find_first_not_of(" \t\r\n");
		if (start == std::string_view::npos)
		{
			return "";
		}
		const auto end = sv.find_last_not_of(" \t\r\n");
		return std::string(sv.substr(start, end - start + 1));
	}

	/**
	 * @brief Returns lowercase copy of string_view with leading/trailing whitespace removed.
	 */
	[[nodiscard]] inline std::string toLowerTrimmed(std::string_view sv)
	{
		const std::string trimmed = trim(sv);
		std::string result;
		result.reserve(trimmed.size());
		for (char c : trimmed)
		{
			result.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
		}
		return result;
	}

	/**
	 * @brief Removes XML comment blocks (`<!-- ... -->`) so commented tags are ignored.
	 */
	[[nodiscard]] inline std::string stripXmlComments(std::string_view xml)
	{
		std::string clean;
		clean.reserve(xml.size());
		size_t pos = 0;
		while (pos < xml.size())
		{
			const size_t open = xml.find("<!--", pos);
			if (open == std::string_view::npos)
			{
				clean.append(xml.substr(pos));
				break;
			}
			clean.append(xml.substr(pos, open - pos));
			const size_t close = xml.find("-->", open + 4);
			if (close == std::string_view::npos)
			{
				break;
			}
			pos = close + 3;
		}
		return clean;
	}

	/**
	 * @brief Finds substring case-insensitively and returns its byte offset in haystack.
	 */
	[[nodiscard]] inline size_t findCaseInsensitive(std::string_view haystack, std::string_view needle, size_t pos = 0)
	{
		if (needle.empty())
		{
			return pos <= haystack.size() ? pos : std::string_view::npos;
		}
		if (pos + needle.size() > haystack.size())
		{
			return std::string_view::npos;
		}

		auto it = std::search(
			haystack.begin() + static_cast<std::ptrdiff_t>(pos), haystack.end(),
			needle.begin(), needle.end(),
			[](char c1, char c2) {
				return std::tolower(static_cast<unsigned char>(c1)) ==
				       std::tolower(static_cast<unsigned char>(c2));
			});

		if (it == haystack.end())
		{
			return std::string_view::npos;
		}
		return static_cast<size_t>(std::distance(haystack.begin(), it));
	}

	/**
	 * @brief Extracts text between `<tagName>` and `</tagName>` (case-insensitive tag matching).
	 */
	[[nodiscard]] inline std::string extractTagContent(const std::string& xml, const std::string& tagName)
	{
		const std::string openTag = "<" + tagName + ">";
		const std::string closeTag = "</" + tagName + ">";

		const size_t start = findCaseInsensitive(xml, openTag);
		if (start == std::string::npos)
		{
			return "";
		}

		const size_t contentStart = start + openTag.length();
		const size_t end = findCaseInsensitive(xml, closeTag, contentStart);
		if (end == std::string::npos)
		{
			return "";
		}

		return xml.substr(contentStart, end - contentStart);
	}

	/**
	 * @brief Extracts floating-point numeric tag content, falling back to defaultValue if missing/invalid.
	 */
	[[nodiscard]] inline double extractDouble(const std::string& xml, const std::string& tagName, double defaultValue = 0.0)
	{
		const std::string content = extractTagContent(xml, tagName);
		if (content.empty())
		{
			return defaultValue;
		}
		try
		{
			return std::stod(content);
		}
		catch (...)
		{
			return defaultValue;
		}
	}

	/**
	 * @brief Validates and parses a TCP/UDP port string in the range [1, 65535].
	 */
	[[nodiscard]] inline bool parsePort(std::string_view raw, uint16_t& outPort)
	{
		const std::string s = trim(raw);
		if (s.empty() || !std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isdigit(c) != 0; }))
		{
			return false;
		}
		try
		{
			const unsigned long parsed = std::stoul(s);
			if (parsed == 0 || parsed > 65535ul)
			{
				return false;
			}
			outPort = static_cast<uint16_t>(parsed);
			return true;
		}
		catch (...)
		{
			return false;
		}
	}

	/**
	 * @brief Parses `<network>` (`<targethost>`, `<configport>`, `<stateport>`) from XML text.
	 * Retains existing values if `<network>` or individual sub-tags are omitted.
	 */
	[[nodiscard]] inline bool parseNetworkBlock(const std::string& xml,
	                                            std::string& outTargetHost,
	                                            uint16_t& outConfigPort,
	                                            uint16_t& outStatePort,
	                                            std::string& outError)
	{
		const std::string netBlock = extractTagContent(xml, xml_tags::kNetwork);
		if (netBlock.empty())
		{
			return true;
		}

		const std::string rawHost = trim(extractTagContent(netBlock, xml_tags::kTargetHost));
		if (!rawHost.empty())
		{
			outTargetHost = rawHost;
		}

		const std::string rawConfigPort = trim(extractTagContent(netBlock, xml_tags::kConfigPort));
		if (!rawConfigPort.empty() && !parsePort(rawConfigPort, outConfigPort))
		{
			outError = "Invalid <" + std::string(xml_tags::kConfigPort) + "> port value: '" + rawConfigPort + "' (expected 1..65535)";
			return false;
		}

		const std::string rawStatePort = trim(extractTagContent(netBlock, xml_tags::kStatePort));
		if (!rawStatePort.empty() && !parsePort(rawStatePort, outStatePort))
		{
			outError = "Invalid <" + std::string(xml_tags::kStatePort) + "> port value: '" + rawStatePort + "' (expected 1..65535)";
			return false;
		}

		return true;
	}

	/**
	 * @brief Reads UDP endpoints from an XML config file on disk if accessible and valid.
	 */
	inline bool loadNetworkEndpointsFromFile(const std::string& filePath,
	                                         std::string& outTargetHost,
	                                         uint16_t& outConfigPort,
	                                         uint16_t& outStatePort)
	{
		std::ifstream file(filePath);
		if (!file.is_open())
		{
			return false;
		}
		std::stringstream buffer;
		buffer << file.rdbuf();
		const std::string cleanXml = stripXmlComments(buffer.str());
		if (cleanXml.empty())
		{
			return false;
		}
		std::string err;
		return parseNetworkBlock(cleanXml, outTargetHost, outConfigPort, outStatePort, err);
	}

	/**
	 * @brief Resolves `config/<fileName>` dynamically from CWD or an extra search root.
	 * Checks `explicitPath` first if set, then walks up ancestor directories looking for
	 * `config/<fileName>` or `space_vde/config/<fileName>` without hardcoded folder names.
	 */
	[[nodiscard]] inline std::string resolveConfigFilePath(std::string_view fileName,
	                                                         std::string_view explicitPath = {},
	                                                         std::string_view extraSearchRoot = {})
	{
		namespace fs = std::filesystem;
		std::error_code ec;

		if (!explicitPath.empty())
		{
			const fs::path explicitFile(explicitPath);
			if (fs::is_regular_file(explicitFile, ec))
			{
				return explicitFile.lexically_normal().generic_string();
			}
		}

		const fs::path targetSuffix = fs::path("config") / fileName;
		const fs::path repoSuffix   = fs::path("space_vde") / targetSuffix;

		auto matchUnder = [&](const fs::path& base, std::string& out) -> bool
		{
			for (const fs::path& suffix : {targetSuffix, repoSuffix})
			{
				const fs::path candidate = (base.empty() ? suffix : (base / suffix)).lexically_normal();
				if (fs::is_regular_file(candidate, ec))
				{
					out = candidate.generic_string();
					return true;
				}
			}
			return false;
		};

		// Walk upward from CWD using relative prefixes ("", "..", "../..", "../../..")
		std::string resolved;
		fs::path relPrefix;
		for (int depth = 0; depth < 4; ++depth)
		{
			if (matchUnder(relPrefix, resolved))
			{
				return resolved;
			}
			relPrefix = relPrefix.empty() ? fs::path("..") : (relPrefix / "..");
		}

		// Fallback: walk upward from extraSearchRoot (e.g. executable directory) when launched externally
		if (!extraSearchRoot.empty())
		{
			fs::path absDir(extraSearchRoot);
			for (int depth = 0; depth < 4 && !absDir.empty(); ++depth)
			{
				if (matchUnder(absDir, resolved))
				{
					return resolved;
				}
				const fs::path parent = absDir.parent_path();
				if (parent == absDir)
				{
					break;
				}
				absDir = parent;
			}
		}

		return targetSuffix.generic_string();
	}
}

