#include "space_docking/network/Protocol.hpp"
#include "space_docking/network/UdpSocket.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <thread>

#define REQUIRE(cond) \
	do { \
		if (!(cond)) { \
			std::cerr << "FAILED REQUIRE(" #cond ") at " << __FILE__ << ":" << __LINE__ << std::endl; \
			std::exit(1); \
		} \
	} while (false)

int main()
{
	using namespace space_docking::network;
	using namespace space_docking::network::vde_net;

	RunConfigPacket cfg{};
	REQUIRE(cfg.magic == kProtocolMagic);
	REQUIRE(cfg.version == kProtocolVersion);
	REQUIRE(sizeof(RunConfigPacket) == 261);

	StatePacket state{};
	REQUIRE(state.magic == kProtocolMagic);
	REQUIRE(state.version == kProtocolVersion);
	REQUIRE(sizeof(StatePacket) == 1049);

	HardwareCommandPacket hwCmd{};
	REQUIRE(hwCmd.magic == kProtocolMagic);
	REQUIRE(hwCmd.version == kProtocolVersion);
	REQUIRE(sizeof(HardwareCommandPacket) == 105);

	HardwareTelemetryPacket hwTel{};
	REQUIRE(hwTel.magic == kProtocolMagic);
	REQUIRE(hwTel.version == kProtocolVersion);
	REQUIRE(sizeof(HardwareTelemetryPacket) == 225);

	// Verify BSD-compatible UdpSocket loopback send/recv across non-blocking sockets
	NetSystem::initialize();
	UdpSocket rx;
	UdpSocket tx;
	REQUIRE(rx.open() && rx.isValid());
	REQUIRE(rx.setReuseAddr(true));
	REQUIRE(rx.setNonBlocking(true));
	REQUIRE(tx.open() && tx.isValid());
	REQUIRE(tx.setNonBlocking(true));

	uint16_t boundPort = 0;
	for (uint16_t candidate = 55120; candidate <= 55140; ++candidate)
	{
		if (rx.bindSocket(candidate, "127.0.0.1"))
		{
			boundPort = candidate;
			break;
		}
	}
	REQUIRE(boundPort != 0);

	cfg.chaserMassKg = 1234.5;
	cfg.targetMassKg = 6789.0;
	const int sent = tx.sendTo(&cfg, sizeof(cfg), "127.0.0.1", boundPort);
	REQUIRE(sent == static_cast<int>(sizeof(cfg)));

	RunConfigPacket rxCfg{};
	int received = -1;
	for (int attempt = 0; attempt < 50; ++attempt)
	{
		received = rx.recvFrom(&rxCfg, sizeof(rxCfg));
		if (received > 0)
		{
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	REQUIRE(received == static_cast<int>(sizeof(rxCfg)));
	REQUIRE(rxCfg.magic == kProtocolMagic);
	REQUIRE(rxCfg.chaserMassKg == 1234.5);
	REQUIRE(rxCfg.targetMassKg == 6789.0);

	// Verify master HIL HardwareCommandPacket and HardwareTelemetryPacket loopback round-trip
	hwCmd.sequence = 42;
	hwCmd.timeS = 1.25;
	hwCmd.commandMode = static_cast<std::uint8_t>(HardwareCommandMode::TRACK_POSE);
	hwCmd.relativePose.position = { -0.250, 0.010, -0.005 };
	hwCmd.relativePose.orientation = { 1.0, 0.0, 0.0, 0.0 };
	hwCmd.relativeVelocity = { 0.05, 0.0, 0.0 };
	REQUIRE(tx.sendTo(&hwCmd, sizeof(hwCmd), "127.0.0.1", boundPort) == static_cast<int>(sizeof(hwCmd)));

	HardwareCommandPacket rxHwCmd{};
	received = -1;
	for (int attempt = 0; attempt < 50; ++attempt)
	{
		received = rx.recvFrom(&rxHwCmd, sizeof(rxHwCmd));
		if (received > 0)
		{
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	REQUIRE(received == static_cast<int>(sizeof(rxHwCmd)));
	REQUIRE(rxHwCmd.magic == kProtocolMagic);
	REQUIRE(rxHwCmd.sequence == 42);
	REQUIRE(rxHwCmd.relativePose.position.x == -0.250);
	REQUIRE(rxHwCmd.relativeVelocity.x == 0.05);

	hwTel.sequence = 77;
	hwTel.gspForceTorque.torque = { 0.5, -0.2, 0.1 }; // Torque FIRST
	hwTel.gspForceTorque.force = { -120.0, 3.0, -2.0 }; // Force SECOND
	hwTel.legLengths[0] = 0.412;
	hwTel.contactActive = 1;
	REQUIRE(tx.sendTo(&hwTel, sizeof(hwTel), "127.0.0.1", boundPort) == static_cast<int>(sizeof(hwTel)));

	HardwareTelemetryPacket rxHwTel{};
	received = -1;
	for (int attempt = 0; attempt < 50; ++attempt)
	{
		received = rx.recvFrom(&rxHwTel, sizeof(rxHwTel));
		if (received > 0)
		{
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	REQUIRE(received == static_cast<int>(sizeof(rxHwTel)));
	REQUIRE(rxHwTel.magic == kProtocolMagic);
	REQUIRE(rxHwTel.sequence == 77);
	REQUIRE(rxHwTel.gspForceTorque.torque.x == 0.5);
	REQUIRE(rxHwTel.gspForceTorque.force.x == -120.0);
	REQUIRE(rxHwTel.legLengths[0] == 0.412);
	REQUIRE(rxHwTel.contactActive == 1);

	std::cout << "[PASS] TestProtocol completed successfully (261B RunConfigPacket, 1049B StatePacket, 105B HardwareCommandPacket, 225B HardwareTelemetryPacket & UdpSocket loopback verified).\n";
	return 0;
}
