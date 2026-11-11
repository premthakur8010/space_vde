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

	std::cout << "[PASS] TestProtocol completed successfully (261-byte RunConfigPacket, 1049-byte StatePacket & UdpSocket loopback verified).\n";
	return 0;
}
