#include "space_docking/network/Protocol.hpp"

#include <cassert>
#include <iostream>

int main()
{
	using namespace space_docking::network::vde_net;

	RunConfigPacket cfg{};
	assert(cfg.magic == kProtocolMagic);
	assert(cfg.version == kProtocolVersion);
	assert(sizeof(RunConfigPacket) == 261);

	StatePacket state{};
	assert(state.magic == kProtocolMagic);
	assert(state.version == kProtocolVersion);
	assert(sizeof(StatePacket) == 1049);

	std::cout << "[PASS] TestProtocol completed successfully (261-byte RunConfigPacket & 1049-byte StatePacket verified).\n";
	return 0;
}
