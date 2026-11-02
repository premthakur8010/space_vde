#include "space_docking/network/Protocol.hpp"

#include <cassert>
#include <iostream>

int main()
{
	using namespace space_docking::network;

	SnapshotMsg snapshot{};
	assert(snapshot.magic == kSnapshotMsgMagic);
	assert(snapshot.tick == 0);

	InputCommandMsg cmd{};
	assert(cmd.magic == kInputCommandMsgMagic);
	assert(cmd.command == SimCommandId::Reset);

	std::cout << "[PASS] TestProtocol completed successfully.\n";
	return 0;
}
