# Network Architecture & Netcode Guidelines

## 1. Network Protocol
- **Transport**: UDP Datagram Sockets.
- **IPC Loopback / LAN**:
	- Telemetry Multicast: Default group `239.255.0.1`, port `9870`.
	- Command Unicast: Default host `127.0.0.1`, port `9871`.
- **Packet Structure**: Packed C-style structures (`#pragma pack(push, 1)`) with 32-bit magic identifiers and validation fields.

## 2. Decoupling & Synchronization
- The dynamic engine dispatches `SnapshotMsg` at the conclusion of each physics step (or throttled interval).
- Clients maintain a ring buffer or latching slot of the latest received `SnapshotMsg` packets.
- Telemetry drop handling: UDP is intentionally lossy; client displays state continuously without head-of-line blocking.
- Commands are processed synchronously by the simulation engine at safe points in the frame loop.
