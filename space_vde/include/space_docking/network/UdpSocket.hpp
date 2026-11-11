#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

/**
 * @brief Cross-platform non-blocking UDP datagram socket wrapper using standard BSD socket semantics.
 * Uses native POSIX sockets on Ubuntu/Linux and maps onto standard socket calls on Windows.
 * Preconditions: Sockets must be bound or connected before transmit/receive.
 * Thread-safety: Individual calls are safe, but concurrent recv/send on the same instance should be managed by caller.
 */
namespace space_docking::network
{
	class NetSystem
	{
	public:
		static void initialize();
	};

	class UdpSocket
	{
	public:
		UdpSocket();
		~UdpSocket();

		UdpSocket(const UdpSocket&) = delete;
		UdpSocket& operator=(const UdpSocket&) = delete;

		UdpSocket(UdpSocket&& other) noexcept;
		UdpSocket& operator=(UdpSocket&& other) noexcept;

		bool open();
		void close();

		[[nodiscard]] bool isValid() const noexcept;
		bool setReuseAddr(bool enable = true);
		bool setNonBlocking(bool nonBlocking);
		bool setMulticastLoop(bool enable);
		bool joinMulticastGroup(const std::string& groupIp);

		bool bindSocket(uint16_t port, const std::string& host = "127.0.0.1");
		int sendTo(const void* data, size_t size, const std::string& host, uint16_t port);
		int recvFrom(void* buffer, size_t maxSize);

	private:
		std::uintptr_t m_socket;
	};
}
