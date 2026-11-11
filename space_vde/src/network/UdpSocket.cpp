/**
 * @file UdpSocket.cpp
 * @brief Cross-platform IPv4 UDP socket wrapper using standard BSD socket semantics.
 *
 * Uses native POSIX sockets (<sys/socket.h>) on Ubuntu/Linux directly. Because Windows
 * lacks native <sys/socket.h>, a thin compatibility adapter at the top maps BSD primitives
 * onto Windows sockets so every socket method shares one unified implementation.
 */

#include "space_docking/network/UdpSocket.hpp"
#include "space_docking/utils/Logger.hpp"

#include <cstring>
#include <utility>

#if defined(_WIN32)
	#ifndef WIN32_LEAN_AND_MEAN
	#define WIN32_LEAN_AND_MEAN
	#endif
	#include <winsock2.h>
	#include <ws2tcpip.h>

	namespace
	{
		using NativeSocket = SOCKET;
		using SockLen      = int;
		using MulticastOpt = int;
		constexpr std::uintptr_t kInvalidSocketHandle = static_cast<std::uintptr_t>(INVALID_SOCKET);

		inline NativeSocket toNative(std::uintptr_t h) noexcept { return static_cast<NativeSocket>(h); }
		inline bool isInvalidNative(NativeSocket fd) noexcept { return fd == INVALID_SOCKET; }
		inline void closeNativeSocket(NativeSocket fd) noexcept { ::closesocket(fd); }

		inline bool setNonBlockingNative(NativeSocket fd, bool nonBlocking) noexcept
		{
			u_long mode = nonBlocking ? 1UL : 0UL;
			return ::ioctlsocket(fd, FIONBIO, &mode) == 0;
		}

		inline void initPlatformSockets()
		{
			static bool initialized = false;
			if (!initialized)
			{
				WSADATA wsaData{};
				if (::WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
				{
					space_docking::utils::Logger::getInstance().error("Socket subsystem startup failed");
				}
				initialized = true;
			}
		}
	}
#else
	#include <arpa/inet.h>
	#include <fcntl.h>
	#include <netinet/in.h>
	#include <sys/socket.h>
	#include <sys/types.h>
	#include <unistd.h>

	namespace
	{
		using NativeSocket = int;
		using SockLen      = socklen_t;
		using MulticastOpt = unsigned char;
		constexpr std::uintptr_t kInvalidSocketHandle = ~std::uintptr_t{0};

		inline NativeSocket toNative(std::uintptr_t h) noexcept { return static_cast<NativeSocket>(h); }
		inline bool isInvalidNative(NativeSocket fd) noexcept { return fd < 0; }
		inline void closeNativeSocket(NativeSocket fd) noexcept { ::close(fd); }

		inline bool setNonBlockingNative(NativeSocket fd, bool nonBlocking) noexcept
		{
			int flags = ::fcntl(fd, F_GETFL, 0);
			if (flags < 0)
			{
				return false;
			}
			flags = nonBlocking ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
			return ::fcntl(fd, F_SETFL, flags) == 0;
		}

		inline void initPlatformSockets() noexcept {}
	}
#endif

namespace space_docking::network
{
	// Initializes OS socket subsystem once if required by the host platform (no-op on Ubuntu/Linux)
	void NetSystem::initialize()
	{
		initPlatformSockets();
	}

	// Constructor: ensures socket subsystem is ready and sets handle to invalid
	UdpSocket::UdpSocket()
		: m_socket(kInvalidSocketHandle)
	{
		NetSystem::initialize();
	}

	// Destructor: safely closes active socket handle to release OS networking resources
	UdpSocket::~UdpSocket()
	{
		close();
	}

	// Move constructor: transfers ownership of socket descriptor without closing the handle
	UdpSocket::UdpSocket(UdpSocket&& other) noexcept
		: m_socket(other.m_socket)
	{
		other.m_socket = kInvalidSocketHandle;
	}

	// Move assignment operator: closes current socket and claims ownership of incoming socket
	UdpSocket& UdpSocket::operator=(UdpSocket&& other) noexcept
	{
		if (this != &other)
		{
			close();
			m_socket = other.m_socket;
			other.m_socket = kInvalidSocketHandle;
		}
		return *this;
	}

	// Creates an IPv4 UDP datagram socket via standard BSD socket() with address reuse enabled
	bool UdpSocket::open()
	{
		close();
		const NativeSocket fd = ::socket(AF_INET, SOCK_DGRAM, 0);
		if (isInvalidNative(fd))
		{
			utils::Logger::getInstance().error("Failed to create UDP socket");
			return false;
		}

		m_socket = static_cast<std::uintptr_t>(fd);
		setReuseAddr(true);
		return true;
	}

	// Closes the underlying socket descriptor cleanly
	void UdpSocket::close()
	{
		if (m_socket != kInvalidSocketHandle)
		{
			closeNativeSocket(toNative(m_socket));
			m_socket = kInvalidSocketHandle;
		}
	}

	// Returns true if socket descriptor is open and valid
	bool UdpSocket::isValid() const noexcept
	{
		return m_socket != kInvalidSocketHandle;
	}

	// Allows immediate port rebinding across quick simulator restarts
	bool UdpSocket::setReuseAddr(bool enable)
	{
		if (m_socket == kInvalidSocketHandle)
		{
			return false;
		}
		const int opt = enable ? 1 : 0;
		const bool ok = (::setsockopt(toNative(m_socket), SOL_SOCKET, SO_REUSEADDR,
		                              reinterpret_cast<const char*>(&opt), sizeof(opt)) == 0);
#ifdef SO_REUSEPORT
		::setsockopt(toNative(m_socket), SOL_SOCKET, SO_REUSEPORT,
		             reinterpret_cast<const char*>(&opt), sizeof(opt));
#endif
		return ok;
	}

	// Switches socket to non-blocking mode so recvFrom returns immediately when no datagram is queued
	bool UdpSocket::setNonBlocking(bool nonBlocking)
	{
		if (m_socket == kInvalidSocketHandle)
		{
			return false;
		}
		return setNonBlockingNative(toNative(m_socket), nonBlocking);
	}

	// Enables or disables local loopback of outgoing multicast datagrams
	bool UdpSocket::setMulticastLoop(bool enable)
	{
		if (m_socket == kInvalidSocketHandle)
		{
			return false;
		}
		const MulticastOpt loop = enable ? 1 : 0;
		return ::setsockopt(toNative(m_socket), IPPROTO_IP, IP_MULTICAST_LOOP,
		                    reinterpret_cast<const char*>(&loop), sizeof(loop)) == 0;
	}

	// Joins an IPv4 multicast group address (e.g. "239.255.0.1") using IP_ADD_MEMBERSHIP
	bool UdpSocket::joinMulticastGroup(const std::string& groupIp)
	{
		if (m_socket == kInvalidSocketHandle)
		{
			return false;
		}
		ip_mreq mreq{};
		::inet_pton(AF_INET, groupIp.c_str(), &mreq.imr_multiaddr.s_addr);
		mreq.imr_interface.s_addr = htonl(INADDR_ANY);
		return ::setsockopt(toNative(m_socket), IPPROTO_IP, IP_ADD_MEMBERSHIP,
		                    reinterpret_cast<const char*>(&mreq), sizeof(mreq)) == 0;
	}

	// Binds the socket to a local host address and UDP port for receiving packets
	bool UdpSocket::bindSocket(uint16_t port, const std::string& host)
	{
		if (m_socket == kInvalidSocketHandle && !open())
		{
			return false;
		}

		sockaddr_in addr{};
		addr.sin_family = AF_INET;
		addr.sin_port = htons(port);

		if (host.empty() || host == "0.0.0.0")
		{
			addr.sin_addr.s_addr = INADDR_ANY;
		}
		else
		{
			::inet_pton(AF_INET, host.c_str(), &addr.sin_addr);
		}

		if (::bind(toNative(m_socket), reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0)
		{
			utils::Logger::getInstance().error("bind failed on port " + std::to_string(port));
			return false;
		}

		return true;
	}

	// Transmits a binary UDP datagram to the destination IPv4 host and port
	int UdpSocket::sendTo(const void* data, size_t size, const std::string& host, uint16_t port)
	{
		if (m_socket == kInvalidSocketHandle && !open())
		{
			return -1;
		}

		sockaddr_in dest{};
		dest.sin_family = AF_INET;
		dest.sin_port = htons(port);
		::inet_pton(AF_INET, host.c_str(), &dest.sin_addr);

		return static_cast<int>(::sendto(
			toNative(m_socket),
			reinterpret_cast<const char*>(data),
			static_cast<int>(size),
			0,
			reinterpret_cast<const sockaddr*>(&dest),
			sizeof(dest)
		));
	}

	// Reads an incoming UDP datagram into buffer (returns byte count, or -1 if queue is empty)
	int UdpSocket::recvFrom(void* buffer, size_t maxSize)
	{
		if (m_socket == kInvalidSocketHandle)
		{
			return -1;
		}

		sockaddr_in sender{};
		SockLen senderLen = sizeof(sender);

		return static_cast<int>(::recvfrom(
			toNative(m_socket),
			reinterpret_cast<char*>(buffer),
			static_cast<int>(maxSize),
			0,
			reinterpret_cast<sockaddr*>(&sender),
			&senderLen
		));
	}
}
