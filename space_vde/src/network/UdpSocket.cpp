#include "space_docking/network/UdpSocket.hpp"

#include "space_docking/utils/Logger.hpp"

#include <cstring>
#include <utility>

#if !defined(_WIN32)
#include <fcntl.h>
#include <unistd.h>
#endif

namespace space_docking::network
{
	namespace
	{
		constexpr SocketFd kInvalidSocketFd =
#if defined(_WIN32)
			INVALID_SOCKET;
#else
			-1;
#endif
	}

	void NetSystem::initialize()
	{
#if defined(_WIN32)
		static bool initialized = false;
		if (!initialized)
		{
			WSADATA wsaData{};
			if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
			{
				utils::Logger::getInstance().error("WSAStartup failed in NetSystem::initialize");
			}
			initialized = true;
		}
#endif
	}

	UdpSocket::UdpSocket()
		: m_socket(kInvalidSocketFd)
	{
		NetSystem::initialize();
	}

	UdpSocket::~UdpSocket()
	{
		close();
	}

	UdpSocket::UdpSocket(UdpSocket&& other) noexcept
		: m_socket(other.m_socket)
	{
		other.m_socket = kInvalidSocketFd;
	}

	UdpSocket& UdpSocket::operator=(UdpSocket&& other) noexcept
	{
		if (this != &other)
		{
			close();
			m_socket = other.m_socket;
			other.m_socket = kInvalidSocketFd;
		}
		return *this;
	}

	bool UdpSocket::open()
	{
		close();
		m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (m_socket == kInvalidSocketFd)
		{
			utils::Logger::getInstance().error("Failed to create UDP socket");
			return false;
		}

		setReuseAddr(true);
		return true;
	}

	void UdpSocket::close()
	{
		if (m_socket != kInvalidSocketFd)
		{
#if defined(_WIN32)
			closesocket(m_socket);
#else
			::close(m_socket);
#endif
			m_socket = kInvalidSocketFd;
		}
	}

	bool UdpSocket::isValid() const noexcept
	{
		return m_socket != kInvalidSocketFd;
	}

	bool UdpSocket::setReuseAddr(bool enable)
	{
		if (m_socket == kInvalidSocketFd)
		{
			return false;
		}
		int opt = enable ? 1 : 0;
#if defined(_WIN32)
		return setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt)) == 0;
#else
		bool ok = (setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == 0);
#ifdef SO_REUSEPORT
		setsockopt(m_socket, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif
		return ok;
#endif
	}

	bool UdpSocket::setNonBlocking(bool nonBlocking)
	{
		if (m_socket == kInvalidSocketFd)
		{
			return false;
		}
#if defined(_WIN32)
		u_long mode = nonBlocking ? 1 : 0;
		return ioctlsocket(m_socket, FIONBIO, &mode) == 0;
#else
		int flags = fcntl(m_socket, F_GETFL, 0);
		if (flags < 0)
		{
			return false;
		}
		flags = nonBlocking ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
		return fcntl(m_socket, F_SETFL, flags) == 0;
#endif
	}

	bool UdpSocket::setMulticastLoop(bool enable)
	{
		if (m_socket == kInvalidSocketFd)
		{
			return false;
		}
#if defined(_WIN32)
		int opt = enable ? 1 : 0;
		return setsockopt(m_socket, IPPROTO_IP, IP_MULTICAST_LOOP, reinterpret_cast<const char*>(&opt), sizeof(opt)) == 0;
#else
		u_char loop = enable ? 1 : 0;
		return setsockopt(m_socket, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop)) == 0;
#endif
	}

	bool UdpSocket::joinMulticastGroup(const std::string& groupIp)
	{
		if (m_socket == kInvalidSocketFd)
		{
			return false;
		}
		ip_mreq mreq{};
		inet_pton(AF_INET, groupIp.c_str(), &mreq.imr_multiaddr.s_addr);
		mreq.imr_interface.s_addr = htonl(INADDR_ANY);
#if defined(_WIN32)
		return setsockopt(m_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, reinterpret_cast<const char*>(&mreq), sizeof(mreq)) == 0;
#else
		return setsockopt(m_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) == 0;
#endif
	}

	bool UdpSocket::bindSocket(uint16_t port, const std::string& host)
	{
		if (m_socket == kInvalidSocketFd && !open())
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
			inet_pton(AF_INET, host.c_str(), &addr.sin_addr);
		}

		if (bind(m_socket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
		{
			utils::Logger::getInstance().error("bind failed on port " + std::to_string(port));
			return false;
		}

		return true;
	}

	int UdpSocket::sendTo(const void* data, size_t size, const std::string& host, uint16_t port)
	{
		if (m_socket == kInvalidSocketFd && !open())
		{
			return -1;
		}

		sockaddr_in dest{};
		dest.sin_family = AF_INET;
		dest.sin_port = htons(port);
		inet_pton(AF_INET, host.c_str(), &dest.sin_addr);

		return static_cast<int>(sendto(
			m_socket,
			reinterpret_cast<const char*>(data),
			static_cast<int>(size),
			0,
			reinterpret_cast<sockaddr*>(&dest),
			sizeof(dest)
		));
	}

	int UdpSocket::recvFrom(void* buffer, size_t maxSize)
	{
		if (m_socket == kInvalidSocketFd)
		{
			return -1;
		}

		sockaddr_in sender{};
#if defined(_WIN32)
		int senderLen = sizeof(sender);
#else
		socklen_t senderLen = sizeof(sender);
#endif

		return recvfrom(
			m_socket,
			reinterpret_cast<char*>(buffer),
			static_cast<int>(maxSize),
			0,
			reinterpret_cast<sockaddr*>(&sender),
			&senderLen
		);
	}
}
