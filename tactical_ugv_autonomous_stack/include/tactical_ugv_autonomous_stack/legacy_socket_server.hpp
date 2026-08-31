#ifndef TACTICAL_UGV_AUTONOMOUS_STACK_LEGACY_SOCKET_SERVER_HPP
#define TACTICAL_UGV_AUTONOMOUS_STACK_LEGACY_SOCKET_SERVER_HPP

// Minimal blocking TCP "server" socket that is wire-compatible with the SOCKET::init()
// client-connect logic in trajectory_planner/comm_server/src/client/include/my_socket.h:
// that class always connects out to 127.0.0.1:<port> and, immediately after connecting,
// sends a fixed 30-byte handshake ("Hi this is client 1." padded to 30 bytes). This class
// plays the comm_server role for exactly one port: bind + listen + accept, consume that
// handshake, then exchange fixed-size buffers exactly like comm_server's own SOCKET class
// (trajectory_planner/comm_server/include/my_socket.h) does.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace tactical_ugv_autonomous_stack
{

class LegacySocketServer
{
public:
	LegacySocketServer() = default;

	~LegacySocketServer()
	{
		close_all();
	}

	// Binds and listens on the given port. Does not block.
	void bind_and_listen(uint16_t port, const std::string & name)
	{
		name_ = name;
		port_ = port;

		listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
		if (listen_fd_ < 0) {
			throw std::runtime_error("tactical_ugv_autonomous_stack: failed to create socket for " + name_);
		}

		int reuse = 1;
		setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

		sockaddr_in addr{};
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = INADDR_ANY;
		addr.sin_port = htons(port_);

		if (bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
			throw std::runtime_error("tactical_ugv_autonomous_stack: failed to bind " + name_ + " on port " + std::to_string(port_));
		}

		if (listen(listen_fd_, 1) < 0) {
			throw std::runtime_error("tactical_ugv_autonomous_stack: failed to listen on " + name_);
		}
	}

	// Blocks until the legacy CLIENT connects, then consumes its fixed 30-byte handshake.
	// Safe to call again after a disconnect to accept a fresh connection.
	void accept_and_handshake()
	{
		sockaddr_in client_addr{};
		socklen_t addrlen = sizeof(client_addr);

		client_fd_ = accept(listen_fd_, reinterpret_cast<sockaddr *>(&client_addr), &addrlen);
		if (client_fd_ < 0) {
			throw std::runtime_error("tactical_ugv_autonomous_stack: accept() failed for " + name_);
		}

		char handshake[30];
		recv_exact(handshake, sizeof(handshake));
	}

	// Blocking receive of exactly `size` bytes into `buffer`. Returns false if the peer
	// closed the connection (recv returned <= 0) partway through.
	bool recv_exact(void * buffer, size_t size)
	{
		size_t received = 0;
		auto * p = reinterpret_cast<char *>(buffer);
		while (received < size) {
			ssize_t n = recv(client_fd_, p + received, size - received, MSG_WAITALL);
			if (n <= 0) {
				return false;
			}
			received += static_cast<size_t>(n);
		}
		return true;
	}

	// Blocking send of exactly `size` bytes from `buffer`.
	bool send_exact(const void * buffer, size_t size)
	{
		size_t sent = 0;
		const auto * p = reinterpret_cast<const char *>(buffer);
		while (sent < size) {
			ssize_t n = send(client_fd_, p + sent, size - sent, 0);
			if (n <= 0) {
				return false;
			}
			sent += static_cast<size_t>(n);
		}
		return true;
	}

	const std::string & name() const {return name_;}
	uint16_t port() const {return port_;}

	void close_client()
	{
		if (client_fd_ >= 0) {
			shutdown(client_fd_, SHUT_RDWR);
			close(client_fd_);
			client_fd_ = -1;
		}
	}

	void close_all()
	{
		close_client();
		if (listen_fd_ >= 0) {
			close(listen_fd_);
			listen_fd_ = -1;
		}
	}

private:
	int listen_fd_ = -1;
	int client_fd_ = -1;
	std::string name_;
	uint16_t port_ = 0;
};

}  // namespace tactical_ugv_autonomous_stack

#endif  // TACTICAL_UGV_AUTONOMOUS_STACK_LEGACY_SOCKET_SERVER_HPP
