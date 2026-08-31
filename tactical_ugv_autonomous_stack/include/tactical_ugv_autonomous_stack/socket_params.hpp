#ifndef TACTICAL_UGV_AUTONOMOUS_STACK_SOCKET_PARAMS_HPP
#define TACTICAL_UGV_AUTONOMOUS_STACK_SOCKET_PARAMS_HPP

// Parses the same socket_parameters_<package>.txt file format the legacy CLIENT/SERVER
// classes read (trajectory_planner/comm_server/src/client/include/my_client.h and
// trajectory_planner/comm_server/include/my_server.h): fixed-order blocks of
// (port number, name, active flag), comment lines beginning with "//".
//
// Socket index order (matches the legacy convention exactly):
//   0 map_from_server        1 map_to_server
//   2 pose_from_server       3 pose_to_server
//   4 goal_from_server       5 goal_to_server
//   6 path_from_server       7 path_to_server
//   8 constraint_from_server 9 constraint_to_server
//  10 trajectory_from_server 11 trajectory_to_server
//
// Sockets beyond the 12th are package-specific additions -- e.g. index 12,
// "control_to_server", exists only in trajectory_planner's own socket_parameters file
// (it sends the planned control sequence F_x/delta_f, which no other package produces).
// `num_sockets` therefore isn't a fixed constant: pass 12 for goal_generation/path_planner
// and 13 for trajectory_planner.

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace tactical_ugv_autonomous_stack
{

struct SocketDesc
{
	int port = 0;
	std::string name;
	bool active = false;
};

// Default socket count for packages that only use the original 12 legacy sockets.
constexpr int kNumSockets = 12;

inline std::vector<SocketDesc> load_socket_params(const std::string & path, int num_sockets = kNumSockets)
{
	std::ifstream file(path);
	if (!file.is_open()) {
		throw std::runtime_error("tactical_ugv_autonomous_stack: could not open socket parameters file: " + path);
	}

	std::vector<SocketDesc> sockets(num_sockets);
	std::string line;
	std::stringstream ss;

	auto next_data_line = [&]() {
			do {
				ss.clear();
				if (!std::getline(file, line)) {
					throw std::runtime_error("tactical_ugv_autonomous_stack: unexpected end of file in " + path);
				}
				ss.str(line);
			} while (line.size() >= 2 && line[0] == '/' && line[1] == '/');
		};

	for (int i = 0; i < num_sockets; ++i) {
		next_data_line();
		ss >> sockets[i].port;

		next_data_line();
		ss >> sockets[i].name;

		next_data_line();
		ss >> sockets[i].active;
	}

	return sockets;
}

}  // namespace tactical_ugv_autonomous_stack

#endif  // TACTICAL_UGV_AUTONOMOUS_STACK_SOCKET_PARAMS_HPP
