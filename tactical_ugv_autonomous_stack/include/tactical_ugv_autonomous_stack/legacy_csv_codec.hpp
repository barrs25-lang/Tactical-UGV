#ifndef TACTICAL_UGV_AUTONOMOUS_STACK_LEGACY_CSV_CODEC_HPP
#define TACTICAL_UGV_AUTONOMOUS_STACK_LEGACY_CSV_CODEC_HPP

// Encode/decode helpers for the legacy comm_server "TAG,v0,v1,...,vN!" wire framing used by
// the pose/goal/path/constraint/trajectory sockets (see trajectory_planner/comm_server/src/
// my_server.cpp and trajectory_planner/trajectory_planner/src/f_mpc_uncut/
// f_mpc_communication.cpp). Every sender writes the tag character, then a comma, then each
// value separated by commas, then '!' immediately after the last value (no trailing comma).
// Every receiver checks buffer[0] against the tag, then walks from index 1 splitting on ','
// and stopping at '!' -- a comma must therefore follow the tag even though there is no value
// before it, or the receiver's first token index is invalid.

#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace tactical_ugv_autonomous_stack
{

// Builds "tag,v0,v1,...,vN!" into `buffer` (size `buffer_size`), zero-padding the remainder.
// Returns false if the encoded message would not fit.
inline bool encode_legacy_csv(
	char tag, const std::vector<double> & values, std::vector<char> & buffer)
{
	std::ostringstream oss;
	// The legacy pipeline only ever carries 32-bit floats end to end (boost::lexical_cast<float>
	// on receipt), so 9 significant digits is enough for an exact float round-trip.
	oss << std::setprecision(9) << tag;
	for (size_t i = 0; i < values.size(); ++i) {
		oss << ',' << values[i];
	}
	oss << '!';

	const std::string s = oss.str();
	if (s.size() > buffer.size()) {
		return false;
	}

	std::fill(buffer.begin(), buffer.end(), '\0');
	std::memcpy(buffer.data(), s.data(), s.size());
	return true;
}

// Parses "tag,v0,v1,...,vN!" out of `buffer`. Returns false (and leaves `values` unspecified)
// if the buffer's first byte does not match `expected_tag`.
inline bool decode_legacy_csv(
	const std::vector<char> & buffer, char expected_tag, std::vector<double> & values)
{
	if (buffer.empty() || buffer[0] != expected_tag) {
		return false;
	}

	values.clear();
	int counter = 0;
	std::string token;

	for (size_t i = 1; i < buffer.size(); ++i) {
		char c = buffer[i];
		if (c == '!') {
			break;
		} else if (c == ',') {
			counter++;
			values.resize(counter, 0.0);
			token.clear();
		} else if (c == '\0') {
			// Padding beyond a short message with no terminating '!' -- treat as end.
			break;
		} else {
			token += c;
			if (counter > 0) {
				values[counter - 1] = std::strtod(token.c_str(), nullptr);
			}
		}
	}

	return true;
}

}  // namespace tactical_ugv_autonomous_stack

#endif  // TACTICAL_UGV_AUTONOMOUS_STACK_LEGACY_CSV_CODEC_HPP
