#ifndef TACTICAL_UGV_AUTONOMOUS_STACK_LATEST_MESSAGE_BOX_HPP
#define TACTICAL_UGV_AUTONOMOUS_STACK_LATEST_MESSAGE_BOX_HPP

#include <atomic>
#include <condition_variable>
#include <mutex>

namespace tactical_ugv_autonomous_stack
{

// Thread-safe single-slot mailbox: a ROS2 subscription callback (on the executor thread)
// calls set(); a socket-sender thread blocks in wait_for_update() until a new value arrives,
// then reads it out via get(). Used to hand the latest message of each "*_from_server" stream
// from its ROS2 subscription to the thread that forwards it down the matching TCP socket.
template<typename T>
class LatestMessageBox
{
public:
	void set(const T & value)
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			value_ = value;
			has_value_ = true;
			version_++;
		}
		cv_.notify_all();
	}

	// Blocks until a value newer than `last_seen_version` is available, or `stop` becomes
	// true. Returns false if woken up by `stop`.
	bool wait_for_update(uint64_t & last_seen_version, T & out, const std::atomic<bool> & stop)
	{
		std::unique_lock<std::mutex> lock(mutex_);
		cv_.wait(lock, [&] {return (has_value_ && version_ != last_seen_version) || stop.load();});
		if (stop.load()) {
			return false;
		}
		out = value_;
		last_seen_version = version_;
		return true;
	}

	// Wakes any thread blocked in wait_for_update() so it can re-check an externally-set stop
	// flag (used on shutdown).
	void notify_stop()
	{
		cv_.notify_all();
	}

private:
	std::mutex mutex_;
	std::condition_variable cv_;
	T value_{};
	bool has_value_ = false;
	uint64_t version_ = 0;
};

}  // namespace tactical_ugv_autonomous_stack

#endif  // TACTICAL_UGV_AUTONOMOUS_STACK_LATEST_MESSAGE_BOX_HPP
