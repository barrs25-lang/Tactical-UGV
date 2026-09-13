// Translates octomap_server's octomap_binary output into the fixed dense VoxelMap layout the
// legacy planner binaries hardcode (see tactical_ugv_autonomous_stack/msg/VoxelMap.msg and the
// receive loops in goal_generation.cpp / LPAstar.cpp / f_mpc_communication.cpp / constraint.cpp,
// which all agree on: 100x30x100 voxels @ 0.2m, byte index = k*(grid_y*grid_x) + j*grid_x + i).
//
// Anchoring: this node does NOT track vehicle pose itself. octomap_server is expected to be
// configured with frame_id set to the camera's start-anchored world frame (ZED's VIO/odometry
// frame is identity at power-on), so octree coordinate (0,0,0) already IS the vehicle's start
// location by the time a cloud reaches the octree. All three axes are corner-anchored at that
// origin, 0 to +20m horizontally and 0 to +6m vertically (NOT centered on start) -- this must
// match the legacy planner binaries' own world<->voxel convention: goal_generation's octree.h
// and path_planner's LPAstar.cpp both convert with a bare `position / voxel_resolution`, no
// offset, so voxel index 0 is world 0 for them too. A previous version of this node centered
// the horizontal axes on start (+-10m, voxel 50 = world 0), which put the legacy binaries'
// real-world start position at voxel ~(0,0) -- wedged against the forced-occupied boundary ring
// below -- and left goal_generation reading "explored" cells 10m away from where they actually
// are. If you re-derive this mapping, re-derive it from the legacy code's own convention, not
// from what seems geometrically natural for the new ROS2 side.
//
// Cell coding: 1 = free, 3 = occupied (the only two codes any legacy binary's live code path
// checks), 0 = unknown/unobserved. Unknown is a real third state, not a placeholder -- without
// it, goal_generation's frontier-exploration term (mu_prox/mu_max blend, see octree.h) has
// nothing to seek, since every cell would read as already-explored. The outer i/k boundary ring
// is force-written to 3 on every publish regardless of sensor data: goal_generation rejects the
// entire map outright if any boundary voxel isn't 2 or 3 (goal_generation.cpp).

#include <array>
#include <cstdint>
#include <memory>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "octomap/octomap.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "tactical_ugv_autonomous_stack/msg/voxel_map.hpp"

namespace
{
constexpr int kGridX = 100;   // i: horizontal, start-anchored, 0 to +20m
constexpr int kGridY = 30;    // j: vertical, start height is the floor, 0 to +6m
constexpr int kGridZ = 100;   // k: horizontal, start-anchored, 0 to +20m
constexpr double kResolution = 0.2;
constexpr std::size_t kBufferSize = static_cast<std::size_t>(kGridX) * kGridY * kGridZ;
static_assert(
	kBufferSize == std::tuple_size<decltype(tactical_ugv_autonomous_stack::msg::VoxelMap::data)>::value,
	"kBufferSize must match VoxelMap.msg's fixed-size data array");

constexpr int8_t kUnknown = 0;
constexpr int8_t kFree = 1;
constexpr int8_t kOccupied = 3;

inline std::size_t index_of(int i, int j, int k)
{
	return static_cast<std::size_t>(k) * (kGridY * kGridX) + static_cast<std::size_t>(j) * kGridX + i;
}
}  // namespace

class OctomapToVoxelMap : public rclcpp::Node
{
public:
	OctomapToVoxelMap()
	: Node("octomap_to_voxelmap")
	{
		this->declare_parameter<std::string>("octomap_topic", "octomap_binary");
		this->declare_parameter<std::string>("map_topic", "/ugv/map");

		const std::string octomap_topic = this->get_parameter("octomap_topic").as_string();
		const std::string map_topic = this->get_parameter("map_topic").as_string();

		map_pub_ = this->create_publisher<tactical_ugv_autonomous_stack::msg::VoxelMap>(
			map_topic, rclcpp::QoS(1));

		octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
			octomap_topic, rclcpp::QoS(1),
			[this](const octomap_msgs::msg::Octomap::SharedPtr msg) {on_octomap(msg);});

		RCLCPP_INFO(get_logger(), "Subscribed to [%s], publishing VoxelMap on [%s]",
			octomap_topic.c_str(), map_topic.c_str());
	}

private:
	void on_octomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
	{
		std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::binaryMsgToMap(*msg));
		if (!abstract_tree) {
			RCLCPP_WARN(get_logger(), "Failed to deserialize octomap_binary message, skipping cycle");
			return;
		}
		auto * tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
		if (!tree) {
			RCLCPP_WARN(get_logger(), "Deserialized octree is not an OcTree, skipping cycle");
			return;
		}

		tactical_ugv_autonomous_stack::msg::VoxelMap map_msg;
		map_msg.header.stamp = this->now();
		map_msg.header.frame_id = "odom";
		map_msg.size_x = kGridX;
		map_msg.size_y = kGridY;
		map_msg.size_z = kGridZ;
		map_msg.resolution = kResolution;

		for (int k = 0; k < kGridZ; ++k) {
			const double y_world = (k + 0.5) * kResolution;
			for (int j = 0; j < kGridY; ++j) {
				const double z_world = (j + 0.5) * kResolution;
				for (int i = 0; i < kGridX; ++i) {
					const double x_world = (i + 0.5) * kResolution;

					int8_t code = kUnknown;
					octomap::OcTreeNode * node = tree->search(x_world, y_world, z_world);
					if (node != nullptr) {
						code = tree->isNodeOccupied(node) ? kOccupied : kFree;
					}
					map_msg.data[index_of(i, j, k)] = code;
				}
			}
		}

		// Force the outer i/k boundary ring to occupied regardless of sensor data -- required by
		// goal_generation's map validity check, and functions as the maintained arena's perimeter.
		for (int k = 0; k < kGridZ; ++k) {
			for (int j = 0; j < kGridY; ++j) {
				map_msg.data[index_of(0, j, k)] = kOccupied;
				map_msg.data[index_of(kGridX - 1, j, k)] = kOccupied;
			}
		}
		for (int i = 0; i < kGridX; ++i) {
			for (int j = 0; j < kGridY; ++j) {
				map_msg.data[index_of(i, j, 0)] = kOccupied;
				map_msg.data[index_of(i, j, kGridZ - 1)] = kOccupied;
			}
		}

		map_pub_->publish(map_msg);
	}

	rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
	rclcpp::Publisher<tactical_ugv_autonomous_stack::msg::VoxelMap>::SharedPtr map_pub_;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OctomapToVoxelMap>());
	rclcpp::shutdown();
	return 0;
}
