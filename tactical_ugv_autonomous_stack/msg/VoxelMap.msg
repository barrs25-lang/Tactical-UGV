# Mirrors the raw binary voxel-grid blob exchanged over the legacy comm_server "map" socket:
# a flat char[300000] buffer (100 x 30 x 100 voxels at 0.2 m/voxel resolution in the legacy
# code), sent/received byte-for-byte with no framing. size_x/size_y/size_z/resolution are
# provided for documentation/consumers; the bridge itself treats `data` as an opaque
# fixed-length blob and does not reinterpret indexing -- whatever produces this message must
# match the legacy i/j/k nesting expected by the planner binaries.
std_msgs/Header header

uint32 size_x
uint32 size_y
uint32 size_z
float64 resolution

int8[300000] data
