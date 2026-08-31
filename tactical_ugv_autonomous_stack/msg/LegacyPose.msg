# Mirrors the 18-float pose array exchanged over the legacy comm_server socket protocol
# ('Q' tag + 18 CSV floats): position/velocity/acceleration/jerk (x,y,z triplets) plus
# Euler angles and body-frame angular rates.
std_msgs/Header header

float64 x
float64 y
float64 z
float64 dx
float64 dy
float64 dz
float64 ddx
float64 ddy
float64 ddz
float64 dddx
float64 dddy
float64 dddz
float64 phi
float64 theta
float64 psi
float64 omega1
float64 omega2
float64 omega3
