#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

Vector3d ecef2lla(const Vector3d cart);
Quaterniond fromLonLatRad(double lon_rad, double lat_rad);
Vector3d quat_backtransform( Quaterniond q, Vector3d v );
