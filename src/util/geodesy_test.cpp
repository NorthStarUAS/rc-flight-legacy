#include <math.h>
#include <stdio.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

#include "math/SGMath.h"
#include "math/SGGeodesy.h"

#include "geodesy.h"

int main() {
    double lat = 44.23;
    double lon = -93.77;
    double alt = 280.0;
    SGGeod orig_lla;
    orig_lla = SGGeod::fromDegM(lon, lat, alt);
    printf("orig lla = %.8f %.8f %.2f\n",
           orig_lla.getLatitudeDeg(),
           orig_lla.getLongitudeDeg(),
           orig_lla.getElevationM());

    SGVec3d orig_cart;
    SGGeodesy::SGGeodToCart(orig_lla, orig_cart);
    printf("origin cart = %.4f %.4f %.4f\n", orig_cart(0), orig_cart(1), orig_cart(2));
    
    //SGVec3d ecef( -303.549, -4570.142, 4424.246 );
    SGGeod wgs84;
    SGGeodesy::SGCartToGeod( orig_cart, wgs84 );
    printf("recip wgs84 = %.8f %.8f %.2f\n",
           wgs84.getLatitudeDeg(),
           wgs84.getLongitudeDeg(),
           wgs84.getElevationM());
    
    SGQuatd ecef2ned = SGQuatd::fromLonLat(wgs84);
    printf("orig ecef2ned = %.4f %.4f %.4f %.4f\n",
           ecef2ned.x(), ecef2ned.y(), ecef2ned.z(), ecef2ned.w());
    
    SGVec3d vel_ecef( 3, -45, 40 );
    // SGVec3d vel_ecef = ecef2ned.backTransform(vel_ned);
    SGVec3d vel_ned = ecef2ned.transform( vel_ecef );
    printf("orig vel ned = %.2f %.2f %.2f\n", vel_ned.x(), vel_ned.y(), vel_ned.z());

    Vector3d new_cart = Vector3d( orig_cart.x(), orig_cart.y(), orig_cart.z() );
    Vector3d new_wgs84 = ecef2lla( new_cart );
    printf("new wgs84 = %.8f %.8f %.2f\n",
           new_wgs84[0] * 180.0 / M_PI,
           new_wgs84[1] * 180.0 / M_PI,
           new_wgs84[2]);

    Vector3d new_vel( vel_ecef(0), vel_ecef(1), vel_ecef(2) );
    Quaterniond q = fromLonLatRad(new_wgs84[1], new_wgs84[0]);
    printf("new ecef2ned = %.4f %.4f %.4f %.4f\n",
           q.x(), q.y(), q.z(), q.w());
    Vector3d new_vel_ned = quat_backtransform(q, new_vel);
    printf("new vel ned = %.2f %.2f %.2f\n", new_vel_ned.x(), new_vel_ned.y(), new_vel_ned.z());
}
