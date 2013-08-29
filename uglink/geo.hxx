#ifndef _UGLINK_GEO_HXX
#define _UGLINK_GEO_HXX


#include "math/SGMath.hxx"


// cam_pos: SGGeod representing the lon, lat, alt of camera
// roll/pitch/yaw: aircraft orientation in euler angles (degrees)
// h/v: horizontal/vertical size of camera sensor in mm
// focal_len: focal length of sensor in mm
// lookat/ll/lr/ul/ur: SGGeod represention wgs84 location of center of view and corners of the image
void geolocate_image( SGGeod cam_pos, double ground_m,
		      double roll_deg, double pitch_deg, double yaw_deg,
		      double h_mm, double v_mm, double focal_len_mm,
		      SGGeod *lookat_wgs84, 
		      SGGeod *ll_wgs84, 
		      SGGeod *lr_wgs84, 
		      SGGeod *ul_wgs84, 
		      SGGeod *ur_wgs84 );


#endif // _UGLINK_GEO_HXX
