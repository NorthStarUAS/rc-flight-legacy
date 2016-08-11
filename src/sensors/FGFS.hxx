//
// FILE: FGFS.hxx
// DESCRIPTION: aquire live sensor data from an running copy of Flightgear
//

#ifndef _AURA_FGFS_HXX
#define _AURA_FGFS_HXX


// function prototypes
bool FGFS_update();

bool fgfs_imu_init( string output_path, pyPropertyNode *config );
bool fgfs_imu_update();
void fgfs_imu_close();

bool fgfs_airdata_init( string output_path );
bool fgfs_airdata_update();
void fgfs_airdata_close();

bool fgfs_gps_init( string output_path, pyPropertyNode *config );
bool fgfs_gps_update();
void fgfs_gps_close();

bool fgfs_pilot_init( string output_path, pyPropertyNode *config );
bool fgfs_pilot_update();
void fgfs_pilot_close();

#endif // _AURA_FGFS_HXX
