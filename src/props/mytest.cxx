#include <stdio.h>

#include "props/props.hxx"

int main() {
    // allocate the root property node to match extern/global in props.hxx
    props = new SGPropertyNode;

    // create new properties on the fly by specifying their paths
    SGPropertyNode *imu0 = fgGetNode("/sensors/imu", 0, true);
    SGPropertyNode *imu1 = fgGetNode("/sensors/imu", 1, true);
    SGPropertyNode *p0 = fgGetNode("/sensors/imu/p", true);
    SGPropertyNode *lat = fgGetNode("/sensors/gps/latitude-deg", true);

    // create new children of a parent
    SGPropertyNode *q0 = imu0->getChild("q", 0, true);

    // test if a child exists
    SGPropertyNode *q1 = imu1->getChild("q");
    if ( q1 == NULL ) {
	printf("q doesn't exist in imu1\n");
    }

    // test if a path exists
    SGPropertyNode *differential_press
	= fgGetNode("/sensors/airdata/teensy/i2c_3/adc[14]");
    if ( differential_press == NULL ){
	printf("adc[14] doesn't exist\n");
    }

    // set a value
    q0->setDoubleValue(5.4321);

    // read and print the value
    printf("imu[0]/q = %.5f\n", q0->getDoubleValue());

    // read the value as an int
    printf("imu[0]/q (int) = %d\n", q0->getIntValue());

    // read the value as a C string
    printf("imu[0]/q (string) = %s\n", q0->getStringValue());	  
}