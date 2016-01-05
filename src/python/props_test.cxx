#include "pyprops.hxx"

int main(int argc, char **argv) {
    pyPropsInit(argc, argv);

    pyPropertyNode az_node = pyGetNode("/sensors/imu[2]/accel/az");
    printf("az = %.2f\n", az_node.getDoubleValue());
    printf("az(int) = %ld\n", az_node.getLongValue());
    
    pyPropertyNode name_node = pyGetNode("/sensors/gps[5]/name");
    printf("gps name = %s\n", name_node.getStringValue());

    pyPropertyNode test1_node = pyGetNode("/sensors/gps[5]/test1");
    printf("gps test = %f\n", test1_node.getDoubleValue());
    printf("gps test = %ld\n", test1_node.getLongValue());
    printf("gps test = %s\n", test1_node.getStringValue());
   
    pyGetNode("/sensors/imu[2]/accel");
    pyGetNode("/sensors/imu[2]");
    if ( pyGetNode("/sensors/imu").isBranch() ) {
	printf("/sensors/imu is a branch\n");
    }
    if ( pyGetNode("/sensors/imu[1]").isBranch() ) {
	printf("/sensors/imu[1] is a branch\n");
    }
    
    pyPropsClose();
}
