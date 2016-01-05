#include "pyprops.hxx"

int main(int argc, char **argv) {
    pyPropsInit(argc, argv);

    pyPropertyNode az_node = pyGetNode("/sensors/imu[2]/accel/az");
    double az = az_node.getDoubleValue();
    printf("az = %.2f\n", az);
    
    pyGetNode("/sensors/imu[2]/accel");
    pyGetNode("/sensors/imu[2]");
    pyGetNode("/sensors/imu");
    pyPropsClose();
}
