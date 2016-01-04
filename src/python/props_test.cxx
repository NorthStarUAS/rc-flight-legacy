#include "pyprops.hxx"

int main(int argc, char **argv) {
    pyPropsInit(argc, argv);
    pyPropertyNode *abc;
    pyGetNode("/sensors/imu[2]/accel/az");
    pyPropsClose();
}
