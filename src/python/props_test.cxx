#include "pyprops.hxx"

int main(int argc, char **argv) {
    // cleanup the python interpreter after all the destructors are called
    atexit(pyPropsCleanup);
    
    pyPropsInit(argc, argv);

    pyPropertyNode imu_node = pyGetNode("/sensors/imu[2]");
    printf("before calling getDoubleValue()\n");
    printf("az = %.2f\n", imu_node.getDoubleValue("az"));
    printf("az(int) = %ld\n", imu_node.getLongValue("az"));
    
    pyPropertyNode gps_node = pyGetNode("/sensors/gps[5]");
    printf("gps name = %s\n", gps_node.getStringValue("name").c_str());
    printf("gps test = %f\n", gps_node.getDoubleValue("test1"));
    printf("gps test = %ld\n", gps_node.getLongValue("test1"));
    printf("gps test = %s\n", gps_node.getStringValue("test1").c_str());
   
    pyGetNode("/sensors/imu[2]");
    
    printf("/sensors/imu = %s\n", pyGetNode("/sensors").getStringValue("imu").c_str());
    // this is nonsensical usage, but also causes a segfault which we
    // really need to catch
    pyPropertyNode t1 = pyGetNode("/sensors/imu[1]");
    string t2 = t1.getStringValue("az");
    printf("t1.t2=%s\n", t2.c_str());
    printf("/sensors/imu[1] = %s\n", pyGetNode("/sensors/imu[1]").getStringValue("az").c_str());
}
