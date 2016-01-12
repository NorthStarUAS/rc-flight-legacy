#include "pyprops.hxx"

#include <string>
using std::string;

int main(int argc, char **argv) {
    // cleanup the python interpreter after all the main() and global
    // destructors are called
    atexit(pyPropsCleanup);
    
    pyPropsInit(argc, argv);

    pyPropertyNode imu_node = pyGetNode("/sensors/imu[2]");
    printf("before calling getDouble()\n");
    printf("az = %.2f\n", imu_node.getDouble("az"));
    printf("az(int) = %ld\n", imu_node.getLong("az"));
    imu_node.setDouble("az", -9.7);
    printf("az = %.2f\n", imu_node.getDouble("az"));
    imu_node.setLong("az", -10);
    printf("az = %.2f\n", imu_node.getDouble("az"));
    imu_node.setString("az", "-9.8092322");
    printf("az = %.8f\n", imu_node.getDouble("az"));
   
    pyPropertyNode gps_node = pyGetNode("/sensors/gps[5]", true);
    printf("gps name = %s\n", gps_node.getString("name").c_str());
    printf("gps test = %f\n", gps_node.getDouble("test1"));
    printf("gps test = %ld\n", gps_node.getLong("test1"));
    printf("gps test = %s\n", gps_node.getString("test1").c_str());

    pyPropertyNode sensors = pyGetNode("/sensors");
    printf("gps size = %d\n", sensors.getLen("gps"));

    pyGetNode("/sensors/imu[2]");
    
    printf("/sensors/imu = %s\n", pyGetNode("/sensors").getString("imu").c_str());
    // this is nonsensical usage, but also causes a segfault which we
    // really need to catch
    pyPropertyNode t1 = pyGetNode("/sensors/imu[1]");
    string t2 = t1.getString("az");
    printf("t1.t2=%s\n", t2.c_str());
    printf("/sensors/imu[1] = %s\n", pyGetNode("/sensors/imu[1]").getString("az").c_str());

    string fullpath = "/sensors/imu[1]/az";
    int pos = fullpath.rfind("/");
    printf("%d %s / %s\n", pos, fullpath.substr(0, pos).c_str(),
	   fullpath.substr(pos+1).c_str() );
}
