#include <string>
#include <sstream>
using std::string;
using std::ostringstream;

#include <props2.h>

#include "props_helper.h"

string get_next_path( const char *path, const char *base, bool primary ) {
    PropertyNode node = PropertyNode(path, true);
    int len = node.getLen(base);
    string output_path = (string)path + "/" + base + "/";
    if ( primary ) {
        output_path += "0";
    } else {
        output_path += std::to_string(len);
    }
    printf("  new path: %s\n", output_path.c_str());
    return output_path;
}
