#include <string>
#include <sstream>
using std::string;
using std::ostringstream;

#include <pyprops.h>

#include "props_helper.h"

string get_next_path( const char *path, const char *base,
                             bool primary )
{
    pyPropertyNode node = pyGetNode(path, true);
    int len = node.getLen(base);
    if ( primary ) {
        if ( len > 1 ) {
            // nop
        } else {
            node.setLen(base, 1);
        }
    } else {
        node.setLen(base, len + 1);
    }
    ostringstream output_path;
    if ( primary ) {
        output_path << path << "/" << base << "[0]";
    } else {
        output_path << path << "/" << base << '[' << len << ']';
    }
    printf("  new path: %s\n", output_path.str().c_str());
    return output_path.str();
}
