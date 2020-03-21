#pragma once

#include <string>
using std::string;

// return the next available path
string get_next_path( const char *path, const char *base, bool primary=false );
