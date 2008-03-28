#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>

#include <control/route.hxx>
#include <util/point3d.hxx>
#include <util/polar3d.hxx>

#include <include/globaldefs.h>

using std::string;

double wind_dir = 0.0;
double wind_speed = 0.0;


void usage( char *prog ) {
    printf("Usage: %s [ --options ]\n", prog);
    printf("\n");
    printf("Note: coordinates can be specified in the following formats:\n");
    printf("  Decimal degrees: -158.25929 or decimal minutes: W158-15.5574\n");
    printf("\n");
    printf("  --start-lon <lon_coord>\n");
    printf("  --start-lat <lat_coord>\n");
    printf("  --home-lon <lon_coord>\n");
    printf("  --home-lat <lat_coord>\n");
    printf("  --swath-heading <initial swath heading deg true>\n");
    printf("  --swath-length <swath length meters>\n");
    printf("  --swath-width <width between swaths in meters>\n");
    printf("  --cross-heading <cross angle>\n");
    printf("         ideally this is into the wind and 90 degrees from the swath heading>\n");
    printf("  --wind-from-deg <wind 'from' direction>\n");
    printf("  --wind-speed-kts <wind speed kts>\n");
    printf("  --flight-speed-kts <estimated air speed kts>\n");
    printf("  --flight-time-min <estimated safe flight time in minutes>\n");

    exit(0);
}


double get_dist( const Point3D& start, const Point3D& dest ) {
    double course;
    double dist;
    calc_gc_course_dist( start, dest, &course, &dist );
    return dist;
}


double get_course( const Point3D& start, const Point3D& dest ) {
    double course;
    double dist;
    calc_gc_course_dist( start, dest, &course, &dist );
    return course;
}


double parse_coord( const string coord ) {
    double result = 0.0;

    printf("parse_coord: %s\n", coord.c_str());

    if ( !coord.length() ) {
        printf("Parse error: zero length coord\n");
        return result;
    }

    char c = coord.at(0);
    if ( (c >= '0' && c <= '9') || c == '-' ) {
        // decimal degree
        result = atof( coord.c_str() );
    } else {
        int sign = 1;
        if ( c == 's' || c == 'S' || c == 'w' || c == 'W' ) {
            sign = -1;
        }
        string mindeg = coord.substr(1);
        string::size_type pos = mindeg.find_first_of("-");
        if ( pos == string::npos ) {
            printf("whole degrees\n");
            result = atof( mindeg.c_str() );
        } else {
            printf("degree-minute.decimal\n");
            string deg = mindeg.substr(0, pos);
            string min = mindeg.substr(pos + 1);
            printf("deg = %s  min = %s\n", deg.c_str(), min.c_str());
            result = atof(deg.c_str()) + atof(min.c_str())/60.0;
        }
        result *= sign;
    }

    printf("result = %f\n", result);
    return result;
}


double calc_ete_min(double course, double dist_m, double flight_speed,
                    double wind_speed, double wind_from_deg) 
{
    if ( dist_m < 0.001 ) {
        return 0.0;
    }

    double wind_dir = wind_from_deg * SGD_DEGREES_TO_RADIANS;
    double swc = (wind_speed/flight_speed) * sin(wind_dir-course);
    double gs = 0.0;
    if ( fabs(swc) >= 1.0 ) {
        printf("course cannot be flown-- wind too strong\n");
        exit(-1);
    } else {
        double heading = course + asin(swc);
        if ( heading < 0.0 ) { heading = heading + SGD_2PI; }
        if ( heading > SGD_2PI ) { heading = heading - SGD_2PI; }
        gs = flight_speed * sqrt(1-swc*swc) - wind_speed * cos(wind_dir-course);
        if ( gs <= 0.0 ) {
            printf("course cannot be flown-- wind too strong\n");
            exit(-1);
        }
    }

    double dist_nm = dist_m * SG_METER_TO_NM;
    double ete_hour = dist_nm / gs;
    double ete_min = ete_hour * 60;

    printf("course = %.0f dist = %.0f tas = %.0f ws = %.0f winddir = %.0f\n",
           course * SGD_RADIANS_TO_DEGREES, dist_m, flight_speed,
           wind_speed, wind_from_deg);
    printf("  ete_min = %.2f\n", ete_min);

    return ete_min;
}

double route_length( const SGRoute route ) {
    // assumptions: flight is launched from "home" coordinates, UAS flies the
    // route.  The last waypoint in the route is the "home" location.

    double result = 0.0;
    SGWayPoint wpt;
    Point3D current, next;

    // launch pt (aka home) location is the last waypoint
    int size = route.size();
    wpt = route.get_waypoint( size - 1 );
    current = Point3D( wpt.get_target_lon(), wpt.get_target_lat(), 0 );

    for ( int i = 0; i < size; ++i ) {
        double course, dist;
        wpt = route.get_waypoint( i );
        next = Point3D( wpt.get_target_lon(), wpt.get_target_lat(), 0 );
        // printf("%.5f %.5f  %.5f %.5f\n", current.lon(), current.lat(),
        //        next.lon(), next.lat());
        calc_gc_course_dist( current, next, &course, &dist );
        // printf("  course = %.2f  dist = %.2f\n", course, dist);

        result += dist;
        current = next;
    }

    return result;
}


double route_max_dist( const SGRoute route ) {
    // Assumption: the last waypoint in the route is the home/launch location.

    double maxdist = 0.0;
    SGWayPoint wpt;
    Point3D home, current;

    // launch pt (aka home) location is the last waypoint
    int size = route.size();
    wpt = route.get_waypoint( size - 1 );
    home = Point3D( wpt.get_target_lon(), wpt.get_target_lat(), 0 );

    for ( int i = 0; i < size; ++i ) {
        double course, dist;
        wpt = route.get_waypoint( i );
        current = Point3D( wpt.get_target_lon(), wpt.get_target_lat(), 0 );
        // printf("%.5f %.5f  %.5f %.5f\n", current.lon(), current.lat(),
        //        next.lon(), next.lat());
        calc_gc_course_dist( home, current, &course, &dist );
        // printf("  course = %.2f  dist = %.2f\n", course, dist);

        if ( dist > maxdist ) { maxdist = dist; }
    }

    return maxdist;
}


int main( int argc, char **argv )
{
    double start_lon = 0.0;
    double start_lat = 0.0;
    double home_lon = 0.0;
    double home_lat = 0.0;
    double swath_hdg = 0.0;
    double swath_len = 0.0;
    double swath_width = 0.0;
    double cross_hdg = 0.0;
    double flight_speed = 0.0;
    double flight_time = 0.0;
    
    // Parse the command line
    for ( int iarg = 1; iarg < argc; iarg++ ) {
        if ( !strcmp(argv[iarg], "--start-lon" )  ) {
            ++iarg;
            // start_lon = atof( argv[iarg] ) * SGD_DEGREES_TO_RADIANS;
            start_lon = parse_coord( argv[iarg] ) * SGD_DEGREES_TO_RADIANS;
        } else if ( !strcmp(argv[iarg],"--start-lat") ) {
            ++iarg;
            start_lat = parse_coord( argv[iarg] ) * SGD_DEGREES_TO_RADIANS;
        } else if ( !strcmp(argv[iarg],"--home-lon") ) {
            ++iarg;
            home_lon = parse_coord( argv[iarg] ) * SGD_DEGREES_TO_RADIANS;
        } else if ( !strcmp(argv[iarg],"--home-lat") ) {
            ++iarg;
            home_lat = parse_coord( argv[iarg] ) * SGD_DEGREES_TO_RADIANS;
        } else if ( !strcmp(argv[iarg],"--swath-heading") ) {
            ++iarg;
            swath_hdg = SGD_2PI - atof( argv[iarg] ) * SGD_DEGREES_TO_RADIANS;
        } else if ( !strcmp(argv[iarg],"--swath-length") ) {
            ++iarg;
            swath_len = atof( argv[iarg] );
        } else if ( !strcmp(argv[iarg],"--swath-width") ) {
            ++iarg;
            swath_width = atof( argv[iarg] );
        } else if ( !strcmp(argv[iarg],"--cross-heading") ) {
            ++iarg;
            cross_hdg = SGD_2PI - atof( argv[iarg] ) * SGD_DEGREES_TO_RADIANS;
        } else if ( !strcmp(argv[iarg],"--wind-from-deg") ) {
            ++iarg;
            wind_dir = atof( argv[iarg] );
        } else if ( !strcmp(argv[iarg],"--wind-speed-kts") ) {
            ++iarg;
            wind_speed = atof( argv[iarg] );
        } else if ( !strcmp(argv[iarg],"--flight-speed-kts") ) {
            ++iarg;
            flight_speed = atof( argv[iarg] );
        } else if ( !strcmp(argv[iarg],"--flight-time-min") ) {
            ++iarg;
            flight_time = atof( argv[iarg] );
        } else if ( !strcmp(argv[iarg], "--help") ) {
            usage( argv[0] );
        } else {
            printf("Unknown option \"%s\"\n\n", argv[iarg]);
            usage( argv[0] );
        }
    }

    // Generate route waypoints
    SGRoute route;
    double time_remaining = flight_time;
    double ete_home = 0.0;
    int state = 0;

    Point3D current( start_lon, start_lat, 0.0 );
    route.add_waypoint( SGWayPoint(current.lon(), current.lat()) );

    // subtract out distance from launch/recover point (aka home) to
    // first waypoint
    Point3D home( home_lon, home_lat, 0.0 );
    double course, dist, ete;
    calc_gc_course_dist( current, home, &course, &dist );
    ete = calc_ete_min(course, dist, flight_speed, wind_speed, wind_dir);
    time_remaining -= ete;
    printf("time remaining = %f\n", time_remaining);

    while ( true ) {
        // once we determine the next leg distance below, we can
        // decide if it is time to break out of this loop
        
        double hdg = swath_hdg;
        double dist = swath_len;
        switch ( state ) {
        case 0:
            hdg = swath_hdg;
            dist = swath_len;
            break;
        case 1:
            hdg = cross_hdg;
            dist = swath_width;
            break;
        case 2:
            hdg = (swath_hdg - SGD_PI);
            if ( hdg < 0.0 ) { hdg += SGD_2PI; }
            dist = swath_len;
            break;
        case 3:
            hdg = cross_hdg;
            dist = swath_width;
            break;
        default:
            break;
        }

        // compute the next waypoint
        Point3D next = calc_gc_lon_lat( current, hdg, dist );
        ete = calc_ete_min(hdg, dist, flight_speed, wind_speed, wind_dir);

        // compute distance to return home
        double dist_home;
        calc_gc_course_dist( next, home, &course, &dist_home );
        ete_home = calc_ete_min(course, dist_home, flight_speed,
                                wind_speed, wind_dir);

        // finish the loop if we've run out of flight time
        if ( ete_home + ete > time_remaining ) {
            break;
        }

        route.add_waypoint( SGWayPoint(next.lon(), next.lat()) );
        
        current = next;
        time_remaining -= ete;

        state = (state + 1) % 4;
    }
    route.add_waypoint( SGWayPoint(home_lon, home_lat) );

    printf("\n");
    printf("Route summary:\n");
    printf("  waypoints: %d\n", route.size());
    printf("  route length: %.0f meters\n", route_length( route ) );
    printf("  max distance from home: %.0f meters\n", route_max_dist( route ) );

    int size = route.size();
    int i;
    FILE *fd;

    // Output route for gnuplot
    fd = fopen("route.gnuplot", "w");
    for ( i = 0; i < size; ++i ) {
        SGWayPoint wpt = route.get_waypoint( i );
        fprintf(fd, "%.8f %.8f\n",
               wpt.get_target_lon() * SGD_RADIANS_TO_DEGREES,
               wpt.get_target_lat() * SGD_RADIANS_TO_DEGREES );
    }
    fclose(fd);
    
    // Output route for microgear
    fd = fopen("route.xml", "w");
    fprintf(fd, "<?xml version=\"1.0\"?>\n");
    fprintf(fd, "<PropertyList>\n");
    for ( i = 0; i < size; ++i ) {
        SGWayPoint wpt = route.get_waypoint( i );
        fprintf(fd, "  <wpt>\n");
        fprintf(fd, "    <lat>%.8f</lat>\n",
                wpt.get_target_lat() * SGD_RADIANS_TO_DEGREES );
        fprintf(fd, "    <lon>%.8f</lon>\n",
                wpt.get_target_lon() * SGD_RADIANS_TO_DEGREES );
        fprintf(fd, "  </wpt>\n");
    }
    fprintf(fd, "</PropertyList>\n");
    fclose(fd);
    
    // Output route for gnuplot
    fd = fopen("route.pln", "w");
    for ( i = 0; i < size; ++i ) {
        SGWayPoint wpt = route.get_waypoint( i );
        fprintf(fd, "G %.8f %.8f 0.0 WP%02d\n",
                wpt.get_target_lon() * SGD_RADIANS_TO_DEGREES,
                wpt.get_target_lat() * SGD_RADIANS_TO_DEGREES,
                i );
    }
    fclose(fd);

    return 0;
}
