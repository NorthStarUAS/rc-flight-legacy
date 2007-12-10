#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <control/route.hxx>
#include <util/point3d.hxx>
#include <util/polar3d.hxx>

#include <include/globaldefs.h>

double wind_dir = 0.0;
double wind_speed = 0.0;


void usage( char *prog ) {
    printf("Usage: %s [ --options ]\n", prog);
    printf("  --start-lon <lon_decimal_deg>\n");
    printf("  --start-lat <lat_decimal_deg>\n");
    printf("  --home-lon <lon_decimal_deg>\n");
    printf("  --home-lat <lat_decimal_deg>\n");
    printf("  --swath-heading <initial swath heading deg true>\n");
    printf("  --swath-length <swath length meters>\n");
    printf("  --swath-width <width between swaths in meters>\n");
    printf("  --cross-heading <cross angle>\n");
    printf("         ideally this is into the wind and 90 degrees from the swath heading>\n");
    printf("  --wind-dir <wind 'from' direction>\n");
    printf("  --wind-speed <wind speed kts>\n");
    printf("  --flight-speed <estimated air speed kts>\n");
    printf("  --flight-dist <estimated flight distance in meters>\n");

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
    double flight_dist = 0.0;
    
    // Parse the command line
    for ( int iarg = 1; iarg < argc; iarg++ ) {
        if ( !strcmp(argv[iarg], "--start-lon" )  ) {
            ++iarg;
            start_lon = atof( argv[iarg] ) * SGD_DEGREES_TO_RADIANS;
        } else if ( !strcmp(argv[iarg],"--start-lat") ) {
            ++iarg;
            start_lat = atof( argv[iarg] ) * SGD_DEGREES_TO_RADIANS;
        } else if ( !strcmp(argv[iarg],"--home-lon") ) {
            ++iarg;
            home_lon = atof( argv[iarg] ) * SGD_DEGREES_TO_RADIANS;
        } else if ( !strcmp(argv[iarg],"--home-lat") ) {
            ++iarg;
            home_lat = atof( argv[iarg] ) * SGD_DEGREES_TO_RADIANS;
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
        } else if ( !strcmp(argv[iarg],"--wind-dir") ) {
            ++iarg;
            wind_dir = atof( argv[iarg] );
        } else if ( !strcmp(argv[iarg],"--wind-speed") ) {
            ++iarg;
            wind_speed = atof( argv[iarg] );
        } else if ( !strcmp(argv[iarg],"--flight-speed") ) {
            ++iarg;
            flight_speed = atof( argv[iarg] );
        } else if ( !strcmp(argv[iarg],"--flight-dist") ) {
            ++iarg;
            flight_dist = atof( argv[iarg] );
        } else if ( !strcmp(argv[iarg], "--help") ) {
            usage( argv[0] );
        } else {
            printf("Unknown option \"%s\"\n\n", argv[iarg]);
            usage( argv[0] );
        }
    }

    // Generate route waypoints'
    SGRoute route;
    double dist_remaining = flight_dist;
    int state = 0;

    Point3D current( start_lon, start_lat, 0.0 );
    route.add_waypoint( SGWayPoint(current.lon(), current.lat()) );

    while ( dist_remaining > 0.0 ) {
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

        Point3D next = calc_gc_lon_lat( current, hdg, dist );
        route.add_waypoint( SGWayPoint(next.lon(), next.lat()) );

        current = next;
        dist_remaining -= dist;
        state = (state + 1) % 4;
    }
    route.add_waypoint( SGWayPoint(home_lon, home_lat) );

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
