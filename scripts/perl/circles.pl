#!/usr/bin/perl

use Math::Trig;

require "telnet.pl";

use strict;

# script configuration
my( $server ) = "localhost";
my( $port ) = 6499;
my( $timeout ) = 5;

my( $range_nm ) = 0.25;
my( $interval ) = 240;
my( $speed ) = 30.0;

# constants
my( $pi ) = 3.1415926535897932384626433832795029;
my( $rad_to_deg ) = 180.0 / $pi;
my( $deg_to_rad ) = $pi / 180.0;

my( $home_lon_deg ) = 0.0;
my( $home_lat_deg ) = 0.0;
my( $xfact ) = 1.0

&get_home_pos();
$xfact = abs(cos($home_lat_deg * $deg_to_rad));
print "xfact=$xfact\n";

printf "Home = %.8f %.8f\n", $home_lon_deg, $home_lat_deg;

#while (1) {
#    do_random_fixed_range( $range_nm );
#    sleep( $interval );
#}

# main action here
#&make_discrete_spiraling_circles();
&make_continuous_spiraling_circles();

sub make_discrete_spiraling_circles() {
    my( $angle ) = 0.0;
    my( $range_nm ) = 0.1;
    my( $speed_kt ) = 25.0;
    while (1) {
	position_circle( $angle*$deg_to_rad, $range_nm, $speed_kt );
	
	$angle += 29.0;
	if ( $angle > 360.0 ) { $angle -= 360.0; }

	$range_nm += 0.01;

	$speed_kt += 1.0;
	if ( $speed_kt > 55.0 ) { $speed_kt = 55.0; }

	sleep( $interval );
    }
}

sub make_continuous_spiraling_circles() {
    #my( $dr ) = 0.000042;
    #my( $da ) = 0.125;
    #my( $ds ) = 0.0042;
    my( $dr ) = 0.001;
    my( $da ) = 0.2;
    my( $ds ) = 0.01;

    my( $angle ) = 0.0;
    my( $range_nm ) = 0.1;
    my( $speed_kt ) = 25.0;

    while (1) {
	position_circle( $angle*$deg_to_rad, $range_nm, $speed_kt );
	
	$angle += $da;
	if ( $angle > 360.0 ) { $angle -= 360.0; }

	$range_nm += $dr;

	$speed_kt += $ds;
	if ( $speed_kt > 55.0 ) { $speed_kt = 55.0; }

	sleep( 1 );
    }
}



sub do_random_fixed_range() {
    my( $range_nm ) = shift;

    print "Setting new circle target location...\n";
    my( $rand1 ) = rand(2*$pi);
    my( $rand2 ) = rand($range_nm);
    $rand2 = $range_nm;

    my( $dx_nm ) = sin($rand1) * $rand2;
    my( $dy_nm ) = cos($rand1) * $rand2;

    printf "angle=%.2f dist=%.3f  dx=%.4f dy=%.4f\n", $rand1 * $rad_to_deg, $rand2, $dx_nm, $dy_nm;

    my( $dx_deg ) = 0.0;
    if ( $xfact > 0.0001 ) {
	$dx_deg = ($dx_nm / 60.0) / $xfact;
    }
    my( $dy_deg ) = $dy_nm / 60.0;
    printf "dx_deg=%.8f dy_deg=%.8f\n", $dx_deg, $dy_deg;
	
    my( $target_lon_deg ) = $home_lon_deg + $dx_deg;
    my( $target_lat_deg ) = $home_lat_deg + $dy_deg;
    &set_circle_target( $target_lon_deg, $target_lat_deg );
}


sub position_circle() {
    my( $angle ) = shift;
    my( $range_nm ) = shift;
    my( $target_speed_kt ) = shift;

    print "Setting new circle target location...\n";
    my( $rand1 ) = $angle;
    my( $rand2 ) = $range_nm;

    my( $dx_nm ) = sin($rand1) * $rand2;
    my( $dy_nm ) = cos($rand1) * $rand2;

    printf "angle=%.2f dist=%.3f  dx=%.4f dy=%.4f\n", $rand1 * $rad_to_deg, $rand2, $dx_nm, $dy_nm;

    my( $dx_deg ) = 0.0;
    if ( $xfact > 0.0001 ) {
	$dx_deg = ($dx_nm / 60.0) / $xfact;
    }
    my( $dy_deg ) = $dy_nm / 60.0;
    printf "dx_deg=%.8f dy_deg=%.8f\n", $dx_deg, $dy_deg;
	
    my( $target_lon_deg ) = $home_lon_deg + $dx_deg;
    my( $target_lat_deg ) = $home_lat_deg + $dy_deg;
    &set_circle_target( $target_lon_deg, $target_lat_deg, $target_speed_kt );
}


sub get_home_pos {
    my( $fgfs );

    if ( !( $fgfs = &connect($server, $port, $timeout) ) ) {
        print "Error: can't open socket\n";
        return;
    }

    &send( $fgfs, "data" );     # switch to raw data mode

    $home_lon_deg = &get_prop( $fgfs, "/navigation/home/longitude-deg" );
    $home_lat_deg = &get_prop( $fgfs, "/navigation/home/latitude-deg" );

    &send( $fgfs, "quit" );
    close $fgfs;
}


sub set_circle_target {
    my( $lon_deg ) = shift;
    my( $lat_deg ) = shift;
    my( $target_speed_kt ) = shift;

    my( $command ) = "task,circle," . $lon_deg . "," . $lat_deg;

    my( $fgfs );

    if ( !( $fgfs = &connect($server, $port, $timeout) ) ) {
        print "Error: can't open socket\n";
        return;
    }

    &send( $fgfs, "data" );     # switch to raw data mode

    &set_prop( $fgfs, "/task/command-request", $command );
    &set_prop( $fgfs, "/autopilot/settings/target-speed-kt", $target_speed_kt );

    &send( $fgfs, "quit" );
    close $fgfs;
}
