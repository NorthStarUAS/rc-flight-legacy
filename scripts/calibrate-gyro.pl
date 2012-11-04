#!/usr/bin/perl

use strict;

my($pi) = 3.14159265358979323846;

my($file) = shift;
my($start) = shift;
my($end) = shift;
my($revs) = shift;

print "data file = $file\n";
print "time range = $start to $end\n";
print "revolution count = $revs\n";

open( IN, "<$file" ) || die "Cannot open $file\n";

my($last_t) = 0.0;
my($p_total, $q_total, $r_total) = (0.0, 0.0, 0.0);

while ( <IN> ) {
    my(@tokens) = split(/\s+/);
    my($t) = $tokens[0];
    my($dt) = $t - $last_t;
    my($p) = $tokens[1];
    my($q) = $tokens[2];
    my($r) = $tokens[3];

    if ( $t >= $start && $t <= $end ) {
	$p_total += $p * $dt;
	$q_total += $q * $dt;
	$r_total += $r * $dt;
    }

    $last_t = $t;
}

my($exp_total) = $revs * $pi * 2;
printf("Expected total = %.3f\n", $exp_total);
printf("Totals = %.3f %.3f %.3f\n", $p_total, $q_total, $r_total);
printf("Error = %.4f %.4f %.4f\n",
       $p_total/$exp_total, $q_total/$exp_total, $r_total/$exp_total);
