# AuraUAS Survey Areas

AuraUAS can receive the outline of an area to survey.  It will then
generate a coverage route based on current wind, survey altitude, and
camera field of view.

The goal here is to avoid the need to transmit hundreds of waypoints
up from the ground station.  Instead, let the aircraft do the smart
work.  The new aircraft route will then be trickled back down to the
ground station map.

## Notes

I created my own point, vector, line, and area classes so this module
would not need to depend on external modules (such as numpy or other
less well known geometry libraries.)

So far, it appears that turns work best when the route transacts are
perpendicular to the wind and the route incrementally works it's way
upwind.  That way all turns are upwind and wasted effort is minimized.