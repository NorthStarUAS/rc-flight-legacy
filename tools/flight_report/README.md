# Flight Report

The flight_report.py script has three primary goals.

* Produce a flight summary for logging purposes.
* Lookup the weather conditions at the time and location of the flight.
* Generate plots of all the sensor and system state data for a quick
  overview and to assist in validating and debugging the system.

The script is not intended to cover every possible use case or sensor
combination, but is written in python and thus straightforward to add
new plots or plot new combinations of data.