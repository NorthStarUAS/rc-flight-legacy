auralink is a reimplmentation of the original uglink written entirely
in python.  uglink was/is a jack of all trades data handler that could
read data live from the UAS and serve it out to the ground station,
and also replay flights and do various other data processing tasks.
uglink evolved since the very earliest days of this project so it is
extremely messy and convoluted, trying to do way too many things at
once.  auralink is a fresh start, all in python.

Some notable improvements:

- Better websocket support which should now support iOS/safari based clients
- Smarter (and far less cpu intensive) serial polling interface
- Packet packing and unpacking code now shared between base and remote for
  easier maintenance and future packet changes.
- Support for multiple sensor channels (i.e. more than one gps or imu or
  air data sensor.)
- Written entirely in python
- More robust command uplink sequence tracking which reduces the amount of
  wasted resends and improves throughput when multiple messages are queued.
