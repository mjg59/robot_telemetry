Robot telemetry firmware
------------------------

This code reads data from an accelerometer and four VESCs, writes it to SD
card, and transmits it to another machine for recording and visualisation.

IMPORTANT NOTE
--------------

This requires the ADXL372 module from https://github.com/mjg59/ADXL372

Theory
------

At startup, 4 serial ports are configured for communication with the VESCs.
These ports are set to use inverted signalling, in order to cope with the
opto-isolators in the circuit. A command is sent to each, corresponding to
the VESC "Get Values" command.

Another serial port is configured for over the air communication. This is
not connected via opto-isolators, and so does not need signals to be inverted.
However, the buffer size for serial transmission is increased.

An analogue pin is configured with a PWM frequency of 10kHz and a duty cycle
of 10% in order to generate a 10kHz timer signal for external synchronisation.

If an SD card is present, the set of existing files will be examined and the
first available number between 0 and 4095 will be used as a filename. This
will be used to log data locally for later retrieval.

Finally, an SPI-attached ADXL372 will be configured to provide accelerometer
data.

Events
------

Event handlers are registered for each VESC serial port. Upon a byte of data
being received by the hardware, the handler will be called and the byte will
be read into the corresponding VESC's buffer. The data is parsed to ensure
that it is valid, and the expected length of the packet is extracted from the
data provided by the VESC. Once the entire packet has been read, the
corresponding VESC's local structure is updated to indicate that it is complete
and a timestamp is set to show the time of completion. The data is then
written out to SD card.

Main loop
---------

The main loop cycles through each VESC structure in turn, looking for ones
that have complete data packets. If it finds more than one, it will pick the
one that has the oldest data. It will then check whether the telemetry
serial port is currently sending any data, by checking whether the buffer is
empty.  If it isn't, it will write the data. This will not block, since the
hardware is capable of performing serial writeout in the background. If data
is currently being written, the loop will continue.

If a VESC hasn't updated in over a second, the code will assume that something
has gone wrong, reinitialise local state and send another query command.

If data was previously being written but the output serial buffer is now
empty, the VESC whose data was being output is sent another query command in
order to trigger a new data query.

The accelerometer, if present, will then also be read and the values
compared to the last transmitted acceleration data. If greater than a
threshold difference, the accelerometer data will be transmitted and written
to SD card.

Finally, the loop will check whether the remote machine sent an enter byte
three times in a row in under two seconds. If so, it will enter interactive
mode. In this mode no live telemetry data is sent - instead, the user can
send commands to list the files on the SD card, dump the contents of a file
or wipe files from the SD card. This allows recovery of logged data without
having to disassemble the board.