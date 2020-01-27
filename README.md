# AlternativeLSS
High performance library for the Lynxmotion Smart Servos (LSS).

# Installation
Use git to clone this repository into your Arduino libraries folder. This is usually in your home Documents directory "Arduino/libraries". Then start Arduino. You should now have AlternativeLSS as a library and examples under the File | Examples | AlternativeLSS menu.

# Examples
There are 5 examples that use the AlternativeLSS library. You should verify the LSS_SERIAL_PORT (Serial1) and LSS_BAUDRATE (250k) and servo IDs at the top of the examples before compiling. The default serial monitor baud rate is 115200.
* BusScan - scans the bus for servos with ID 1 to 64. It continuously scans and reports a few properties of each servo such as position, target, temperature and voltage.
* SimpleRead - reads multiple properties at once from a single servo in a loop
* SimpleWrite - repeatidly moves a servo from -90 degrees to +90 degrees
* ActionUpdate - Requires two servos on the bus, a Slave servo will follow the position of a dedicated master servo. For best performance you should follow this example that requests updates from the server asynchronously, then on completion of the request issues commands to update servo position or other parameters. 
* UnitTests - runs software simulated unit tests to verify the integrity of the library. Can also run physical servo tests.

# Known Issues
* Since micros() wraps every ~70 minutes. We need to guard against this case somewhow. This might affect timeout detection.
