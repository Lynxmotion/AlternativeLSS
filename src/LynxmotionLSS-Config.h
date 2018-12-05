
#pragma once


#define INTERPACKET_DELAY   1

// maximum amount of time we will wait for a response from a servo before considering it a timeout
#define TRANSACTION_TIMEOUT   50000u

// if a servo fails to respond to this number of packets in a row, we consider it unresponsive
// if a servo responds, this counter is reset to zero
#define UNRESPONSIVE_REQUEST_LIMIT  2

// amount of milliseconds a servo gets disabled when found unresponsive
// after this timout elapses, we will give the servo another chance to respond.
#define UNRESPONSIVE_DISABLE_INTERVAL   15000u

///\brief Enable to include toString() functions that require the Arduino String class.
/// If you don't define this you will still have functions to fill a C-style char buffer. You lack the convenience 
/// of Arduino String but you don't get the code-space penalty.
#define HAVE_STRING

///\brief Enable this option to log all bus transactions to a Serial port.
/// by default we log to the first serial port 'Serial' which all Arduino's have, but you can change this to any 
/// Arduino stream supporting class.
//#define LSS_LOGGING Serial
//#define LSS_LOG_PACKETS
//#define LSS_LOG_SERVO_DISPATCH

///\brief Generate a pulse on a pin when bus read errors occur.
/// If defined and set to a GPIO pin, the library will toggle this pin anytime a read error has occured. This is
/// great for tracing electrical errors using an oscilloscope. Hook the GPIO pin to the trigger input of your oscope.
//#define LSS_OSCOPE_TRIGGER_PIN 4

/// \brief Tracks statistics on bus transactions (NOT YET IMPLEMENTED)
/// Statistics will include bus transaction counts, timing, read errors, and servo contention. You can read the
/// stats structure directly, or print it using LssChannel::printStats(Stream&).
#define LSS_STATS 1
