#define CHANNEL_AMOUNT 16
#define DETECTION_SPACE 2500

struct {
  short* channels;
  byte count;
  short pin;
  bool invert;
} PPM;

byte ppm_count() {
  return PPM.count;
}

short ppm_channel(byte chnumber) { 
  return PPM.channels[chnumber];
}

void ppm_start(int pin, bool invert) {
  pinMode(pin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(RECEIVE_PIN), ppm_interrupt, METHOD);
  PCattachInterrupt(pin, ppm_interrupt, CHANGE);
  memset(&PPM, 0, sizeof(PPM));
  PPM.pin = pin;
  PPM.invert = invert;
}

void ppm_stop() {
  if(PPM.pin) {
    PCdetachInterrupt(PPM.pin);
    memset(&PPM, 0, sizeof(PPM));
  }
}

void ppm_interrupt()
{
  static short channels[CHANNEL_AMOUNT * 2];  // declare 2 pages so we write to one while the client can read from the other
  static short* ch_write_base = channels;     // current page write position
  static byte i;
  static unsigned long int t_old;

  // read the pin, ignore falling transitions because the PPM time is the sum of 
  // the high and low pulse. I.e. the pulse initiates the channel timer and the
  // next pulse ends the timer and starts the next channel timer.
  bool pinHigh = digitalRead(PPM.pin)==HIGH;
  if(pinHigh == PPM.invert) 
    return;

  // calculate the pulse width for this channel
  unsigned long int t = micros(); //store time value a when pin value falling/rising
  unsigned long int dt = t - t_old; //calculating time inbetween two peaks
  t_old = t;

  // detect last channels and go back to first
  if ((dt > DETECTION_SPACE) || (i > CHANNEL_AMOUNT)) {
    PPM.channels = ch_write_base;
    PPM.count = i;
    i = 0;

    // switch pages
    ch_write_base = (ch_write_base == channels)
      ? (channels + CHANNEL_AMOUNT)  // write to second page
      : channels;                    // write to first page
  } else
    ch_write_base[i++] = dt;
}
