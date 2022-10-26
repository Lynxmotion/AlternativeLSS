- ### LSS 2MC Code


# Stepper Mode Features
- angle or position mode


# Stepper Commands
** indicates commands that are not part of the standard LSS Communication Protocol
**???      Set StepsPerRev (and set full, half stepper?)
MD         Relative Move in Degrees (1/10th degs using steps/rev)
WD         [DONE] Wheel Mode - continuous rotation (degs/sec)
Q          Query Status
D          Position in Degrees
QD         Query Position (degrees)
L          [DONE] Limp
H          [DONE] Halt and Hold

set acceleration???

# Dual Brushed Motor Commands
WD         Wheel Mode - continuous rotation (degs/sec)  (I probably can't do this since there is no encoder/feedback)
RDM        Raw Duty-cycle Move
Q          Query Status
L          Limp


# Global Commands:
CMM??      Motor Mode (Dual Brushed or Stepper)
SD         Max Speed in Deg/Sec
SR         Max Speed in RPM
QMS        "LSS-2MC-ST" (stepper mode) or "LSS-2MC-DBR" (dual brushed mode)
QFV        "major.minor.build"
QID/CID
QB/CB
G/QG/CG    Gyre Direction
RESET
QN         ???


# Modifiers?
S          Speed
SD         Speed n deg/sec
T          Timed move  (really, just no)


# Examples:
MM0   - set mode to stepper
MM1   - set mode to dual brushed




Todo:
- gyre direction is in Common but will have to be moved to each dev

set graph to full view

# Dual Brushed Motor Mode (#215 or #216)

CMM1   		// enter dual brushed mode
QMM		// verify motor mode
RDM255		// set duty cycle to max (anything over 255 is truncated to 255)
RDM0 		// stop, same as L
L               // coast to a stop
H               // stop w/ brake

# Decay Rates
See the MP6508 datasheet or other motor control material for description of slow vs fast
decay rates. 
For slow decay mode the current circulates through the two low-side MOSFETs and has a
braking effect. For fast decay mode, the current flows through the body diodes of the
other diagonal two MOSFETS.

WR0		// fast decay rate (default)
WR1		// slow decay rate
QWR		// query the current decay rate




# Fixes

## cast of LED color property
 #define DISPATCH(bit, member) if ((cmd & bit)>0) { LSS_LOGGING.print("  " #bit " "); LSS_LOGGING.print(pkt.value); LSS_LOGGING.print(" => " #member); member = pkt.value; mask.completed |= bit; }
 #else
 #define DISPATCH(bit, member) if ((cmd & bit)>0) { member = pkt.value; mask.completed |= bit; }
+#define DISPATCH_CAST(bit, member, type) if ((cmd & bit)>0) { member = (type)pkt.value; mask.completed |= bit; }
 #endif
 
 void LynxServo::dispatch(LynxPacket pkt)
@@ -563,7 +564,7 @@ void LynxServo::dispatch(LynxPacket pkt)
       DISPATCH(LssFirstPosition, config->firstPosition)
       else DISPATCH(LssGyreDirection, config->gyreDirection)
       else DISPATCH(LssBaudRate, config->baudrate)
-      else DISPATCH(LssLEDColor, config->ledColor)
+      else DISPATCH_CAST(LssLEDColor, config->ledColor, LssColors)
       else DISPATCH(LssAngularStiffness, config->angularStiffness)
       else DISPATCH(LssMaxSpeed, config->maxSpeed)
       else DISPATCH(LssAngularRange, config->angularRange)


## Attempting code sync/merges

On branch master
Your branch is behind 'origin/master' by 76 commits, and can be fast-forwarded.
  (use "git pull" to update your local branch)



commit be5a1382418db435c0b4b076ae32b7df45831c1f (HEAD -> master)
Author: Colin MacKenzie <colin@flyingeinstein.com>
Date:   Tue Dec 15 14:18:51 2020 -0500

    fixed flash write using old AngularRange command instead of LssAnalog




