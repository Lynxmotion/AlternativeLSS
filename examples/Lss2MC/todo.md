### LSS 2MC Code


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
MM??       Motor Mode (Dual Brushed or Stepper)
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





Todo:
- process_packet checks ID and jumps to device type. We have only 1 ID, so change this to just jump on motor_mode
- probably get rid of LssDevice stuff, not needed (Handlers are written to take a LssDevice ref)
- how do we implement 2 motors? ID and ID+1? or all set/gets take/return two values
- gyre direction is in Common but will have to be moved to each dev


set graph to full view





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


