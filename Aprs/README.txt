NOTE: USE Arduino 1.6.9 1.6.5 doesn't work.
aprs library

==============================
Status:
Modified Jun 15 2016 by KN added ground speed and course functions
Modified Jun 14 2016 by KN to add some functions. Fixed the [if altitude larger than int --> long] problem.
Newly created Jun 11 2016 by KN
===============================
Now has six functions:
.getAltitude();
.getLongitude();
.getLatitude();
.getCallsign();
.getGroundspeed();**
.getCourse();**

Check out the example aprs.ino for details
**two different implementations of the same thing, neither pretty. Should be fixed.
