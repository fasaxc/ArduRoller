This repo contains the source code and assets for my ArduRoller balance bot.

<a href="http://www.flickr.com/photos/fasaxc/5932277215/" title="ArduRoller out for a spin by fasaxc, on Flickr"><img src="http://farm7.static.flickr.com/6014/5932277215_88f5a6aed2.jpg" width="333" height="500" alt="ArduRoller out for a spin"></a>

Specs:
-  Chassis: laser cut 2.7mm bamboo ply (Ponoko); various M2.5 machine screws from Amazon; Instamorph low-melt-point thermoplastic to fill in the gaps.
-  Brains: 1 x Arduino Uno
-  Motor driver: 1 x Sparkfun Ardumoto
-  Motors: Sparkfun 2 x 24:1 gearmotor
-  Wheels: 1 set Sparkfun 70mm -- repaired with Instamorph after they cracked around the axle
-  Gyro: 1 x ADXRS613 (Sparkfun breakout) mounted at the axis of rotation
-  Accelerometer: 1 x ADXL203CE (Sparkfun breakout) mounted at the axis of rotation
-  Batteries: 2 x 3.7V Li-poly 850MAh (Sparkfun)

The code is in main.cpp.  I develop in Eclipse rather than the Arduino IDE but
it should still work if you copy and paste into the Arduino IDE as a sketch.

The code has 3 parts:
-  *setup()* the standard Arduino setup routine, run once at start of day.
-  *loop()* the standard Arduino loop function, does nothing by default.
-  *ISR(TIMER1_OVF_vect)* the interrupt service routine for the timer 1 interrupt.
   The ISR is the main workhorse function.  It runs several hundred times per 
   second. 

The diagram for the chassis is in the assets directory.  I used Ponoko to 
manufacture it in (2.7mm blonde bamboo).  *WARNING* It's prototype standard -- 
I had to rebuild the motor mounts with Instamorph because they were too loose 
in one direction and sand them heavily because they were too tight in the other.