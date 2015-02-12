ArduRoller
==========

This repo contains the source code and assets for my ArduRoller balance bot.

<a href="http://www.flickr.com/photos/fasaxc/5932277215/" title="ArduRoller out for a spin by fasaxc, on Flickr"><img src="http://farm7.static.flickr.com/6014/5932277215_88f5a6aed2.jpg" width="333" height="500" alt="ArduRoller out for a spin"></a>

Here's a video of it in action: http://flic.kr/p/a4iTvU

Specs:

*  Chassis: laser cut 2.7mm bamboo ply (Ponoko); various M2.5 machine screws from Amazon; Instamorph low-melt-point thermoplastic to fill in the gaps.
*  Brains: 1 x Arduino Uno
*  Motor driver: 1 x Sparkfun Ardumoto
*  Motors: Sparkfun 2 x 24:1 gearmotor
*  Wheels: 1 set Sparkfun 70mm -- repaired with Instamorph after they cracked around the axle
*  Gyro: 1 x ADXRS613 (Sparkfun breakout) mounted at the axis of rotation
*  Accelerometer: 1 x ADXL203CE (Sparkfun breakout) mounted at the axis of rotation
*  Batteries: 2 x 3.7V Li-poly 850MAh (Sparkfun)

Code
----

The code is in main.c.  The code only depends on avr-libc and it includes 
the proect files for the AVR plugin for Eclipse.

The code has 3 parts:

* `main()` is the entry point, which runs
* `setup()` which does start-of-day initialization.
*  `ISR(TIMER1_OVF_vect)` the interrupt service routine for the timer 1 interrupt.
   The ISR is the main workhorse function.  It runs several hundred times per 
   second. 

There are a lot of magic values in the code that took me considerable time to
tune.  I also find that some of them aren't constant (e.g. the gyro drift 
changes with temperature and battery voltage).  If you try to make one of these
it'd probably be a good idea to pick my brains before you start or to use
a more standard approach like a complementary filter.

Assets
------

The diagram for the chassis is in the assets directory.  I used Ponoko to 
manufacture it in (2.7mm blonde bamboo).  

<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/3.0/"><img alt="Creative Commons License" style="border-width:0" src="http://i.creativecommons.org/l/by-nc-sa/3.0/88x31.png" /></a><br /><span xmlns:dct="http://purl.org/dc/terms/" href="http://purl.org/dc/dcmitype/StillImage" property="dct:title" rel="dct:type">ArduRoller Chassis</span> by <span xmlns:cc="http://creativecommons.org/ns#" property="cc:attributionName">Shaun Crampton</span> is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/3.0/">Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License</a>.

*WARNING* It's prototype standard -- 
I had to rebuild the motor mounts with Instamorph because they were too loose 
in one direction and sand them heavily because they were too tight in the other.
