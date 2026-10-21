# Honda-Ignition-Controller
Designed on my 1997 Honda Prelude with an H22A + chipped P28 ECU over the course of 6 weeks (3 months)

Arduino - P28
Pin 2 - A21 5v 10k ohm pullup or through FC1 daughter/knock board slot 5 on P28/Pxx motherboard
Pin 8 - (CYP sensor) through FC1 daughter/knock board slot 4 on P28/P## motherboard
Pin 4 -  out A straight to coil pair A triggers
Pin 5 - out B straight to coil pair B triggers

Power:
modified a 12v cig adapter and hooked it up to a switch and a reliable *constant* 12v source, and chassis ground
considering using the FC1 VCC slot, Pin 1, to power the Arduino and give it a reliable 5v that also doesn't fluctuate during cranking.  

Grounds:
Tap a reference ground LG1 from the connectors or solder into slot 10/11 on FC1 and tie it to the Arduino

if i forget anything it'll be in the write up

