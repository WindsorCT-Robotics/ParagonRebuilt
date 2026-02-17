# LED Subsystem

The LED Subsystem will control a series of LED strips on the robot that will match the alliance color during a match, red for red alliance, blue for blue alliance.

## Depedancies

Requires addressable LEDs.

## Public APIs

## Code Structure

### Commands

#### setAlliedColor

`setAlliedColor` Sets LED strips to current match alliance color.

#### setSingleLEDColor

`setSingleLEDColor` Sets a single valid LED address to any color.

#### turnLEDOff

`turnLEDOff` Turns off all LED strips.  