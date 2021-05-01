# ZzzMovingAvg
Simple Arduino / C++ library to control stepper motor


The library consist of a single header file (ZzzStepper.h) containing template classes.


### Constructor

```cpp

ZzzStepper <DRIVER> stepper(stepsPerTurn,rpm=60,stepsPerMm=0); //Constructor need a driver class as template param


```

### Functions

```cpp
void go(clockwise=true)  // Turn until stop (async function need to call update() frequently)
void update()            // To call in Arduino loop
void stop()              // Stop the stepper motor

void step(steps=1, endActionCallback=null) // Number of steps to perform. Steps can be negative to go backward.
void turn(steps=1, endActionCallback=null) // Number of full turn to perform. Turns can be negative to go backward.

void goMs(ms, clockwise=true, endActionCallback=null) // Turn motor for given milliseconds

void travelMm(mm=1, endActionCallback=null) // Number of millimeters to travel. mm can be negative to go backward. (stepsPerMm need to be correct during constructor initialization or using setStepsPerMm())

void setStepsPerMm(stepsPerMm) // Set number of steps to travel 1mm
void setSpeed(rpm)             // Set rotation speed per minute (RPM) Driver will adjust to best suitable RPM to avoid motor damage.

```

### Included examples

- `StepperBasic/StepperBasic.ino` - Show basic usage counting turns and changing rotation direction every full turn


### Simple code example for 28BYJ-48 stepper motor using ULN2003 driver board 

```cpp
#include <ZzzStepper.h>

//ULN2003 use 4 pins connected here to PIN 16, PIN 17, PIN 25, PIN 26
//28BYJ-48 stepper motor needs 4096 steps to make one full turn in Half step mode
ZzzStepper < ZzzStepperDriver4Pins<16,17,25,26> > stepper(4096);

void setup()
{
    ...
    stepper.go();
}

void loop()
{
    ...

    stepper.update();

    ...
}
```

