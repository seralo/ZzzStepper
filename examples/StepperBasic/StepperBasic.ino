#include <ZzzStepper.h>

// 28BYJ-48 stepper motor gear ratio 64:1 (per manufacturer specs) 4096 steps
ZzzStepper < ZzzStepperDriver4Pins<16,17,25,26> > stepper(4096);

//Wave mode use less steps only one electromagnet at a time => lower consumption but less torque and less precisions
//28BYJ-48 stepper motor gear ratio 64:1. Force step microseconds 28BYJ-48 stepper motor 600 - 1465 microseconds to avoid motor damage 
//ZzzStepper < ZzzStepperDriver4Pins<PIN1,PIN2,PIN3,PIN4, 1800, 2600, ZzzStepperMode4PinsWave> > stepper(2048);


int nbTurn;
void turnEnded() {
	nbTurn++;
	Serial.print("Turn:");
	Serial.println(nbTurn);

	//set rotation speed
	stepper.setSpeed(20);

	//alternate rotation
	stepper.turn(nbTurn % 2 ? -1 : 1, turnEnded);
}

void setup() {
	Serial.begin(115200);
	delay(250); //to ensure correctly initialized

	Serial.println("Stepper demo");

	nbTurn=0;
	//ask to make 1 full turn and call function turnEnded() at the end 
	stepper.turn(1, turnEnded);
}

void loop() {
	stepper.update();
}
