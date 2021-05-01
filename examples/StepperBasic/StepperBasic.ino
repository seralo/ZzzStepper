#include <ZzzStepper.h>

/* 28BYJ-48 stepper motor gear ratio 64:1 (per manufacturer specs) 4096 steps */


//Use less steps only one electromagnet at a time => lower consumption but less torque and less precisions
//ZzzStepper < ZzzStepperDriver4Pins<16,17,25,26,ZzzStepperMode4PinsWave> > stepper(4096);
ZzzStepper < ZzzStepperDriver4Pins<16,17,25,26> > stepper(4096);

int nbTurn;
void turnEnded() {
	nbTurn++;
	Serial.print("Turn:");
	Serial.println(nbTurn);

	//alternate rotation
	stepper.turn(nbTurn % 2 ? -1 : 1, turnEnded);
}

void setup() {
	Serial.begin(115200);
	delay(250); //to ensure correctly initialized

	Serial.println("Stepper demo");
	
	//stepper.go(clockWise);
	nbTurn=0;
	stepper.turn(1, turnEnded);
}

void loop() {
	stepper.update();
}
