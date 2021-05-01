/***************************************************************
  ZzzStepper.h
***************************************************************/
#ifndef ZZZ_STEPPER_H
#define ZZZ_STEPPER_H

typedef void(*ZzzStepperCallback)();


template <size_t NBPINS, size_t NBSTEPS, int ... STEPS> class ZzzStepperMode {
	protected:
		/** Current step */
		int _curStep=0;
		const int pSteps[NBSTEPS*NBPINS]={STEPS...};
	public:
		int getSteps() {
			return NBSTEPS;
		}

		int getPins() {
			return NBPINS;
		}

		/**
		 * @return pointer to an array of values contains at least NBPINS values
		 */
		const int* pinStates() {
			return &(pSteps[_curStep*NBPINS]);
		}
		
		int nextStep(bool cw=true) {
			if (cw) {
				_curStep++;
				if (_curStep>=NBSTEPS) {
					_curStep=0;
				}
			} else {
				_curStep--;
				if (_curStep<0) {
					_curStep=NBSTEPS-1;
				}
			}
			return _curStep;
		}
};


/** Wave driver: Use less steps only one magnet at a time => lower consumption but less torque */
typedef ZzzStepperMode<4, 4,  1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1 > ZzzStepperMode4PinsWave;
/** Half step driver: More steps, more precise movements */
typedef ZzzStepperMode<4, 8,  1,0,0,0,  1,1,0,0,  0,1,0,0,  0,1,1,0,  0,0,1,0,  0,0,1,1,  0,0,0,1,  1,0,0,1 > ZzzStepperMode4PinsHalf;


template <int PIN1, int PIN2, int PIN3, int PIN4, typename MODE=ZzzStepperMode4PinsHalf> class ZzzStepperDriver4Pins {
	protected:
		MODE _mode;
	public:
		ZzzStepperDriver4Pins() {
			pinMode(PIN1, OUTPUT);
			pinMode(PIN2, OUTPUT);
			pinMode(PIN3, OUTPUT);
			pinMode(PIN4, OUTPUT);
		}

		/** Send stop command to stepper */
		void stop(bool force=false) {
			if (force) {
				digitalWrite(PIN1, 0);
				digitalWrite(PIN2, 0);
				digitalWrite(PIN3, 0);
				digitalWrite(PIN4, 0);
			}
		}

		/** Ask stepper to go next step or previous step depending on clockwise */
		void nextStep(bool cw) {
			const int *pValues;

			_mode.nextStep( cw );
			pValues=_mode.pinStates();

			/*Serial.print("updatePins:");
			Serial.print(PIN1); Serial.print("="); Serial.print(pValues[0]); Serial.print(" ");
			Serial.print(PIN2); Serial.print("="); Serial.print(pValues[1]); Serial.print(" ");
			Serial.print(PIN3); Serial.print("="); Serial.print(pValues[2]); Serial.print(" ");
			Serial.print(PIN4); Serial.print("="); Serial.print(pValues[3]); Serial.print(" ");
			Serial.println();*/
			
			digitalWrite(PIN1, pValues[0]);
			digitalWrite(PIN2, pValues[1]);
			digitalWrite(PIN3, pValues[2]);
			digitalWrite(PIN4, pValues[3]);
		}
};



/**
 * Template class to manage a stepper motor. The template need a Driver parameter to control the stepper.
 * The driver class must implement stop(bool force) and nextStep(bool cw).
 */
template <typename DRIVER> class ZzzStepper {
	protected:
		/** Stop state: all flags set */
		static const int STATE_STOP=-1;
		/** Clockwise */
		static const int STATE_CW=1;
		/** Counter-clockwise */
		static const int STATE_CCW=0;
		
		static const int STATE_GO_STEPS=4;
		static const int STATE_GO_TIME=8;

		/** Bit state (CW, CCW) Steps, continuous */
		int _state;

		/** Duration of a step in microseconds */
		unsigned long _stepTimeUs;
		unsigned long _lastStepUs;

		/** Counter for asyncStep or asyncTurn or asyncGoMm valid when state has STATE_GO_STEPS flag set */
		unsigned long _remainingSteps;
		/** Timer for asyncGoMs (2 values to make it overflow proof) */
		unsigned long _timerStartMs;
		unsigned long _timerDurationMs;
		
		int _stepsPerTurn;
		int _stepsPerMm;

		DRIVER _driver;

		/** Callback called at the end of asyncStep or asyncTurn or asyncGoMm or asyncGoMs  */
		ZzzStepperCallback _endActionCallback;
		
		void endAction() {
			_driver.stop(false);
			_state=STATE_STOP;
			if (_endActionCallback!=nullptr) {
				_endActionCallback();
			}
		}

	public:		
		/** Stop the stepper motor */
		void stop() {
			if (_state!=STATE_STOP) {
				_driver.stop(true);
			}
			_state=STATE_STOP;
		}

		/** To call in Arduino loop for async methods */
		void update() {
			if (_state==STATE_STOP) {
				return; //nothing to do
			}
			//check elapsed time (overflow proof)
			if (micros() - _lastStepUs > _stepTimeUs) {
				if ((_state & STATE_GO_STEPS)==STATE_GO_STEPS) {
					if (_remainingSteps==0) {
						endAction();
						return;
					}
					_remainingSteps--;
				}
				if ((_state & STATE_GO_TIME)==STATE_GO_TIME) {
					if (millis() - _timerStartMs >_timerDurationMs) {
						endAction();
						return;
					}
				}
				_driver.nextStep( ((_state & STATE_CW) == 1) );
				_lastStepUs=micros();
			}
		}

		/** Set rotation speed per minute (RPM) */
		void setSpeed(int rpm) {
			_stepTimeUs=1000; //TODO compute
		}
		
		/** Set number of steps to perform to travel 1mm (useful in combination with ) */
		void setStepsPerMm(int stepsPerMm) {
			_stepsPerMm=stepsPerMm;
		}

		/** Turn until stop (async function need to call update() frequently) */
		void go(bool cw=true) {
			_state=(cw ? STATE_CW : STATE_CCW);
		}

		/**
		 * Number of steps to perform (async function need to call update() frequently)
		 * @steps the number of steps to turn the motor - positive to turn one direction, negative to turn the other
		 */		
		void step(long steps=1, ZzzStepperCallback endActionCallback=nullptr) {
			bool cw=(steps>0);
			_remainingSteps=(steps>=0) ? steps : -steps;
			_state=(cw ? STATE_CW : STATE_CCW) | STATE_GO_STEPS;
			_endActionCallback=endActionCallback;
		}

		/**
		 * Number of full turn to perform (async function need to call update() frequently)
		 * @turns the number of full turn to perform - positive to turn one direction, negative to turn the other
		 */
		void turn(long turns=1, ZzzStepperCallback endActionCallback=nullptr) {
			step(turns*_stepsPerTurn, endActionCallback);
		}

		/**
		 * Number of millimeters to travel (async function need to call update() frequently)
		 * @turns the number of millimeters to travel - positive to travel one direction, negative to travel the other
		 */
		void goMm(long mm, ZzzStepperCallback endActionCallback=nullptr) {
			step(mm*_stepsPerMm, endActionCallback);
		}

		/** Number of milliseconds (time) to run (async function need to call update() frequently) */
		void goMs(unsigned long ms, bool cw=true, ZzzStepperCallback endActionCallback=nullptr) {
			_timerDurationMs=ms;
			_timerStartMs=millis();
			_state=(cw ? STATE_CW : STATE_CCW) | STATE_GO_TIME;
			_endActionCallback=endActionCallback;
		}
				
		/** Constructor */
		ZzzStepper(int stepsPerTurn, int stepsPerMm=0) {
			_state=STATE_STOP;
			_stepsPerTurn=stepsPerTurn;
			if (stepsPerMm<=0) {
				_stepsPerMm=stepsPerTurn;
			}
			_stepTimeUs=1500; //NOTE max 1000 ?
		}
};

#endif

