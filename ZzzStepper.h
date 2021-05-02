/***************************************************************
  ZzzStepper.h
***************************************************************/
#ifndef ZZZ_STEPPER_H
#define ZZZ_STEPPER_H

typedef void(*ZzzStepperCallback)();


template <size_t NB_PINS, size_t NB_STEPS, int ... STEPS> class ZzzStepperMode {
	protected:
		/** Current step */
		int _curStep=0;
		const int pSteps[NB_STEPS*NB_PINS]={STEPS...};
	public:
		int getSteps() {
			return NB_STEPS;
		}

		int getPins() {
			return NB_PINS;
		}

		/**
		 * @return pointer to an array of values contains at least NBPINS values
		 */
		const int* pinStates() {
			return &(pSteps[_curStep*NB_PINS]);
		}
		
		int nextStep(bool cw=true) {
			if (cw) {
				_curStep++;
				if (_curStep>=NB_STEPS) {
					_curStep=0;
				}
			} else {
				_curStep--;
				if (_curStep<0) {
					_curStep=NB_STEPS-1;
				}
			}
			return _curStep;
		}
};


/** Wave driver: Use less steps only one magnet at a time => lower consumption but less torque */
typedef ZzzStepperMode<4, 4,  1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1 > ZzzStepperMode4PinsWave;
/** Half step driver: More steps, more precise movements */
typedef ZzzStepperMode<4, 8,  1,0,0,0,  1,1,0,0,  0,1,0,0,  0,1,1,0,  0,0,1,0,  0,0,1,1,  0,0,0,1,  1,0,0,1 > ZzzStepperMode4PinsHalf;


/**
 * PIN1-4 specify the pins to control
 * MIN_US, MAX_US parameters allow limiting ste duration in microseconds to avoid motor damage (28BYJ-48 stepper motor 600 - 1465 microseconds)
 * MODE allow to change Wave or Half step driving
 */
template <int PIN1, int PIN2, int PIN3, int PIN4, unsigned long MIN_US=800, unsigned long MAX_US=1400, typename MODE=ZzzStepperMode4PinsHalf> class ZzzStepperDriver4Pins {
	protected:
		MODE _mode;
	public:
		ZzzStepperDriver4Pins() {
			pinMode(PIN1, OUTPUT);
			pinMode(PIN2, OUTPUT);
			pinMode(PIN3, OUTPUT);
			pinMode(PIN4, OUTPUT);
		}

		/** Return the step duration in microseconds to achieve given RPM. Return closest value to avoid motor damage. */
		unsigned long getStepUs(unsigned int rpm, int stepsPerTurn) {
			//1 min = 60 s per 1000 ms per 1000 us
			unsigned long d = (60 * 1000 * 1000) / (stepsPerTurn * rpm);
			if (d<MIN_US) {
				d=MIN_US;
			}
			if (d>MAX_US) {
				d=MAX_US;
			}
			return d;
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

			digitalWrite(PIN1, pValues[0]);
			digitalWrite(PIN2, pValues[1]);
			digitalWrite(PIN3, pValues[2]);
			digitalWrite(PIN4, pValues[3]);
		}
};



/**
 * Template class to manage a stepper motor. The template need a Driver parameter to control the stepper.
 * The driver class must implement stop(bool force), nextStep(bool cw) and getStepUs(rpm, stepsPerTurn).
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

		/** Is the stepper motor running (not stopped) */
		bool isRunning() {
			return (_state!=STATE_STOP);
		}

		/** To call frequently (ie: in Arduino loop) */
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

		/** Set rotation speed per minute (RPM). Driver will adjust to best suitable RPM to avoid motor damage. */
		void setSpeed(unsigned int rpm) {
			_stepTimeUs=_driver.getStepUs(rpm, _stepsPerTurn);
		}
		
		/** Set number of steps to travel 1mm (useful in combination with goMm()) */
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
		void travelMm(long mm=1, ZzzStepperCallback endActionCallback=nullptr) {
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
		ZzzStepper(int stepsPerTurn, int rpm=15, int stepsPerMm=0) {
			_state=STATE_STOP;
			_stepsPerTurn=stepsPerTurn;
			if (stepsPerMm<=0) {
				_stepsPerMm=stepsPerTurn;
			}			
			setSpeed(rpm);
		}
};

#endif

