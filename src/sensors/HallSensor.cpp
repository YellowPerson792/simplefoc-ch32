#include "HallSensor.h"


/*
  HallSensor(int hallA, int hallB , int cpr, int index)
  - hallA, hallB, hallC    - HallSensor A, B and C pins
  - pp           - pole pairs
*/
HallSensor::HallSensor(int _hallA, int _hallB, int _hallC, int _pp){

  // hardware pins
  pinA = _hallA;
  pinB = _hallB;
  pinC = _hallC;

  // hall has 6 segments per electrical revolution
  cpr = _pp * 6;

  // extern pullup as default
  pullup = Pullup::USE_EXTERN;
}

void HallSensor::attachInterruptHelper(uint32_t pin, void (*callback)(void), int mode) {
    GPIOMode_TypeDef io_mode = GPIO_Mode_IPU; // 假设所有的中断都是在输入模式下工作的
    EXTIMode_TypeDef it_mode = EXTI_Mode_Interrupt; // 假设所有的中断都是在上升沿和下降沿触发的
    EXTITrigger_TypeDef trigger_mode;

    // 根据 `mode` 参数来设置触发模式
    if (mode == CHANGE) {
        trigger_mode = EXTI_Trigger_Rising_Falling;
    } else if (mode == RISING) {
        trigger_mode = EXTI_Trigger_Rising;
    } else if (mode == FALLING) {
        trigger_mode = EXTI_Trigger_Falling;
    } else {
        return; // 无效的模式
    }

    // 调用第二个 `attachInterrupt` 函数
    attachInterrupt(pin, io_mode, callback, it_mode, trigger_mode);
}


//  HallSensor interrupt callback functions
// A channel
void HallSensor::handleA() {
  A_active= digitalRead(pinA);
  updateState();
}
// B channel
void HallSensor::handleB() {
  B_active = digitalRead(pinB);
  updateState();
}

// C channel
void HallSensor::handleC() {
  C_active = digitalRead(pinC);
  updateState();
}

/**
 * Updates the state and sector following an interrupt
 */
void HallSensor::updateState() {
  long new_pulse_timestamp = _micros();

  int8_t new_hall_state = C_active + (B_active << 1) + (A_active << 2);

  // glitch avoidance #1 - sometimes we get an interrupt but pins haven't changed
  if (new_hall_state == hall_state) {
    return;
  }
  hall_state = new_hall_state;

  int8_t new_electric_sector = ELECTRIC_SECTORS[hall_state];
  if (new_electric_sector - electric_sector > 3) {
    //underflow
    direction = Direction::CCW;
    electric_rotations += direction;
  } else if (new_electric_sector - electric_sector < (-3)) {
    //overflow
    direction = Direction::CW;
    electric_rotations += direction;
  } else {
    direction = (new_electric_sector > electric_sector)? Direction::CW : Direction::CCW;
  }
  electric_sector = new_electric_sector;

  // glitch avoidance #2 changes in direction can cause velocity spikes.  Possible improvements needed in this area
  if (direction == old_direction) {
    // not oscilating or just changed direction
    pulse_diff = new_pulse_timestamp - pulse_timestamp;
  } else {
    pulse_diff = 0;
  }

  pulse_timestamp = new_pulse_timestamp;
  total_interrupts++;
  old_direction = direction;
  if (onSectorChange != nullptr) onSectorChange(electric_sector);
}

/**
 * Optionally set a function callback to be fired when sector changes
 * void onSectorChange(int sector) {
 *  ... // for debug or call driver directly?
 * }
 * sensor.attachSectorCallback(onSectorChange);
 */
void HallSensor::attachSectorCallback(void (*_onSectorChange)(int sector)) {
  onSectorChange = _onSectorChange;
}



// Sensor update function. Safely copy volatile interrupt variables into Sensor base class state variables.
void HallSensor::update() {
  // Copy volatile variables in minimal-duration blocking section to ensure no interrupts are missed
  noInterrupts();
  angle_prev_ts = pulse_timestamp;
  long last_electric_rotations = electric_rotations;
  int8_t last_electric_sector = electric_sector;
  interrupts();
  angle_prev = ((float)((last_electric_rotations * 6 + last_electric_sector) % cpr) / (float)cpr) * _2PI ;
  full_rotations = (int32_t)((last_electric_rotations * 6 + last_electric_sector) / cpr);
}



/*
	Shaft angle calculation
  TODO: numerical precision issue here if the electrical rotation overflows the angle will be lost
*/
float HallSensor::getSensorAngle() {
  return ((float)(electric_rotations * 6 + electric_sector) / (float)cpr) * _2PI ;
}

/*
  Shaft velocity calculation
  function using mixed time and frequency measurement technique
*/
float HallSensor::getVelocity(){
  noInterrupts();
  long last_pulse_timestamp = pulse_timestamp;
  long last_pulse_diff = pulse_diff;
  interrupts();
  if (last_pulse_diff == 0 || ((long)(_micros() - last_pulse_timestamp) > last_pulse_diff*2) ) { // last velocity isn't accurate if too old
    return 0;
  } else {
    return direction * (_2PI / (float)cpr) / (last_pulse_diff / 1000000.0f);
  }

}

// HallSensor initialisation of the hardware pins 
// and calculation variables
void HallSensor::init(){
  // initialise the electrical rotations to 0
  electric_rotations = 0;

  // HallSensor - check if pullup needed for your HallSensor
  if(pullup == Pullup::USE_INTERN){
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    pinMode(pinC, INPUT_PULLUP);
  }else{
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    pinMode(pinC, INPUT);
  }

    // init hall_state
  A_active= digitalRead(pinA);
  B_active = digitalRead(pinB);
  C_active = digitalRead(pinC);
  updateState();

  pulse_timestamp = _micros();

  // we don't call Sensor::init() here because init is handled in HallSensor class.
}

// function enabling hardware interrupts for the callback provided
// if callback is not provided then the interrupt is not enabled
void HallSensor::enableInterrupts(void (*doA)(), void(*doB)(), void(*doC)()){
  // attach interrupt if functions provided

  // A, B and C callback
  if(doA != nullptr) attachInterruptHelper(digitalPinToInterrupt(pinA), doA, CHANGE);
  if(doB != nullptr) attachInterruptHelper(digitalPinToInterrupt(pinB), doB, CHANGE);
  if(doC != nullptr) attachInterruptHelper(digitalPinToInterrupt(pinC), doC, CHANGE);
}
