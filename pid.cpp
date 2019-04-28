#include "pid.hpp"

#define PID_SCALE 100

void PID::init(PIDSettings pidSettings){
  setKp(pidSettings.kP);
  setKi(pidSettings.kI);
  setKd(pidSettings.kD);
  setKff(pidSettings.kFF);
  _outLimits = pidSettings.outLimits;
  _dFilter = pidSettings.dFilter;
  _accLimit = pidSettings.accLimit * PID_SCALE;
  reset(); //reset, in case the PID has been initialized before
}

void PID::init(float kP, float kI, float kD, float kFF, float outLimits,
               float dFilter, float accLimit){
  setKp(kP);
  setKi(kI);
  setKd(kD);
  setKff(kFF);
  _outLimits = outLimits;
  _dFilter = dFilter;
  accLimit = accLimit * PID_SCALE;
  reset(); //reset, in case the PID has been initialized before
}

PIDSettings PID::getPIDsettings(){
  PIDSettings pidSettings;
  pidSettings.kP = getKp();
  pidSettings.kI = getKi();
  pidSettings.kD = getKd();
  pidSettings.kFF = getKff();
  pidSettings.outLimits = _outLimits;
  pidSettings.dFilter = _dFilter;
  pidSettings.accLimit = _accLimit;
  return pidSettings;
}

float PID::spin(float input, float setpoint){
  return spin(input, setpoint, 1);
}

float PID::spin(float input, float setpoint, float attenuation){

  float output = 0;
  float pControl = 0;
  float iControl = 0;
  float dControl = 0;
  float ffControl = 0;

  if(_enable){

    if(_resetState){
      _prevInput = input;
      _resetState = false;
    }

    float error = setpoint - input;

    //P
    pControl = error * _kP * attenuation;

    //I
    // accumulator + accumulator limiting
    if(_kI != 0){

      _accumulator += error;
      if(_accumulator *_kI * attenuation > _outLimits * _accLimit){
        _accumulator = _outLimits * _accLimit/(_kI * attenuation);
      }
      else if (_accumulator *_kI * attenuation < (-_outLimits * _accLimit)){
        _accumulator = -(_outLimits * _accLimit/(_kI * attenuation));
      }

      iControl = _accumulator *_kI * attenuation;

    } else {

      _accumulator = 0;
      iControl = 0;

    }

    //D
    //low passing the velocity part
    if(_kD != 0){

    _dBuffer = _dBuffer + (((input - _prevInput) - _dBuffer)/(_dFilter+1));

    dControl = _dBuffer * _kD * attenuation;

    } else {
      _dBuffer = 0;
      dControl = 0;
    }

    //FF
    //feed forward control

    ffControl = setpoint * _kFF * attenuation;

    //sum of all 4 control loops
    output = (pControl + iControl + dControl + ffControl)/PID_SCALE;

    _prevInput = input;

    //output limiting
    if (output > _outLimits){
      output = _outLimits;
    }
    else if (output < -(_outLimits)){
      output = -(_outLimits);
    }

  } else {
    _accumulator = 0;
    _dBuffer = 0;
    output = 0;
  }

  return output;
}

void PID::setKp(float kP){
  _kP = kP;
}
void PID::setKi(float kI){
  _kI = kI;
}
void PID::setKd(float kD){
  _kD = kD;
}
void PID::setKff(float kFF){
  _kFF = kFF;
}

void PID::enable(){
  _enable = true;
}
void PID::disable(){
  _enable = false;
  reset();
}

void PID::reset(){
  _prevInput = 0;
  _accumulator = 0;
  _dBuffer = 0;
  _resetState = true;
}

float PID::getKp(){
  return _kP;
}
float PID::getKi(){
  return _kI;
}
float PID::getKd(){
  return _kD;
}
float PID::getKff(){
  return _kFF;
}
