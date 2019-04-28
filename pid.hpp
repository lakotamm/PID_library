#pragma once

struct PIDSettings {
  float kP;		//P controller constant
  float kI;		//I controller constant
  float kD;		//V controller constant
  float kFF;		//FF controller constant, Airplane for control surface control, use instead of D (V)
  float outLimits;	//output limits
  float dFilter;	//IIR filter for D part, start with 0 and increase during testing 
  float accLimit;	//a separate limit for the Integral part, start with 1 and decrease during testing
};

class PID {
  private:
    //variables used for setting up the PID loop
    float _kP;
    float _kI;
    float _kD;
    float _kFF;

    float _outLimits;
    float _dFilter;
    float _accLimit;

    bool _enable;

    //internally used variables
    float _prevInput;
    float _accumulator;
    float _dBuffer;
    bool  _resetState;

  public:
    PID () :
        _kP(0),  
        _kI(0),  
        _kD(0),  
        _kFF(0), 
        _outLimits(1),
        _dFilter(0),
        _accLimit(1),
        _enable(false),
        _prevInput(0),
        _accumulator(0),
        _dBuffer(0),
        _resetState(true){}

    void init(PIDSettings pidSettings);
    void init(float kP, float kI, float kD, float kFF, float outLimits, float dFilter, float accLimit);
    PIDSettings getPIDsettings();

	//run the calculations and return new output value
	//the spin function should be called at a stable frequency
    float spin(float input, float setpoint);
    float spin(float input, float setpoint, float attenuation);

    //commands for modifying coefficients on the fly
    void setKp(float kP);
    void setKi(float kI);
    void setKd(float kD);
    void setKff(float kFF);

    //enable or disable PID
    void enable();
    void disable();
    void reset();

    //read coefficients
    float getKp();
    float getKi();
    float getKd();
    float getKff();
};
