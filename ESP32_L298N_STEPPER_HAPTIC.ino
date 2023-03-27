//NOTES ABOUT ESP32
//GPIOs 34 to 39 can’t generate PWM

#include <SimpleFOC.h>

#define IN1_1 32 
#define IN2_1 33
#define IN3_1 25
#define IN4_1 26

#define IN1_2 16
#define IN2_2 4
#define IN3_2 0
#define IN4_2 2

MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5048_SPI, 5);
MagneticSensorSPI sensor2 = MagneticSensorSPI(AS5048_SPI, 15);

// motor instance
StepperMotor motor1 = StepperMotor(50, 4.7, 240); // StepperMotor(int pp, (optional R, KV))
StepperMotor motor2 = StepperMotor(50, 4.7, 240); // StepperMotor(int pp, (optional R, KV))

// driver instance
StepperDriver4PWM driver1 = StepperDriver4PWM(IN1_1, IN2_1, IN3_1, IN4_1); // StepperDriver4PWM(IN1, IN2, IN3, IN4, ENB, ENA)
StepperDriver4PWM driver2 = StepperDriver4PWM(IN1_2, IN2_2, IN3_2, IN4_2); // StepperDriver4PWM(IN1, IN2, IN3, IN4, ENB, ENA)

void setup() {

  // inialize magnetic sensor 
  sensor1.init();
  sensor2.init();

  // link motor to the sensor
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);
  
  // choose FOC modulation
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // power supply voltage [V]
  driver1.voltage_power_supply = 12;
  driver1.init();
  driver2.voltage_power_supply = 12;
  driver2.init();
  
  // link the motor to the sensor
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);
  
  // motor limits
  motor1.current_limit = 4.0;
  motor2.current_limit = 4.0;
  
  // set control loop to be used
  motor1.controller = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;
  
   // initialise motor
  motor1.init();
  motor2.init();
  
  // align encoder and start FOC
  motor1.initFOC();
  motor2.initFOC();

  _delay(1000);
  
}

void loop() {
  motor1.loopFOC();
  motor1.move(5*(motor2.shaft_angle - motor1.shaft_angle));
  motor2.loopFOC();
  motor2.move(5*(motor1.shaft_angle - motor2.shaft_angle));
}
