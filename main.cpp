#include "mbed.h"

Serial pc(SERIAL_TX, SERIAL_RX);


class IR {
public:
	IR(PinName ir_out, PinName ir_receiver) : 
		_ir_out(ir_out), _ir_receiver(ir_receiver) {
			_ir_out = 0;
	}

	float read(){
		_ir_out = 1;
		//wait_ms(500);
		float reading = _ir_receiver.read();
		_ir_out = 0;
		return reading;
	}

	DigitalOut _ir_out;
	AnalogIn _ir_receiver;

};



//IR(ir_out, ir_receiver) TODO
IR IR_RIGHT(PC_10, PA_0);
IR IR_CENTER_RIGHT(PC_11, PA_4);
IR IR_CENTER_LEFT(PB_0, PC_1);
IR IR_LEFT(PB_7, PC_0);

//maybe in this order
//PC_10 PC_11 PB_0 PB_7 //ir emitter
//PA_0  PA_4  PC_1 PC_0 //ir receiver



PwmOut LeftMotorPWMF(PA_7);
PwmOut RightMotorPWMF(PB_10);

PwmOut LeftMotorPWMB(PB_6);
PwmOut RightMotorPWMB(PC_7);

int left_travelled = 0;
int right_travelled = 0;

InterruptIn right_encoder(PA_15); //right //b
InterruptIn left_encoder(PA_1); //left //g

void IE_right(){right_travelled++;}
void IE_left(){left_travelled++;}

Ticker Systicker; //to do error calculator and PID at regular intervals
Timer timer; //for dt calculations for Derivative controller

DigitalOut myled(LED1);

const int Kp = 1;
const int Kd = 1;
const int CORRECTION_PERIOD = 5; //in ms
const int Kp_ir = 650;
const int Kd_ir = 15;


int encoder_error = 0;
int prev_error = 0;
int correction = 0;
const int pwm_period = 10000;
const int base_speed = 2000; //was 1300

float base_error = 0;
float ir_error = 0;
float ir_prev_error = 0;

int P_Controller(int error) {
    int correct = Kp*error;
    return correct;    
}

int D_Controller(int error){
  int dError = error - prev_error;
  int dt = timer.read_us();
  timer.reset();
  prev_error = error;
  int loc_correction = Kd*dError/dt;
  return loc_correction;

}

int P_IR_controller(float error){
	int correct = (int)(Kp_ir*error);
	return correct;
}

int D_IR_Controller(float error){
  float dError = error - ir_prev_error;
  int dt = timer.read_us();
  timer.reset();
  ir_prev_error = error;
  int loc_correction = (((int)(Kd_ir*dError))/dt);
  return loc_correction;

}

void systick() {
  //  encoder_error = right_travelled - left_travelled;
  //  correction = P_Controller(encoder_error) + D_Controller(encoder_error);
  	ir_error = IR_LEFT.read() -IR_RIGHT.read() - base_error;
  	correction = P_IR_controller(ir_error); //+ D_IR_Controller(ir_error); //TODO: maybe change back
}

void reset_wheels(){
	LeftMotorPWMF.pulsewidth_us(0);
    RightMotorPWMF.pulsewidth_us(0);
    LeftMotorPWMB.pulsewidth_us(0);
    RightMotorPWMB.pulsewidth_us(0);
    wait_ms(100);
}

const int left_turn_time = 400;
const int left_turn_left_motor = 1200;
const int left_turn_right_motor = 1200;
void turn_left(){
	reset_wheels();
	LeftMotorPWMB.pulsewidth_us(left_turn_left_motor);
  RightMotorPWMF.pulsewidth_us(left_turn_right_motor);
  wait_ms(left_turn_time);
}

const int right_turn_time = 400;
const int right_turn_left_motor = 1200;
const int right_turn_right_motor = 1200;
void turn_right(){
	reset_wheels();
	LeftMotorPWMF.pulsewidth_us(right_turn_left_motor); //turn backward
  RightMotorPWMB.pulsewidth_us(right_turn_right_motor);
  wait_ms(right_turn_time);
}

void brake(){
//	LeftMotorPWMF.pulsewidth_us(pwm_period);
//	RightMotorPWMF.pulsewidth_us(pwm_period);
  LeftMotorPWMB.pulsewidth_us(pwm_period); 
  RightMotorPWMB.pulsewidth_us(pwm_period);

  reset_wheels();
}

void go_straight(){
//  correction = P_IR_controller(ir_error) + D_IR_Controller(ir_error); //TODO: maybe remove
//  correction = 0;
  LeftMotorPWMF.pulsewidth_us(base_speed+correction); 
  RightMotorPWMF.pulsewidth_us(base_speed-correction);
  wait_ms(CORRECTION_PERIOD); 
}

float LEFT_HAS_WALL_THRESHOLD;
float RIGHT_HAS_WALL_THRESHOLD;

int main() {
    right_encoder.rise(&IE_right);
    right_encoder.fall(&IE_right);
    left_encoder.rise(&IE_left);
    left_encoder.fall(&IE_left);
    
    wait_ms(500);
  	//IR Calibration
  	base_error = IR_LEFT.read() - IR_RIGHT.read();
  	wait_ms(100);
  	base_error = IR_LEFT.read() - IR_RIGHT.read();

  	wait_ms(100);
    LEFT_HAS_WALL_THRESHOLD = IR_LEFT.read();
    RIGHT_HAS_WALL_THRESHOLD = IR_RIGHT.read();

  	base_error = 0.5*(base_error + IR_LEFT.read() - IR_RIGHT.read());
  	base_error = 0.5*(base_error + IR_LEFT.read() - IR_RIGHT.read());
  	base_error = 0.5*(base_error + IR_LEFT.read() - IR_RIGHT.read());

    Systicker.attach_us(&systick, 500);

    pc.printf("lol");  
    
    //Motors
    LeftMotorPWMF.period_us(pwm_period);
    RightMotorPWMF.period_us(pwm_period);
    LeftMotorPWMB.period_us(pwm_period);
    RightMotorPWMB.period_us(pwm_period);

    float CENTER_LEFT_WALL_THRESHOLD = 0.005;
    float CENTER_RIGHT_WALL_THRESHOLD = 0.02;
    float error;

    LEFT_HAS_WALL_THRESHOLD = IR_LEFT.read();
    RIGHT_HAS_WALL_THRESHOLD = IR_RIGHT.read();


    pc.printf("Thresholds %.2f %.2f", LEFT_HAS_WALL_THRESHOLD, RIGHT_HAS_WALL_THRESHOLD);
    wait_ms(200);
    LEFT_HAS_WALL_THRESHOLD = 0.5*(IR_LEFT.read() + LEFT_HAS_WALL_THRESHOLD);
    RIGHT_HAS_WALL_THRESHOLD = 0.5*(IR_RIGHT.read() + RIGHT_HAS_WALL_THRESHOLD);
    pc.printf("Thresholds %.2f %.2f", LEFT_HAS_WALL_THRESHOLD, RIGHT_HAS_WALL_THRESHOLD);
    wait_ms(200);

    LEFT_HAS_WALL_THRESHOLD = 0.5*(IR_LEFT.read() + LEFT_HAS_WALL_THRESHOLD);
    RIGHT_HAS_WALL_THRESHOLD = 0.5*(IR_RIGHT.read() + RIGHT_HAS_WALL_THRESHOLD);
    pc.printf("Thresholds %.2f %.2f", LEFT_HAS_WALL_THRESHOLD, RIGHT_HAS_WALL_THRESHOLD);
    wait_ms(200);
    LEFT_HAS_WALL_THRESHOLD = 0.5*(IR_LEFT.read() + LEFT_HAS_WALL_THRESHOLD);
    RIGHT_HAS_WALL_THRESHOLD = 0.5*(IR_RIGHT.read() + RIGHT_HAS_WALL_THRESHOLD);

    LEFT_HAS_WALL_THRESHOLD -= 0.07;
    RIGHT_HAS_WALL_THRESHOLD -= 0.07;
    
    pc.printf("%.2f %.2f", LEFT_HAS_WALL_THRESHOLD, RIGHT_HAS_WALL_THRESHOLD);
    wait_ms(200);
    pc.printf("%.2f %.2f", LEFT_HAS_WALL_THRESHOLD, RIGHT_HAS_WALL_THRESHOLD);
    wait_ms(200);
    pc.printf("%.2f %.2f", LEFT_HAS_WALL_THRESHOLD, RIGHT_HAS_WALL_THRESHOLD);
    wait_ms(200);
    pc.printf("%.2f %.2f", LEFT_HAS_WALL_THRESHOLD, RIGHT_HAS_WALL_THRESHOLD);
     wait_ms(200);
    pc.printf("%.2f %.2f", LEFT_HAS_WALL_THRESHOLD, RIGHT_HAS_WALL_THRESHOLD);
    wait_ms(200);
    pc.printf("%.2f %.2f", LEFT_HAS_WALL_THRESHOLD, RIGHT_HAS_WALL_THRESHOLD);
    wait_ms(200);
    pc.printf("%.2f %.2f", LEFT_HAS_WALL_THRESHOLD, RIGHT_HAS_WALL_THRESHOLD);
    wait_ms(200);
    pc.printf("%.2f %.2f", LEFT_HAS_WALL_THRESHOLD, RIGHT_HAS_WALL_THRESHOLD);
     wait_ms(200);
    pc.printf("%.2f %.2f", LEFT_HAS_WALL_THRESHOLD, RIGHT_HAS_WALL_THRESHOLD);
  

    reset_wheels();

    while(1) {

    	if( !(IR_CENTER_LEFT.read() > CENTER_LEFT_WALL_THRESHOLD && IR_CENTER_RIGHT.read() > CENTER_RIGHT_WALL_THRESHOLD)) { //see wall
        go_straight();
    	}
      else {
      
          brake();
      //  pc.printf("LOL");
    //    brake();
        
      //  if(IR_LEFT.read() > 0.2 && IR_RIGHT.read() > 0.2){
      //   brake();
       //  wait_ms(100);
       //  continue; //deadend
       // }

        ir_error = IR_LEFT.read() -IR_RIGHT.read() - base_error;
        if(ir_error <= 0.07) {
          turn_left();
        }
        else //if(ir_error > 0) 
        {
          turn_right();
        }
      //  pc.printf("error: %.3f\r\nn", error);
      //  wait_ms(100);
      //  continue;
      //  if(error > )
        brake();

      } 
//      go_straight();

        
        

   //     LeftMotorPWMF.pulsewidth_us(0);
     //   RightMotorPWMF.pulsewidth_us(0);
    //    pc.printf("left: %.3f %.3f right: %.3f %.3f\r\n", 
     //   	IR_LEFT.read(),IR_CENTER_LEFT.read(), IR_CENTER_RIGHT.read(), IR_RIGHT.read());
 //   	pc.printf("ir error: %.3f\n", ir_error);	

    } //end while
 
  
}
