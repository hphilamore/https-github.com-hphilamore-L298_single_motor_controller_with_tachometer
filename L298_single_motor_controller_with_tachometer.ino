 /******************************************************************************
Code based on:
L298 motor driver:
https://tronixlabs.com.au/news/tutorial-l298n-dual-motor-controller-module-2a-and-arduino/
Optical tachometer:
https://learn.sparkfun.com/tutorials/qrd1114-optical-detector-hookup-guide?_ga=1.57060067.834043405.1459523277
www.instructables.com/id/Arduino-Based-Optical-Tachometer/
Arduino Pins
10 --> L298, pin ENA
9 --> L298, pin IN1
8 --> L298, pin IN2
2 --> Photodetector Collector pin
******************************************************************************/
float EnRes = 12;

// the time (in mS) increment to record the encoder output for before outputting to serial 
int TachoIncrement = 2000;

const float pi = 3.142;

// connect motor controller pins to Arduino digital pins
// motor one
int enA = 10;
int in1 = 9;
int in2 = 8;

unsigned long count;
unsigned long time;
unsigned long timeold;

float timer;    
float rps;
float radps;

int PWM_signal;
int PWM_min;
int PWM_max;

  void setup()
   {   
     Serial.begin(9600);
     //Interrupt 0 is digital pin 2, so that is where the IR detector is connected
     //Triggers on FALLING (change from HIGH to LOW)
     attachInterrupt(0, rps_fun, FALLING);
  
     count = 0;
     timeold = 0;
     
     // set all the motor control pins to outputs
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    
   Serial.print("PWM");
   Serial.print("\t");
   Serial.print("rps");
   Serial.print("\t");
   Serial.println("radps");
 }  
  
 void loop()
 { 
  demoOne(50, "FWD");
  delay(1000);
  //demoTwo(0, 255, "FWD"); 
  //delay(3000);
 }

void rps_fun()
 {
   count +=1;      
 }
 
 void tachometer(int PWM_val)
 {
   // reset the counter and timer in case the speed has changed since the last count
   timeold = millis();
   count = 0;   
   for (int j = 0; j < 5; j++)    
   {
      // the delay determines the total recording time
      delay(TachoIncrement);    
      
      // don't process interrupts during calculations
      detachInterrupt(0);  
    
      // calculate the rotational speed  
      time = millis();
      timer = float(time - timeold);
      rps = 1000*count/(EnRes * timer); 
      radps = 2*pi*1000*count/(EnRes * timer); 
      
      // reset the counter and timer to begin the next increment
      timeold = millis();   
      count = 0;   
      
      // print everything
      Serial.print(PWM_val);
      Serial.print("\t");
      Serial.print(rps);
      Serial.print("\t");
      Serial.println(radps);
      
      //Restart the interrupt processing
      attachInterrupt(0, rps_fun, FALLING);
      
    }

}
 
 
void demoOne(int PWM_signal, String motor_direction)  
{
// run the motors at a fixed speed
// PWM_signal sets speed out of possible range 0~255
// motor_direction takes "FWD" or "REV" to determine direction

// turn on motor
  if(motor_direction == "FWD")
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  analogWrite(enA, PWM_signal);  
//  tachometer(PWM_signal);

}

void demoTwo(int PWM_min, int PWM_max, String motor_direction)
{
// this function will run the motors across the range of possible speeds
// note that maximum speed is determined by the motor itself and the operating voltage
// the PWM values sent by analogWrite() are fractions of the maximum speed possible
// by your hardware

// turn on motor
  if(motor_direction == "FWD")
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  
  else
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  
  // accelerate from zero to maximum speed
  for (int i = PWM_min; i < PWM_max; i = i + 5)  
  {
    analogWrite(enA, i);
    //tachometer(i);
  }
  // decelerate from maximum speed to zero
  for (int i = PWM_max; i >= PWM_min; i = i - 5)
  {
    analogWrite(enA, i);  
    //tachometer(i);
  }  
  }
  

void demoThree()
{
  // now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

