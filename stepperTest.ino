#include <AccelStepper.h>
#include <MultiStepper.h>


#define FreezerGuide_DIR_PIN 40
#define FreezerGuide_STEP_PIN 42
#define ReedSensorFreezerGuidePin 22
/********************************************************************************/

#define BlenderArm_DIR_PIN 46
#define BlenderArm_STEP_PIN 48
#define ReedSensorBlenderArmPin 50

/********************************************************************************/

#define BlenderGuide_DIR_PIN 35
#define BlenderGuide_STEP_PIN 33
#define ReedSensorBlenderGuidePin 12

/********************************************************************************/

#define Blender_DIR_PIN 47
#define Blender_STEP_PIN 49
#define ReedSensorBlenderPin 10




AccelStepper FreezerGuide(AccelStepper::DRIVER,  FreezerGuide_STEP_PIN, FreezerGuide_DIR_PIN);
AccelStepper BlenderArm(AccelStepper::DRIVER,  BlenderArm_STEP_PIN, BlenderArm_DIR_PIN);

AccelStepper BlenderGuide(AccelStepper::DRIVER,  BlenderGuide_STEP_PIN, BlenderGuide_STEP_PIN);
AccelStepper Blender(AccelStepper::DRIVER,  Blender_STEP_PIN, Blender_DIR_PIN);



MultiStepper steppers;

// state of reedsensor
int FreezerGuideState; 
int BlenderArmState;
int BlenderGuideState;
int BlenderState;



int state1=0; //state of stepper 1 freezerguide
int state2=0; //state of stepper 2 blenderarm
int state3=0; //state of stepper 3 blenderguide
int state4=0; // state stepper 4 blender


void setup() {
  // put your setup code here, to run once:
    //Serial setup + debugledledSetup

    Serial.begin(9600);
    pinMode(12,OUTPUT);
    digitalWrite(12, LOW);


    
   /********************************************************************************/

    //Stepper setup 

     FreezerGuide.setMaxSpeed(5000);
     FreezerGuide.setSpeed(300.0);
     //FreezerGuide.moveTo(100000);
            
 
    BlenderArm.setMaxSpeed(5000);
    BlenderArm.setSpeed(200);

    BlenderGuide.setMaxSpeed(5000);
    BlenderGuide.setSpeed(300);
//
    Blender.setMaxSpeed(5000);
    Blender.setSpeed(300);
 

     // if they have to move exactely at hte same time and speed use steppers and not the stepper
    steppers.addStepper(FreezerGuide);
    steppers.addStepper(BlenderArm);
    steppers.addStepper(BlenderGuide);
    steppers.addStepper(Blender);

    /********************************************************************************/
    // switch setup 
    pinMode(ReedSensorFreezerGuidePin, INPUT_PULLUP);
    pinMode(ReedSensorBlenderArmPin, INPUT_PULLUP);
    pinMode(ReedSensorBlenderGuidePin, INPUT_PULLUP);
    pinMode(ReedSensorBlenderPin, INPUT_PULLUP);
    
}

void loop() {
  // put your main code here, to run repeatedly:
  
    /********************************************************************************/
    homing(FreezerGuide, BlenderArm, Blender , BlenderGuide,steppers);
    /********************************************************************************/    
}




void homing(AccelStepper stepper1, AccelStepper stepper2, AccelStepper stepper3,AccelStepper stepper4, MultiStepper steppers11)// AccelStepper stepper3, AccelStepper stepper4
{
     
    
    FreezerGuideState=digitalRead(ReedSensorFreezerGuidePin);
    BlenderArmState =digitalRead(ReedSensorBlenderArmPin);
    BlenderState=digitalRead(12);
    //delay(1);
   
 
   
      //
    // Serial.println(BlenderState);
    
      homingmotor1(stepper1, ReedSensorFreezerGuidePin, steppers11 , &state1, &state2, &state3,&state4);
      homingmotor1(stepper2, ReedSensorBlenderArmPin, steppers11, &state2, &state1,&state3,&state4);
      homingmotor1(stepper3, ReedSensorBlenderPin, steppers11 , &state3, &state2, &state1,&state4);
      homingmotor1(stepper4, ReedSensorBlenderGuidePin, steppers11 , &state4, &state2, &state1, &state3);
     /********************************************************************************/
}

void homingmotor1(AccelStepper stepper1, int homepin, MultiStepper steppers, int *state, int *state2, int *state3, int *state4)
{
   
   int homingstate=digitalRead(homepin); //read homing sensor
   
   if (*state == 0)
      {
           if (homingstate == 0)
           {
           *state = 1;
           }
           else{
         
           stepper1.run();
           }
      }

    else if (*state == 1 &&  *state2 == 1 && *state3 == 1 && *state4 ==1 ) // if all the motors ares at the reedswitch then go back 
       //digitalWrite(12,HIGH);

       FreezerGuide.setMaxSpeed(500);
       BlenderArm.setMaxSpeed(500);
       Blender.setMaxSpeed(500);
       BlenderGuide.setMaxSpeed(500);
       
       
       long positions[4];
       positions[0] = -1500;
       positions[1] = -1500;
       positions[3]=  -1500;
       positions[2] = -1500;
       
       steppers.moveTo(positions);
       steppers.runSpeedToPosition();
          if (stepper1.distanceToGo() == 0)
             {
           *state = 2;
            }
      }
   else if (*state == 2)
      {
          if(homingstate == 1)
          {
            stepper1.setMaxSpeed(200);
            stepper1.setSpeed(100);
            stepper1.run();
          }
          else 
          {
            *state =3;
          }
      }
   else if (*state ==3)
      {
        stepper1.stop();
        *state = 4;
      }
     return *state;
}


