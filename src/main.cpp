
#ifdef TEST
#include <Arduino.h>
#include <vector>
#include "TeensyStep.h"

using std::vector;

#define ENABLE_PIN 8

Stepper stepper_x(2, 5);       // STEP pin: 2, DIR pin: 3
Stepper stepper_y(3, 6);
StepControl controller;    // Use default settings


class CPoint{
    public:
        CPoint(int x, int y): x(x), y(y) {}
        int x = 0;
        int y = 0;
};

vector<CPoint*> points;

void setup(){
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(ENABLE_PIN, LOW);

    points.push_back(new CPoint(500, 500));
   /* points.push_back(new CPoint(200, 200));
    points.push_back(new CPoint(300, 300));
    points.push_back(new CPoint(400, 400));
    points.push_back(new CPoint(500, 500));
    points.push_back(new CPoint(600, 600));
    points.push_back(new CPoint(700, 700));
    points.push_back(new CPoint(800, 800));
    points.push_back(new CPoint(900, 900));
    points.push_back(new CPoint(1000, 1000));*/

    #ifdef TEENSY3
    Serial.println("TEENSY3");
    #else
    Serial.println("STM32F4");
    #endif

    stepper_x.setAcceleration(200)
              .setMaxSpeed(200)
              .setPullInOutSpeed(0, 0);

    stepper_y.setAcceleration(300)
             .setMaxSpeed(500)
             .setPullInOutSpeed(0, 0);
}

void loop(){
    delay(2000);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Starting new sequence");

    //for(auto& point : points){
    for(unsigned i = 0; i < points.size(); i++){
       /* if(i == 0){
            stepper_x.setPullInOutSpeed(0, 500);
            stepper_y.setPullInOutSpeed(0, 500);
        }else if(i == points.size() -1){
            stepper_x.setPullInOutSpeed(500, 0);
            stepper_y.setPullInOutSpeed(500, 0);
        }
        else{
            stepper_x.setPullInSpeed(500);
            stepper_y.setPullInSpeed(500);
        }*/
      //  Serial.printf("Moving to x: %d, y: %d\r\n", points.at(i)->x, points.at(i)->y);
        stepper_x.setTargetRel(points.at(i)->x);
       // stepper_y.setTargetAbs(points.at(i)->y);
        controller.moveAsync(stepper_x);//, stepper_y);
        while(controller.isRunning());
    }

    //controller.step_calls  = controller.pulse_calls = controller.acc_calls = 0;
    //Serial.printf("steps: %u, pulses: %u, acc: %u\r\n", controller.step_calls, controller.pulse_calls, controller.acc_calls);
    digitalWrite(LED_BUILTIN, LOW);
}

#endif
