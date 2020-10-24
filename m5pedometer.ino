// define must ahead #include <M5Stack.h>
#define M5STACK_MPU6886 
// #define M5STACK_MPU9250 
// #define M5STACK_MPU6050
// #define M5STACK_200Q

#include <M5Stack.h>

#include "m5pedometer.h"

struct PedometerData pd;

void init_data(PedometerData *pd){
  pd->ptrAcc = 0;
  pd->ptrVel = 0;
  for(int i=0;i<SZ_ACC_ARRAY;i++) {
    pd->accXArray[i] = 0.0f;
    pd->accYArray[i] = 0.0f;
    pd->accZArray[i] = 0.0f;
  }
  for(int i=0;i<SZ_VEL_ARRAY;i++) pd->velArray[i] = 0.0f;
  pd->oldVelEstimate = 0.0f;
  pd->timeMsCounter = 0;
  pd->stepCounter = 0;
}

void print_step(PedometerData* pd) {
  M5.Lcd.wakeup();
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextSize(3);
  M5.Lcd.printf("%d STEP      ", pd->stepCounter);

  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 170);
  for(int i=0;i<5;i++){
    delay(200);
    int lv = M5.Power.getBatteryLevel();
    if(lv != -1){
      M5.Lcd.print(lv);
      M5.Lcd.print("%(");
      M5.Power.setCharge(!M5.Power.isChargeFull());
      M5.Lcd.print(M5.Power.isCharging());
      M5.Lcd.print(")");
      
      break;
    }
  }

  M5.Lcd.print(int(millis()/1000));
  M5.Lcd.println(" secs elapsed");
  M5.Lcd.println("[A] to return");
  M5.Lcd.println("[B] to reset counter");
  M5.Lcd.println("[C] to power off");

  // break after 30sec 
  for(int i=0;i<300;i++){
    delay(100);
    M5.update();
    if(M5.BtnA.wasReleased()){
      break;
    }
    if(M5.BtnB.wasReleased()){
      pd->stepCounter = 0;
      break;
    }
    if(M5.BtnC.wasReleased()){
      M5.powerOFF();
    }
  }
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.sleep();
  return;
}

float sum(float* a, const int sz){
  float r = 0.0;
  for(int i=0;i<sz;i++){
    r += *a++;
  }
  return r;  
}

float estimate_velocity(PedometerData* pd) {
  float wz0 = sum(pd->accXArray, SZ_ACC_ARRAY) / SZ_ACC_ARRAY;
  float wz1 = sum(pd->accYArray, SZ_ACC_ARRAY) / SZ_ACC_ARRAY;
  float wz2 = sum(pd->accZArray, SZ_ACC_ARRAY) / SZ_ACC_ARRAY;
  
  float norm_factor = sqrt(wz0 * wz0 + wz1 * wz1 + wz2 * wz2);

  pd->velArray[pd->ptrVel] = (wz0 * pd->accXArray[pd->ptrAcc] +
                              wz1 * pd->accYArray[pd->ptrAcc] +
                              wz2 * pd->accZArray[pd->ptrAcc] ) / norm_factor - norm_factor;

  if(++pd->ptrVel==SZ_VEL_ARRAY) pd->ptrVel=0;
  if(++pd->ptrAcc==SZ_ACC_ARRAY) pd->ptrAcc=0;

  return sum(pd->velArray, SZ_VEL_ARRAY);
}

void setup(){
  M5.begin();
  M5.Power.begin();
  M5.IMU.Init();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(2);

  init_data(&pd);
  print_step(&pd);
}

void loop() {

  M5.IMU.getAccelData(&pd.accXArray[pd.ptrAcc], &pd.accYArray[pd.ptrAcc], &pd.accYArray[pd.ptrAcc]);

  float velEstimate = estimate_velocity(&pd);

  if(velEstimate > STEP_THRESHOLD &&
     pd.oldVelEstimate <= STEP_THRESHOLD &&
     (pd.timeMsCounter > STEP_DELAY_MS)){
      pd.stepCounter ++;
      pd.timeMsCounter = 0;
  }

  M5.Power.lightSleep(PERIODIC_MS * 1000); // alternative for delay()
  pd.oldVelEstimate = velEstimate;
  pd.timeMsCounter += PERIODIC_MS;

  M5.update();
  if(M5.BtnA.wasReleased()){
    print_step(&pd);
  }
}
