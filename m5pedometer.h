#ifndef __M5PEDOMETER_H__
#define __M5PEDOMETER_H__

#define SZ_ACC_ARRAY 50
#define SZ_VEL_ARRAY 10
#define STEP_THRESHOLD 2.5f
#define STEP_DELAY_MS 250

#define PERIODIC_MS 10

struct PedometerData {
    int ptrAcc;
    int ptrVel;
    float accXArray[SZ_ACC_ARRAY];
    float accYArray[SZ_ACC_ARRAY];
    float accZArray[SZ_ACC_ARRAY];
    float velArray[SZ_VEL_ARRAY];
    float oldVelEstimate;
    int timeMsCounter;
    int stepCounter;
};

#endif // __M5PEDOMETER_H__
