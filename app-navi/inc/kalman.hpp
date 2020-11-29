// Kalman filter hpp by Yuan

#ifndef KALMAN_H_
#define KALMAN_H_

#include "dji_vehicle.hpp"
#include "dji_telemetry.hpp"
#include "typedef.h"
#include "arm_math.h"
#include "main.h"
#include <cmath>
#include "data_logging.hpp"
#include "data_declare.hpp"

#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252

class UavControl;

class Kalman
{
public:

typedef enum
{
    ULTRA_HEADER    = 0x22,
    ULTRA_SEPARATOR = 0x00,
} ULTRA_DATA;

public:

Kalman(UavControl* uav_control = 0, Vehicle* vehicle = 0);

~Kalman();

UavControl*     uav_control;
Vehicle*        vehicle;
LocalPosition   position;

public:

// output relative position w.r.t. charging point
void kalmanOutput();

// void getRelativePositionData(LocalPosition* pos, float32_t* height);
 void getRelativePositionData(float32_t* dist, float32_t* uwb_dist, float32_t* volt_dist, float32_t* g, float32_t* h);
//void getRelativePositionData(float32_t* dist, float32_t* g, float32_t* h);

// void getUltrasonicData(uint8_t ultrasonic_byte);
void getUltrasonicData(uint16_t ultrasonic_byte);

void getVoltageData(uint32_t volt);

void setTargetGps(Telemetry::GlobalPosition g, uint8_t var);

private:

// obtain current measurements from onboard sensors
void updateOnboardMeasure();

// obtain current control inputs from uav control
void updateUavControlInput();

// update uav relative position
void positionUpdate();

// uwb position estimation
void updateGpsCoordinateFromUwb();

void updateGpsCoordinateFromVolt();

// record Kalman data
void keepRecord();

void circularLocalisationInit();

uint8_t getCircularIntersection(CircularLocalisation* cl1, CircularLocalisation* cl2, LocalPosition* lp);


private:

DataLog*                    data_log;

OnboardMeasure				om;
CircularLocalisation*       cl;

LocalPosition               updatedGps;
LocalPosition               accuUpdatedGps;

// target Gps position
Telemetry::GlobalPosition   tGps;

// ultrasonic data
// uint8_t ultrasonic;
uint16_t 					ultrasonic;
uint16_t					voltage;
};

#endif // KALMAN_H_
