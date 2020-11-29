// by Yuan
// high level uav control class

#ifndef UAV_CONTROL_H_
#define UAV_CONTROL_H_

#include "dji_vehicle.hpp"
#include "data_declare.hpp"
#include "kalman.hpp"
#include "tim.h"
#include "main.h"

#define PROPORTIONAL_FACTOR 0.2
#define INTEGRATION_FACTOR 0.05
#define DERIVATION_FACTOR 0.05
#define PROPORTIONAL_FACTOR_Z 0.3

class UavControl
{
public:

typedef enum 
{
    CMD_VELOCITY_XY = 2,

    CMD_VELOCITY_Z = 2,

    CMD_VELOCITY_YAW = 0,

} CMD_VELOCITY;

typedef enum
{
	I_LENGTH = 8,

	D_LENGTH = 4,

} PID_FACTOR;

typedef enum
{
	DEFAULT_ALTITUDE = 10,

	DESCENDING_DISTANCE = 20,

} CTRL_FACTOR;

#pragma pack(1)

	typedef struct PIDnum
	{
		float32_t p;
		float32_t i;
		float32_t d;
		float32_t pZ;
	} PIDnum;

#pragma pack(1)

public:

UavControl();
~UavControl();

public:

Vehicle*    	vehicle;
Kalman*     	kalman;

LocalVelocity 	velocity;

public:

void uavInit();
bool uavTakeOff();
bool uavLanding();

void onboardProcess();
void moveByVelocity();

void updateGpsAltitude();

private:

void pidValueUpdate(float32_t distance, float32_t height);

private:

PIDnum pid;
float32_t cruise_altitude;

};


#endif // UAV_CONTROL_H_
