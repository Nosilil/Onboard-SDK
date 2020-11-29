// uav control cpp by Yuan

#include "uav_control.hpp"
#include "user_diskio.h"
#include <new>
#include <math.h>

// using namespace DJI::OSDK;
extern uint8_t uav_time_tick;

UavControl::UavControl()
    : vehicle(NULL)
    , kalman(NULL)
{
    vehicle = new Vehicle(false);

    kalman = new Kalman(this, vehicle);

    velocity.x = 0;
    velocity.y = 0;
    velocity.z = 0;
    velocity.yaw = 0;

    pid.p = 0;
    pid.i = 0;
    pid.d = 0;

    cruise_altitude = DEFAULT_ALTITUDE;
}

UavControl::~UavControl()
{
    delete(vehicle);
    delete(kalman);
}

void UavControl::onboardProcess()
{   
    if(uav_time_tick == 1)
    {
        uav_time_tick = 0;
        moveByVelocity();
    }
}

void UavControl::moveByVelocity()
{
   float32_t distance;
   float32_t uwb_dist;
   float32_t volt_dist;
   float32_t gamma;
   float32_t height;
   float32_t cmd_v;
   float32_t cmd_vZ;

   kalman->kalmanOutput();
   kalman->getRelativePositionData(&distance, &uwb_dist, &volt_dist, &gamma, &height);
//   kalman->getRelativePositionData(&distance, &gamma, &height);

   if(distance > DESCENDING_DISTANCE)
	   cruise_altitude = DEFAULT_ALTITUDE;
//   else if(distance <= DESCENDING_DISTANCE && (uwb_dist < 0.3 && uwb_dist > 0) &&
//		   sqrt(pow(velocity.x, 2) + pow(velocity.y, 2)) < 0.3)
   else if(distance <= DESCENDING_DISTANCE && distance < 0.3)
   {
//	   if(volt_dist < 0.3)
		   uavLanding();
   }
   else
	   cruise_altitude = 1;
	//    cruise_altitude = -1;

   pidValueUpdate(distance, height);

   cmd_v = PROPORTIONAL_FACTOR * pid.p + INTEGRATION_FACTOR * pid.i + DERIVATION_FACTOR * pid.d;

   cmd_v = (cmd_v > CMD_VELOCITY_XY) ? CMD_VELOCITY_XY : cmd_v;

   velocity.x = cmd_v * cos(gamma);
   velocity.y = cmd_v * sin(gamma);

   cmd_vZ = PROPORTIONAL_FACTOR_Z * pid.pZ;

   if(cmd_vZ > 0)
	   velocity.z = (cmd_vZ > CMD_VELOCITY_Z) ? CMD_VELOCITY_Z : cmd_vZ;
   else
	   velocity.z = (abs(cmd_vZ) > CMD_VELOCITY_Z) ? -1 * CMD_VELOCITY_Z : cmd_vZ;

   velocity.yaw = 0;

   vehicle->control->velocityAndYawRateCtrl(velocity.x, velocity.y, velocity.z, velocity.yaw);
}

void UavControl::pidValueUpdate(float32_t distance, float32_t height)
{
	static float32_t dist[I_LENGTH] = {0};
	static uint8_t dist_count = 0;

	pid.p = distance;

	pid.i = pid.i - dist[dist_count] + distance;
	dist[dist_count] = distance;

	if(dist_count - D_LENGTH < 0) {
		pid.d = distance - dist[dist_count + I_LENGTH - D_LENGTH];
	} else {
		pid.d = distance - dist[dist_count - D_LENGTH];
	}

	pid.d = pid.d * CTRL_FREQ / D_LENGTH;

	++dist_count %= I_LENGTH;

	pid.pZ = cruise_altitude - height;
}

void UavControl::uavInit()
{
    vehicle->functionalSetUp();
    HAL_Delay(500);

    Vehicle::ActivateData user_act_data;
	char key_buf[65] = "2c21fe7d9a4d9da7287fadc7a38e41003c623c30ae1aae3b5612bbfd68977d62";
	user_act_data.ID = 1033794;
	user_act_data.encKey = key_buf;

    vehicle->activate(&user_act_data);
    HAL_Delay(500);

    if (vehicle->getFwVersion() != Version::M100_31)
	{
		printf("unable to activate\r\n");
		return;
	}

    updateGpsAltitude();

    vehicle->obtainCtrlAuthority();
    HAL_Delay(1000);
}

bool UavControl::uavTakeOff()
{
    vehicle->control->takeoff();
    HAL_Delay(500);

    uint32_t CONTROL_TIMEOUT = 15000; // milliseconds
    uint32_t RETRY_TICK      = 500;   // milliseconds
    uint32_t nextRetryTick   = 0;     // millisesonds
    uint32_t timeoutTick;
    bool     isTakeOffState = false;

    timeoutTick = vehicle->protocolLayer->getDriver()->getTimeStamp() + CONTROL_TIMEOUT;
    do
    {
	    HAL_Delay(3000);

        if (vehicle->broadcast->getStatus().flight == VehicleStatus::M100FlightStatus::IN_AIR_STANDBY)
        {
            isTakeOffState = true;
            break;
        }

        nextRetryTick = vehicle->protocolLayer->getDriver()->getTimeStamp() + RETRY_TICK;
    } while (nextRetryTick < timeoutTick);

    if (!isTakeOffState)
    {
        printf("Takeoff failed. Motors failed to start!\n");
        return false;
    }
    else
        printf("\nSuccessful takeoff!\n");

    HAL_Delay(3000);
    Telemetry::GlobalPosition currentHeight = vehicle->broadcast->getGlobalPosition();
    printf("Aircraft hovering at %d m! \n", currentHeight.altitude);

    return true;
}

bool UavControl::uavLanding()
{
  uint32_t       SUBSCRIBE_TIMOUT = 30000; // milliseconds
  uint32_t       RETRY_TICK       = 500;   // milliseconds
  uint32_t       nextRetryTick    = 0;     // millisesonds
  uint32_t       timeoutTick;
  bool           isLandingState         = false;
  bool           isFinishedLandingState = false;

  // Start landing
  vehicle->control->land();
  HAL_Delay(500);

  timeoutTick = vehicle->protocolLayer->getDriver()->getTimeStamp() + SUBSCRIBE_TIMOUT;
  do
  {
    //! Two seconds delay
    HAL_Delay(2000);

    if (vehicle->broadcast->getStatus().flight != VehicleStatus::M100FlightStatus::LANDING)
    {
      isLandingState = true;
      break;
    }

    nextRetryTick = vehicle->protocolLayer->getDriver()->getTimeStamp() + RETRY_TICK;
  } while (nextRetryTick < timeoutTick);

  if (!isLandingState)
  {
    printf("Landing failed. Aircraft is still in the air.\n");
    return false;
  }
  else
    printf("Vehicle landing...\n");


  // Second check: Finished landing
  timeoutTick = vehicle->protocolLayer->getDriver()->getTimeStamp() + SUBSCRIBE_TIMOUT;
  do
  {
    //! Two seconds delay
    HAL_Delay(2000);

    if(vehicle->broadcast->getStatus().flight == VehicleStatus::M100FlightStatus::FINISHING_LANDING)
        HAL_Delay(100);
    else
    {
        Telemetry::GlobalPosition gp;
        do
        {
            HAL_Delay(2000);
            gp = vehicle->broadcast->getGlobalPosition();
        } while (gp.altitude != 0);

        if(gp.altitude == 0)
            isFinishedLandingState = true;
    }
    
        nextRetryTick = vehicle->protocolLayer->getDriver()->getTimeStamp() + RETRY_TICK;
    } while (nextRetryTick < timeoutTick);

    if (isFinishedLandingState)
        printf("Successful landing!\n");
    else
    {
        printf("Landing finished, but the aircraft is in an unexpected mode. "
            "Please connect DJI GO.\n");
        return false;
    }

  return true;
}

void UavControl::updateGpsAltitude()
{
    float32_t gpsAlti;
    Telemetry::GlobalPosition g;
    for(int i = 0; i < 50; i++)
    {
        g = vehicle->broadcast->getGlobalPosition();
        gpsAlti += g.altitude / 50;
        HAL_Delay(50);
    }

    kalman->setTargetGps(g, 2);
}
