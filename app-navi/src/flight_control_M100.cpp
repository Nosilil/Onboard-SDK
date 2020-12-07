/*! @file FlightControlSample.cpp
 *  @version 3.3
 *  @date May 2017
 *
 *  @brief
 *  Flight control STM32 example.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "flight_control.hpp"
#include "stdio.h"
#include "dji_telemetry.hpp"
// is studio.h meant to be local for compiling purpose?

extern Vehicle  vehicle;
extern Vehicle* v;

bool
monitoredTakeOff()
{
  //@todo: remove this once the getErrorCode function signature changes
  char           func[50];
  ACK::ErrorCode ack;
  int            pkgIndex;


  // Start takeoff
  v->control->takeoff();
  HAL_Delay(500);

  // First check: Motors started
  uint32_t CONTROL_TIMEOUT = 15000; // milliseconds
  uint32_t RETRY_TICK      = 500;   // milliseconds
  uint32_t nextRetryTick   = 0;     // millisesonds
  uint32_t timeoutTick;
  bool     isTakeOffState = false;
  bool     isInAirState   = false;
  bool     isHoverState   = false;

  // timeoutTick = v->protocolLayer->getDriver()->getTimeStamp() + CONTROL_TIMEOUT;

  // protocolLayer is no longer in place in V4.0.1, need to find an alternative to do the timestamping 
  // attempt to use the broadcast and subscribe function to use the GPS signal as timestamp
  timeoutTick = v->subscribe->
  
  do
  {
    //! Two seconds delay
	  HAL_Delay(2000);

    if (v->broadcast->getStatus().flight ==
      DJI::OSDK::VehicleStatus::M100FlightStatus::TAKEOFF ||
      v->broadcast->getStatus().flight ==
      DJI::OSDK::VehicleStatus::M100FlightStatus::IN_AIR_STANDBY
      )
    {
      isTakeOffState = true;
      break;
    }

    nextRetryTick = v->protocolLayer->getDriver()->getTimeStamp() + RETRY_TICK;
  } while (nextRetryTick < timeoutTick);

  if (!isTakeOffState)
  {
    printf("Takeoff failed. Motors failed to start!\n");

    return false;
  }
  else
  {
    printf("\nSuccessful takeoff!\n");
  }

  timeoutTick = v->protocolLayer->getDriver()->getTimeStamp() + CONTROL_TIMEOUT;
  do
  {
    //! Two seconds delay
	  HAL_Delay(2000);

    if (v->broadcast->getStatus().flight ==
      DJI::OSDK::VehicleStatus::M100FlightStatus::IN_AIR_STANDBY)
    {
      isInAirState = true;
      break;
    }

    nextRetryTick = v->protocolLayer->getDriver()->getTimeStamp() + RETRY_TICK;
  } while (nextRetryTick < timeoutTick);

  if (!isInAirState)
  {
    printf("Takeoff failed. Aircraft is still on the ground, but the "
           "motors are spinning.\n");

    return false;
  }
  else
  {
    printf("Vehicle ascending...\n");
  }

  timeoutTick = v->protocolLayer->getDriver()->getTimeStamp() + CONTROL_TIMEOUT;
  
  float32_t delta;
  Telemetry::GlobalPosition currentHeight;
  Telemetry::GlobalPosition deltaHeight = v->broadcast->getGlobalPosition();

  do
  {
	HAL_Delay(3000);
    currentHeight = v->broadcast->getGlobalPosition();
    delta         = fabs(currentHeight.altitude - deltaHeight.altitude);
    deltaHeight.altitude   = currentHeight.altitude;
  } while (delta >= 0.009);

  printf("Aircraft hovering at %d m! \n", currentHeight.altitude);

  return true;
}

/*! Monitored Takeoff (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/
bool
monitoredLanding()
{
  //@todo: remove this once the getErrorCode function signature changes
  char           func[50];
  ACK::ErrorCode ack;
  uint32_t       SUBSCRIBE_TIMOUT = 30000; // milliseconds
  uint32_t       RETRY_TICK       = 500;   // milliseconds
  uint32_t       nextRetryTick    = 0;     // millisesonds
  uint32_t       timeoutTick;
  bool           isLandingState         = false;
  bool           isFinishedLandingState = false;
  bool           isHoverState           = false;

  // Telemetry: Subscribe to flight status and mode at freq 10 Hz
  int pkgIndex = 0;
  int freq     = 10;

  // Start landing
  v->control->land();
  HAL_Delay(500);
  /*ack = waitForACK();
  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, func);
    return false;
  }*/

  // First check: Landing started

  timeoutTick =
    v->protocolLayer->getDriver()->getTimeStamp() + SUBSCRIBE_TIMOUT;
  do
  {
    //! Two seconds delay
    HAL_Delay(2000);

    if (v->broadcast->getStatus().flight !=
        DJI::OSDK::VehicleStatus::M100FlightStatus::LANDING)
    {
      isLandingState = true;
      break;
    }

    nextRetryTick = v->protocolLayer->getDriver()->getTimeStamp() + RETRY_TICK;
  } while (nextRetryTick < timeoutTick);

  if (!isLandingState)
  {
    printf("Landing failed. Aircraft is still in the air.\n");
    // Cleanup before return
    if (v->getFwVersion() != Version::M100_31)
    {
      v->subscribe->removePackage(pkgIndex);
      HAL_Delay(500);
    }
    return false;
  }
  else
  {
    printf("Vehicle landing...\n");
  }

  // Second check: Finished landing

  timeoutTick =
    v->protocolLayer->getDriver()->getTimeStamp() + SUBSCRIBE_TIMOUT;
  do
  {
    //! Two seconds delay
    HAL_Delay(2000);

    if(v->broadcast->getStatus().flight ==
          DJI::OSDK::VehicleStatus::M100FlightStatus::FINISHING_LANDING)
    {
      HAL_Delay(100);
    }
    else
    {
      Telemetry::GlobalPosition gp;
      do
      {
        HAL_Delay(2000);
        gp = v->broadcast->getGlobalPosition();
      } while (gp.altitude != 0);

      if(gp.altitude == 0)
      {
        isFinishedLandingState = true;
      }
    }
    
    nextRetryTick = v->protocolLayer->getDriver()->getTimeStamp() + RETRY_TICK;
  } while (nextRetryTick < timeoutTick);

  if (isFinishedLandingState)
  {
    printf("Successful landing!\n");
  }
  else
  {
    printf("Landing finished, but the aircraft is in an unexpected mode. "
           "Please connect DJI GO.\n");
    return false;
  }

  return true;
}

/*! Position Control. Allows you to set an offset from your current
    location. The aircraft will move to that position and stay there.
    Typical use would be as a building block in an outer loop that does not
    require many fast changes, perhaps a few-waypoint trajectory. For smoother
    transition and response you should convert your trajectory to attitude
    setpoints and use attitude control or convert to velocity setpoints
    and use velocity control.
!*/
int
moveByPositionOffset(float xOffsetDesired, float yOffsetDesired,
                     float zOffsetDesired, float yawDesired,
                     float posThresholdInM, float yawThresholdInDeg)
{
  // Set timeout: this timeout is the time you allow the drone to take to finish
  // the
  // mission
  int timeoutInMilSec              = 100000;
  int controlFreqInHz              = 50; // Hz
  int cycleTimeInMs                = 1000 / controlFreqInHz;
  int outOfControlBoundsTimeLimit  = 10 * cycleTimeInMs; // 10 cycles
  int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs; // 50 cycles

  //@todo: remove this once the getErrorCode function signature changes
  char           func[50];
  ACK::ErrorCode ack;

  // Telemetry: Subscribe to quaternion, fused lat/lon and altitude at freq 50
  // Hz
  int pkgIndex = 0;
  int freq     = 50;

  // Get data

  // Global position retrieved via broadcast
  Telemetry::GlobalPosition currentBroadcastGP;
  Telemetry::GlobalPosition originBroadcastGP;

  // Convert position offset from first position to local coordinates
  Telemetry::Vector3f localOffset;

  currentBroadcastGP = v->broadcast->getGlobalPosition();
  originBroadcastGP  = currentBroadcastGP;
  localOffsetFromGpsOffset(v, localOffset,
                            static_cast<void*>(&currentBroadcastGP),
                            static_cast<void*>(&originBroadcastGP));

  // Get initial offset. We will update this in a loop later.
  double xOffsetRemaining = xOffsetDesired - localOffset.x;
  double yOffsetRemaining = yOffsetDesired - localOffset.y;
  double zOffsetRemaining = zOffsetDesired - (-localOffset.z);

  // Conversions
  double yawDesiredRad     = DEG2RAD * yawDesired;
  double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;

  //! Get Euler angle

  // Quaternion retrieved via broadcast
  Telemetry::Quaternion broadcastQ;
  double                yawInRad;

  broadcastQ = v->broadcast->getQuaternion();
  yawInRad   = toEulerAngle((static_cast<void*>(&broadcastQ))).z / DEG2RAD;

  int   elapsedTimeInMs     = 0;
  int   withinBoundsCounter = 0;
  int   outOfBounds         = 0;
  int   brakeCounter        = 0;
  int   speedFactor         = 2;
  float xCmd, yCmd, zCmd;
  // There is a deadband in position control
  // the z cmd is absolute height
  // while x and y are in relative
  float zDeadband = 0.12 * 10;

  /*! Calculate the inputs to send the position controller. We implement basic
   *  receding setpoint position control and the setpoint is always 1 m away
   *  from the current position - until we get within a threshold of the goal.
   *  From that point on, we send the remaining distance as the setpoint.
   */
  if (xOffsetDesired > 0)
    xCmd = (xOffsetDesired < speedFactor) ? xOffsetDesired : speedFactor;
  else if (xOffsetDesired < 0)
    xCmd =
      (xOffsetDesired > -1 * speedFactor) ? xOffsetDesired : -1 * speedFactor;
  else
    xCmd = 0;

  if (yOffsetDesired > 0)
    yCmd = (yOffsetDesired < speedFactor) ? yOffsetDesired : speedFactor;
  else if (yOffsetDesired < 0)
    yCmd =
      (yOffsetDesired > -1 * speedFactor) ? yOffsetDesired : -1 * speedFactor;
  else
    yCmd = 0;

  zCmd = currentBroadcastGP.height + zOffsetDesired;

  //! Main closed-loop receding setpoint position control
  while (elapsedTimeInMs < timeoutInMilSec)
  {
    v->control->positionAndYawCtrl(xCmd, yCmd, zCmd, yawDesiredRad / DEG2RAD);

    HAL_Delay(cycleTimeInMs);
    elapsedTimeInMs += cycleTimeInMs;

    broadcastQ         = v->broadcast->getQuaternion();
    yawInRad           = toEulerAngle((static_cast<void*>(&broadcastQ))).z;
    currentBroadcastGP = v->broadcast->getGlobalPosition();
    localOffsetFromGpsOffset(v, localOffset,
                              static_cast<void*>(&currentBroadcastGP),
                              static_cast<void*>(&originBroadcastGP));

    //! See how much farther we have to go
    xOffsetRemaining = xOffsetDesired - localOffset.x;
    yOffsetRemaining = yOffsetDesired - localOffset.y;
    zOffsetRemaining = zOffsetDesired - (-localOffset.z);

    //! See if we need to modify the setpoint
    if (std::abs(xOffsetRemaining) < speedFactor)
      xCmd = xOffsetRemaining;
    if (std::abs(yOffsetRemaining) < speedFactor)
      yCmd = yOffsetRemaining;

    if(v->getFwVersion() == Version::M100_31 &&
       std::abs(xOffsetRemaining) < posThresholdInM &&
       std::abs(yOffsetRemaining) < posThresholdInM &&
       std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
    {
      //! 1. We are within bounds; start incrementing our in-bound counter
      withinBoundsCounter += cycleTimeInMs;
    }
    else if(std::abs(xOffsetRemaining) < posThresholdInM &&
	   std::abs(yOffsetRemaining) < posThresholdInM &&
	   std::abs(zOffsetRemaining) < zDeadband &&
	   std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
    {
      //! 1. We are within bounds; start incrementing our in-bound counter
      withinBoundsCounter += cycleTimeInMs;
    }
    else
    {
      if (withinBoundsCounter != 0)
      {
        //! 2. Start incrementing an out-of-bounds counter
        outOfBounds += cycleTimeInMs;
      }
    }
    //! 3. Reset withinBoundsCounter if necessary
    if (outOfBounds > outOfControlBoundsTimeLimit)
    {
      withinBoundsCounter = 0;
      outOfBounds         = 0;
    }
    //! 4. If within bounds, set flag and break
    if (withinBoundsCounter >= withinControlBoundsTimeReqmt)
    {
      break;
    }
  }

  if (elapsedTimeInMs >= timeoutInMilSec)
  {
    printf("Task timeout!\n");

    return ACK::FAIL;
  }

  return ACK::SUCCESS;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
 *  coordinates (accurate when distances are small).
 */
void
localOffsetFromGpsOffset(Vehicle* vehicle, Telemetry::Vector3f& deltaNed,
                         void* target, void* origin)
{
  Telemetry::GPSFused*       subscriptionTarget;
  Telemetry::GPSFused*       subscriptionOrigin;
  Telemetry::GlobalPosition* broadcastTarget;
  Telemetry::GlobalPosition* broadcastOrigin;
  double                     deltaLon;
  double                     deltaLat;

  broadcastTarget = (Telemetry::GlobalPosition*)target;
  broadcastOrigin = (Telemetry::GlobalPosition*)origin;
  deltaLon        = broadcastTarget->longitude - broadcastOrigin->longitude;
  deltaLat        = broadcastTarget->latitude - broadcastOrigin->latitude;
  deltaNed.x      = deltaLat * C_EARTH;
  deltaNed.y      = deltaLon * C_EARTH * cos(broadcastTarget->latitude);
  deltaNed.z      = broadcastTarget->altitude - broadcastOrigin->altitude;
}

Telemetry::Vector3f
toEulerAngle(void* quaternionData)
{
  Telemetry::Vector3f    ans;
  Telemetry::Quaternion* quaternion = (Telemetry::Quaternion*)quaternionData;

  double q2sqr = quaternion->q2 * quaternion->q2;
  double t0    = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
  double t1 =
    +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
  double t2 =
    -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
  double t3 =
    +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
  double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0) ? -1.0 : t2;

  ans.x = asin(t2);
  ans.y = atan2(t3, t4);
  ans.z = atan2(t1, t0);

  return ans;
}

void startGlobalPositionBroadcast(Vehicle* vehicle)
{
  uint8_t freq[16];

  /* Channels definition for A3/N3/M600
   * 0 - Timestamp
   * 1 - Attitude Quaternions
   * 2 - Acceleration
   * 3 - Velocity (Ground Frame)
   * 4 - Angular Velocity (Body Frame)
   * 5 - Position
   * 6 - GPS Detailed Information
   * 7 - RTK Detailed Information
   * 8 - Magnetometer
   * 9 - RC Channels Data
   * 10 - Gimbal Data
   * 11 - Flight Status
   * 12 - Battery Level
   * 13 - Control Information
   */
  freq[0]  = DataBroadcast::FREQ_HOLD;
  freq[1]  = DataBroadcast::FREQ_HOLD;
  freq[2]  = DataBroadcast::FREQ_HOLD;
  freq[3]  = DataBroadcast::FREQ_HOLD;
  freq[4]  = DataBroadcast::FREQ_HOLD;
  freq[5]  = DataBroadcast::FREQ_50HZ; // This is the only one we want to change
  freq[6]  = DataBroadcast::FREQ_HOLD;
  freq[7]  = DataBroadcast::FREQ_HOLD;
  freq[8]  = DataBroadcast::FREQ_HOLD;
  freq[9]  = DataBroadcast::FREQ_HOLD;
  freq[10] = DataBroadcast::FREQ_HOLD;
  freq[11] = DataBroadcast::FREQ_HOLD;
  freq[12] = DataBroadcast::FREQ_HOLD;
  freq[13] = DataBroadcast::FREQ_HOLD;

  vehicle->broadcast->setBroadcastFreq(freq);
}