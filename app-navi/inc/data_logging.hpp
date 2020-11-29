// Data logging function by Yuan

#ifndef DATA_LOGGING_H_
#define DATA_LOGGING_H_

#include "dji_vehicle.hpp"
#include "data_declare.hpp"
#include "typedef.h"

// #include "kalman.hpp"

class Kalman;

class DataLog 
{
public:
 
#pragma pack(1)
    typedef struct LoggedData
    {
        uint32_t                    timeStamp;
        Telemetry::TimeStamp        vTime;

        // position from vehicle GPS
        Telemetry::GlobalPosition   vGps;
        
        // uwb ranging data
        dw1000_usb_data_t           uwbData;

        // ultrasonic data
        OnboardMeasure              onboardMeasure;

        // vehicle control input
        LocalVelocity               vVelocity;

        //
        LocalPosition               tGpsValueUpdate;
        LocalPosition               circularCoordinate;

        // position estimation data
        LocalPosition               kalmanPosition;               

        // maybe a separator here?
        uint32_t                    separator;
    } LoggedData;
#pragma pack(1)

public:

// constructor
DataLog();

// destructor
~DataLog();

public:

void updateLocalVelocity(uint32_t t, LocalVelocity v);

void updateOnboardMeasure(Telemetry::TimeStamp t, Telemetry::GlobalPosition gps, OnboardMeasure om);

void updateKalmanPosition(LocalPosition pos);

void updateTGpsValueBy(LocalPosition pos, LocalPosition circular);

uint32_t getLoggedTime();

void writeDataToSpi();

private:

LoggedData loggedData;
};



#endif // DATA_LOGGING_H_
