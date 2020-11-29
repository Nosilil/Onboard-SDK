// data logging cpp by Yuan

#include "data_logging.hpp"
#include "user_diskio.h"


extern dw1000_usb_data_t dw1000_usb_data;

DataLog::DataLog()
{
    loggedData.separator = 0x5678;
}

DataLog::~DataLog()
{

}

void 
DataLog::updateLocalVelocity(uint32_t t, LocalVelocity v)
{
    loggedData.timeStamp = t;
    loggedData.vVelocity = v;
}

void 
DataLog::updateOnboardMeasure(Telemetry::TimeStamp t, 
                            Telemetry::GlobalPosition gps,
                            OnboardMeasure om)
{
    loggedData.vTime = t;
    loggedData.vGps = gps;
    loggedData.uwbData = dw1000_usb_data;
    loggedData.onboardMeasure = om;
    /* data read */
    dw1000_usb_data.data_updt = 0;
}

void 
DataLog::updateKalmanPosition(LocalPosition pos)
{
    loggedData.kalmanPosition = pos;
//    loggedData.separator = 38;
}

void 
DataLog::updateTGpsValueBy(LocalPosition updatedGps, LocalPosition circular)
{

    loggedData.circularCoordinate = circular;
    loggedData.tGpsValueUpdate = updatedGps;
}

uint32_t
DataLog::getLoggedTime()
{
    return loggedData.timeStamp;
}

void 
DataLog::writeDataToSpi()
{
    write2SD((uint8_t *)&loggedData, sizeof(LoggedData));
}
