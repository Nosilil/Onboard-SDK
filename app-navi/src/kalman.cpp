// Kalman filter cpp by Yuan

#include "kalman.hpp"
#include "uav_control.hpp"
#include "user_diskio.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include "main.h"

#define CL_LIST_NUM 3

extern dw1000_usb_data_t dw1000_usb_data;

// Kalman::Kalman()
Kalman::Kalman(UavControl* uav_control, Vehicle* vehicle)
    : uav_control(uav_control)
    , vehicle(vehicle)
{
    data_log = new DataLog();

    tGps.longitude = -0.623127 * DEG2RAD;
    tGps.latitude = 51.894195 * DEG2RAD;
    tGps.altitude = 0;

    position.x = 0;
    position.y = 0;
    position.z = 0;

    ultrasonic = 0;
    voltage = 0;

    om.x = 0;
    om.y = 0;
    om.z = 0;
    om.health = 0;
    om.uwb = 0;
    om.usRange = 0;
    om.volt = 0;

    updatedGps.x = 0;
    updatedGps.y = 0;

    accuUpdatedGps.x = 0;
    accuUpdatedGps.y = 0;

    circularLocalisationInit();
}

Kalman::~Kalman()
{

}

void Kalman::updateOnboardMeasure()
{
    Telemetry::TimeStamp        t;
    LocalVelocity v;

    // position from vehicle GPS
    Telemetry::GlobalPosition   gps;

    v = uav_control->velocity;

    t = vehicle->broadcast->getTimeStamp();
    gps = vehicle->broadcast->getGlobalPosition();
    
    //latitude difference
    om.x = (float32_t)((tGps.latitude - gps.latitude) * C_EARTH);
    //longitude difference
    om.y = (float32_t)((tGps.longitude - gps.longitude) * C_EARTH * cos(gps.latitude));

    om.z = gps.height;
    om.health = gps.health;

    om.uwb = ((float32_t)dw1000_usb_data.rng_avg) / 1000;
    om.usRange = ((float32_t)ultrasonic) / 100;

    //12 bits adc
    om.volt = 0.3 - ((float32_t)voltage) * 30 / 409500;

    data_log->updateOnboardMeasure(t, gps, om);
    data_log->updateLocalVelocity(HAL_GetTick(), v);
}

void Kalman::positionUpdate()
{
	position.x = om.x + updatedGps.x;
    position.y = om.y + updatedGps.y;;

    data_log->updateKalmanPosition(position);
}

void Kalman::kalmanOutput()
{
    static uint8_t count = 0;

	updateOnboardMeasure();

    // uwb compensation
//    if(count++ % 25 == 0)
//    {
//       if(om.uwb > 0.3)
//            updateGpsCoordinateFromUwb();
//       else
//       {
//           if(om.uwb != 0)
//               updateGpsCoordinateFromVolt();
//       }
//    }

    positionUpdate();
}

void Kalman::updateGpsCoordinateFromUwb()
{
    // CircularLocalisation*       clPrev = cl;
    // CircularLocalisation*       clPtr = cl->Next;
    CircularLocalisation*       clTmp = cl;
    LocalPosition               intersection[CL_LIST_NUM];
    uint8_t                     count = 0;

    cl = cl->Next;

    // update from onboardMeasure
    cl->x = om.x;
    cl->y = om.y;
    if(om.usRange == 7.65)
        cl->r = om.uwb > om.z ? sqrt(pow(om.uwb, 2) - pow(om.z, 2)) : 0;
    else
        cl->r = om.uwb > om.usRange ? sqrt(pow(om.uwb, 2) - pow(om.usRange, 2)) : 0;

    // for(int i = 0; i < CL_LIST_NUM; i++)
    // {
    //     if(getCircularIntersection(clPtr, clPrev, intersection + i) != 0)
    //         count++;
    //     clPrev = clPrev->Next;
    //     clPtr = clPtr->Next;
    // }

    // if(count != 0)
    // {
    //     LocalPosition intersectionAvg;
    //     for(int i = 0; i < CL_LIST_NUM; i++)
    //     {
    //         intersectionAvg.x += intersection[i].x;
    //         intersectionAvg.y += intersection[i].y;
    //     }
    //     updatedGps.x = 0.7 * updatedGps.x + 0.3 * intersectionAvg.x / count;
    //     updatedGps.y = 0.7 * updatedGps.y + 0.3 * intersectionAvg.y / count;
    // }

    if(getCircularIntersection(cl, clTmp, intersection) != 0)
        count ++;
    if(getCircularIntersection(cl->Next, cl, intersection + 1) != 0)
        count ++;
    if(getCircularIntersection(clTmp, cl->Next, intersection + 2) != 0)
        count ++;

    if(count != 0)
    {
        updatedGps.x = 0.7 * updatedGps.x + 0.3 * (intersection[0].x + intersection[1].x + intersection[2].x) / count;
        updatedGps.y = 0.7 * updatedGps.y + 0.3 * (intersection[0].y + intersection[1].y + intersection[2].y) / count;
        updatedGps.z = 0;
        // tGps.latitude = tGps.latitude - ((float64_t)updatedGps.x) / C_EARTH;
        // tGps.longitude = tGps.longitude - ((float64_t)updatedGps.y) / (C_EARTH * cos(tGps.latitude));
    }

    accuUpdatedGps.x = 0.99 * accuUpdatedGps.x + 0.01 * updatedGps.x;
    accuUpdatedGps.y = 0.99 * accuUpdatedGps.y + 0.01 * updatedGps.y;

    data_log->updateTGpsValueBy(updatedGps, accuUpdatedGps);
}

void
Kalman::updateGpsCoordinateFromVolt()
{


}

uint8_t 
Kalman::getCircularIntersection(CircularLocalisation* cl1, CircularLocalisation* cl2, LocalPosition* lp)
{
    float32_t A;
    float32_t B;
    float32_t a;
    float32_t b;
    float32_t c;
    float32_t delta;
    float32_t x1;
    float32_t x2;
    float32_t y1;
    float32_t y2;

    // algebra method to solve the intersection of two circles
    A = (cl1->y - cl2->y) / (cl2->x - cl1->x);
    B = (pow(cl1->r, 2) + pow(cl2->x, 2) + pow(cl2->y, 2) - pow(cl2->r, 2) - pow(cl1->x, 2) - pow(cl1->y, 2)) / 
        (2 * (cl2->x - cl1->x));

    a = pow(A, 2) + 1;
    b = 2 * (A * B - cl1->x * A - cl1->y);
    c = pow(B, 2) - 2 * cl1->x * B - pow(cl1->r, 2) + pow(cl1->x, 2) + pow(cl1->y, 2);

    delta = pow(b, 2) - 4 * a * c;

    // two intersection points
    if(delta > 0)
    {
        // two y coordinates
        y1 = (-1 * b + sqrt(delta)) / (2 * a);
        y2 = (-1 * b - sqrt(delta)) / (2 * a);
        float32_t tmp = sqrt(abs(pow(cl1->r, 2) - pow(y1 - cl1->y, 2)));
        // one of x is the right coordinate
        x1 = cl1->x + tmp;
        x2 = cl1->x - tmp;
        tmp = sqrt(abs(pow(cl1->r, 2) - pow(y2 - cl1->y, 2)));

        // test if this one is the right one
        if(abs(pow(x1 - cl2->x, 2) + pow(y1 - cl2->y, 2) - pow(cl2->r, 2)) >
        		abs(pow(x2 - cl2->x, 2) + pow(y1 - cl2->y, 2) - pow(cl2->r, 2)))
            x1 = x2;
        // test 
        x2 = cl1->x + tmp;
        if(abs(pow(x2 - cl2->x, 2) + pow(y2 - cl2->y, 2) - pow(cl2->r, 2)) >
        	abs(pow(cl1->x - tmp - cl2->x, 2) + pow(y2 - cl2->y, 2) - pow(cl2->r, 2)))
            x2 = cl1->x - tmp;

        if(abs(pow(x1 - cl1->Next->x, 2) + pow(y1 - cl1->Next->y, 2) - pow(cl1->Next->r, 2)) < 
            abs(pow(x2 - cl1->Next->x, 2) + pow(y2 - cl1->Next->y, 2) - pow(cl1->Next->r, 2)))
        {
            lp->x = x1;
            lp->y = y1;
        } else {
            lp->x = x2;
            lp->y = y2;
        }
        return 1;
    // one intersection point
    } else if(delta == 0)
    {
        y1 = (-1 * b) / (2 * a);
        x1 = cl1->x + sqrt(abs(pow(cl1->r, 2) - pow(y1 - cl1->y, 2)));
        if(pow(x1 - cl2->x, 2) + pow(y1 - cl2->y, 2) != pow(cl2->r, 2))
            x1 = cl1->x - sqrt(pow(cl1->r, 2) - pow(y1 - cl1->y, 2));
        lp->x = x1;
        lp->y = y1;
        return 1;
    } else
    // no intersection, should not happen
    {
        lp->x = 0;
        lp->y = 0;
        return 0;
    } 
}

void Kalman::getUltrasonicData(uint16_t ultrasonic_byte)
{
    ultrasonic = ultrasonic_byte;
}

void Kalman::getVoltageData(uint32_t volt)
{
	voltage = volt;
}

 void Kalman::getRelativePositionData(float32_t* dist, float32_t* uwb_dist, float32_t* volt_dist, float32_t* g, float32_t* h)
//void Kalman::getRelativePositionData(float32_t* dist, float32_t* g, float32_t* h)
{
    *dist = sqrt(pow(position.x, 2) + pow(position.y, 2));
    *uwb_dist = sqrt(abs(pow(om.uwb, 2) - pow(om.usRange, 2)));
//    *uwb_dist = om.uwb;
    *volt_dist = om.volt;
    *g = atan2(position.y, position.x);
//    if(om.z < 5)
//    	*h = om.usRange;
//    else
    	*h = om.z;
    
    data_log->writeDataToSpi();
}

// var: 0 set all
//      1 set long/lati
//      2 set altitude
void Kalman::setTargetGps(Telemetry::GlobalPosition g, uint8_t var)
{
    switch (var)
    {
    case 0:
        tGps = g;
        break;
    case 1:
        tGps.longitude = g.longitude;
        tGps.latitude = g.latitude;
        break;
    case 2:
        tGps.altitude = g.altitude;
    default:
        break;
    }
}

void Kalman::circularLocalisationInit()
{
    // CircularLocalisation* clPtr = new CircularLocalisation;

    // cl = clPtr;

    // for(int i = 0; i < CL_LIST_NUM - 1; i++)
    // {
    //     clPtr->Next = new CircularLocalisation;
    //     clPtr = clPtr->Next;
    // }

    // clPtr->Next = cl;
    cl = new CircularLocalisation;
    cl->Next = new CircularLocalisation;
    cl->Next->Next = new CircularLocalisation;
    cl->Next->Next->Next = cl;

    // cl->x = 3;
    // cl->y = 4;
    // cl->r = 5;
    // cl->Next->x = -2;
    // cl->Next->y = 2;
    // cl->Next->r = 4;
    // cl->Next->Next->x = 0;
    // cl->Next->Next->y = 0;
    // cl->Next->Next->r = 1.17;
}

