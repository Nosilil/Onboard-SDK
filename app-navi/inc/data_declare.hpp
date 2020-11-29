// by Yuan

#ifndef DATA_DECLARE_H_
#define DATA_DECLARE_H_

#include "main.h"

#define CTRL_FREQ 50
#define TIME_INV 0.02

#pragma pack(1)
    typedef struct LocalPosition
    {
        float32_t x;
        float32_t y;
        float32_t z;
    } LocalPosition;

    typedef struct LocalVelocity
    {
        float32_t x;
        float32_t y;
        float32_t z;
        float32_t yaw;
    } LocalVelocity;

    // NED coordination with sensor node origin
    typedef struct OnboardMeasure
    {
    	float32_t 	x;
    	float32_t 	y;
    	float32_t 	z;
    	uint8_t 	health;
    	float32_t 	uwb;
    	float32_t 	usRange;
    	float32_t 	volt;
    } OnboradMeasure;

     typedef struct CircularLocalisation
    {
        float32_t x;
        float32_t y;
        float32_t r;
        CircularLocalisation* Next;
        CircularLocalisation(): x(0), y(0), r(0), Next(nullptr)
        {
        }
    } CircularLocalisation;
#pragma pack(1)

#endif // DATA_DECLARE_H_
