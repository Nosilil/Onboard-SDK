#include "main.h"
#include "MedianFilter.h"

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

#define NUM_ELEMENTS	(19)

static sMedianFilter_t medianFilter;
static sMedianNode_t medianBuffer[NUM_ELEMENTS];

int MediaFilterExample()
{
    medianFilter.numNodes = NUM_ELEMENTS;
    medianFilter.medianBuffer = medianBuffer;
    
    MEDIANFILTER_Init(&medianFilter);
    
    while(1)
    {
        int newValue = rand() % 10;
        int medianValue = MEDIANFILTER_Insert(&medianFilter, newValue);
        printf("New value: %d \tMedian value: %d\r\n", newValue, medianValue);
        HAL_Delay(30);
    }

    return 0;
}
