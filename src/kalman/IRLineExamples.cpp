#include "Kalman.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>

// Source : https://medium.com/@ab.jannatpour/kalman-filter-with-python-code-98641017a2bd

#define IRSENSORS_COUNT 5

float pos_left, pos_right, total;
int IR_values[IRSENSORS_COUNT];
int IR_WaterLevel;
int IR_tresh, IR_max;

int crosses;
int cross_count, last_cross_count;
int cross_tresh;
int black_cross_level;

float blacks;   


IR_WaterLevel = 0;
IR_tresh = 512;
cross_tresh = 3;
black_cross_level = 2.8;

  

void example(void)
{
  byte c, found;
  int v, min, max, last_v, min_lim, max_lim, counter, mean;
  short i = 0;
  float intervalo = 0.0625;

  min = 0;
  max = 1023;
  min_lim = 480;                                          //512 - 32 [-2*16 -- right]
  max_lim = 544;                                          //512 + 32 [2*16  -- left]
   

  mean = 0;

  found = 0;
  IR_max = 0,
  total = 0;
  last_v = 0;
  counter = 15;
  
  for(i = 0; i < counter; i++)
  {
    for(c = 0; c < 5; c++)
    {
      v = IR_values[c] - IR_WaterLevel;

      if(v > max_lim) v = max;
      if(v < min_lim) v = min;

      total = total + v;
    }
    mean = total/5;
    IR_WaterLevel = mean;

    IR_tresh = IR_WaterLevel + intervalo * (min - IR_WaterLevel);
  }
/*

 if(v > min_lim && v < IR_tresh) 
    {
      v = min;
      total = total + v;
      IR
    }
    v = min;
    if(v < max_lim && v > min) v = max;

    total = total + v;
  }

  IR_WaterLevel = (min + max)/2;

  IR_tresh = IR_WaterLevel + 0.0625*(max - IR_WaterLevel);


*/

/*
    if(v < min) v = 0;
    if(v > max) v = 0;
    total = total + v;

    if(v > IR_max) IR_max = v;

    if (!found && last_v < IR_tresh && v > IR_tresh) 
    {
      IR_WaterLevel += (v - last_v) / 2;
      found = 1;
    }
    last_v = v;
    
    */
}


int main() {
    example();
    return 0;
}