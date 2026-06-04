/**
  ******************************************************************************
  * @file    moving_average.c
  * @author  Mohammad Hussein Tavakoli Bina, Sepehr Hashtroudi
  * @brief   This file contains an efficient implementation of
	*					 moving average filter.
  ******************************************************************************
	*MIT License
	*
	*Copyright (c) 2018 Mohammad Hussein Tavakoli Bina
	*
	*Permission is hereby granted, free of charge, to any person obtaining a copy
	*of this software and associated documentation files (the "Software"), to deal
	*in the Software without restriction, including without limitation the rights
	*to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	*copies of the Software, and to permit persons to whom the Software is
	*furnished to do so, subject to the following conditions:
	*
	*The above copyright notice and this permission notice shall be included in all
	*copies or substantial portions of the Software.
	*
	*THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	*IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	*FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	*AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	*LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	*OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	*SOFTWARE.
  */

#ifndef MOVING_AVERAGE_MOVING_AVERAGE_C_
#define MOVING_AVERAGE_MOVING_AVERAGE_C_

/* Includes ------------------------------------------------------------------*/
#include "moving_average.h"

/**
  * @brief  This function initializes filter's data structure.
	* @param  filter_struct : Data structure
  * @retval None.
  */
void Moving_Average_Init_Low(FilterTypeDef* filter_struct)
{
	filter_struct->SumLow = 0;
	filter_struct->WindowPointerLow = 0;

	for(uint32_t i=0; i<WindowLengthLow; i++)
	{
		filter_struct->HistoryLow[i] = 0;
	}
}

void Moving_Average_Init_High(FilterTypeDef* filter_struct)
{
	filter_struct->SumHigh = 0;
	filter_struct->WindowPointerHigh = 0;

	for(uint32_t i=0; i<WindowLengthHigh; i++)
	{
		filter_struct->HistoryHigh[i] = 0;
	}
}

/**
  * @brief  This function filters data with moving average filter.
	* @param  raw_data : input raw sensor data.
	* @param  filter_struct : Data structure
  * @retval Filtered value.
  */
// Moving Average for Low Range
float Moving_Average_Compute_Low(uint32_t raw_data, FilterTypeDef* filter_struct)
{
	filter_struct->SumLow += raw_data;
	filter_struct->SumLow -= filter_struct->HistoryLow[filter_struct->WindowPointerLow];
	filter_struct->HistoryLow[filter_struct->WindowPointerLow] = raw_data;
	if(filter_struct->WindowPointerLow < WindowLengthLow - 1)
	{
		filter_struct->WindowPointerLow += 1;
	}
	else
	{
		filter_struct->WindowPointerLow = 0;
	}
	return (float)filter_struct->SumLow/(float)WindowLengthLow;
}

// Moving Average for High Range
float Moving_Average_Compute_High(uint32_t raw_data, FilterTypeDef* filter_struct)
{
	filter_struct->SumHigh += raw_data;
	filter_struct->SumHigh -= filter_struct->HistoryHigh[filter_struct->WindowPointerHigh];
	filter_struct->HistoryHigh[filter_struct->WindowPointerHigh] = raw_data;
	if(filter_struct->WindowPointerHigh < WindowLengthHigh - 1)
	{
		filter_struct->WindowPointerHigh += 1;
	}
	else
	{
		filter_struct->WindowPointerHigh = 0;
	}
	return (float)filter_struct->SumHigh/(float)WindowLengthHigh;
}

#endif /* MOVING_AVERAGE_MOVING_AVERAGE_C_ */
