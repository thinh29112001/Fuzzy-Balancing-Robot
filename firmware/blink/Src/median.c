/*
 * median.c
 *
 *  Created on: Dec 8, 2022
 *      Author: Lenovo
 */
#include "median.h"

float FindMedian(float a[], uint8_t n) {
	uint8_t i, j;
	float t, median;
	for (i = 1 ; i <= n-1 ; i++){ /* Trip-i begins */
	      for (j = 1 ; j <= n-i ; j++) {
	         if (a[j] <= a[j+1]) { /* Interchanging values */
	            t = a[j];
	            a[j] = a[j+1];
	            a[j+1] = t;
	         }
	         else
	         continue ;
	      }
	   } /* sorting ends */
	   /* calculation of median */
	   if ( n % 2 == 0)
	      median = (a[n/2] + a[n/2+1])/2.0 ;
	   else
	   median = a[n/2 + 1];
	   return median;
}
