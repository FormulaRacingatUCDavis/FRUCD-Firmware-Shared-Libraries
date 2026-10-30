#include <stdint.h> 

#include "iir.h"

/**
  * @brief  Initializes an IIR filter
  * @param  filter: Pointer to the IIR_t to initialize
  * @param  init_alpha: The value to initialize alpha to
  * @param  init_val: The value to initialize prev_val to
  */
void IIR_Init_With_Alpha(IIR_t *filter, float init_alpha, uint32_t init_val) {
    filter->alpha = init_alpha;
    filter->prev_val = init_val;
}

/**
  * @brief  Initializes an IIR filter
  * @param  filter: Pointer to the IIR_t to initialize
  * @param  f_c: The cutoff frequency for the filter
  * @param  f_s: The sampling frequency for the filter
  * @param  init_val: The value to initialize prev_val to
  */
void IIR_Init(IIR_t *filter, uint16_t f_c, uint16_t f_s, uint32_t init_val) {
    filter->alpha = calculate_alpha(f_c, f_s);
    filter->prev_val = init_val;
}

/**
  * @brief  Updates an IIR filter and returns the filtered value
  * @note   Will change filter->prev_val when called
  * @param  filter: Pointer to the IIR_t
  * @param  curr_input: The current measurement to be used in the filter
  * @retval The value after running the filter with the given input
  */
uint32_t IIR_update(IIR_t *filter, uint16_t curr_input) {
    float y_curr = filter->prev_val + filter->alpha * (curr_input - filter->prev_val);
    //derived from first order IIR filter equation y[n] = y[n-1] + a(x[n] - y[n-1])
    //curr_input corresponds to x[n] 
    //prev_val corresponds to y[n-1]
    //y_curr corresponds to y[n]

    filter->prev_val = y_curr;
    //save current y to previous value

    return y_curr;
}

/**
  * @brief  Calculates an alpha value given a cutoff and sampling frequency
  * @note   Ideally should only be called once per filter (during initialization)
  * @param  f_c: The cutoff frequency for the filter
  * @param  f_s: The sampling frequency for the filter
  * @retval The calculated alpha value
  */
inline float calculate_alpha(uint16_t f_c, uint16_t f_s) {
    return (1.0 - exp(-2.0 * 3.1415 * (f_c / f_s)));
    //f_c is the cutroff frequency
    //f_s is the sampling frequency
}