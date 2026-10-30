#ifndef IIR_H
#define IIR_H

#include <stdint.h> 

typedef struct {
    float alpha;
    uint32_t prev_val;
} IIR_t;

void IIR_Init(IIR_t *filter, float init_alpha, uint32_t init_val);
void IIR_Init(IIR_t *filter, uint16_t f_c, uint16_t f_s, uint32_t init_val);
uint32_t IIR_update(IIR_t *filter, uint16_t curr_input);

inline float calculate_alpha(uint16_t f_c, uint16_t f_s);


#endif // ifndef IIR_H