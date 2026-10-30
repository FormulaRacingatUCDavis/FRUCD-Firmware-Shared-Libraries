#ifndef INC_CAN_MANAGER_H_
#define INC_CAN_MANAGER_H_

#include <stdint.h> 

//Extract up to 8 bits from a byte into a 16 bit variable, used for staggered bits 
static inline uint16_t can_decode_u16_bits_staggered(uint8_t byte, uint8_t shift, uint8_t mask){
    return (uint16_t)(byte >> shift) & mask;
}

// extracts a uint16_t value from a buffer if given a buffer and its index 
static inline uint16_t can_decode_u16(const uint16_t *buf, uint8_t byte_index){
    return (uint16_t)((buf[byte_index + 1] << 8) | buf[byte_index]);
}

//Extract a 32 bit value from a buffer if given the lowest byte index in the buffer 
static inline uint32_t can_decode_u32(const uint8_t *buf, uint8_t byte_index){
    return ((uint32_t)buf[byte_index + 3] << 24) | //bits 24-31
           ((uint32_t)buf[byte_index + 2] << 16) | //bits 16-23
           ((uint32_t)buf[byte_index + 1] << 8) | //bits 8-15
           ((uint32_t)buf[byte_index]) //bits 0-7
}



#endif 