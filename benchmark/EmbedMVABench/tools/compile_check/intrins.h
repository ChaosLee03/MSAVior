/* Stub for Keil C51 <intrins.h> */
#ifndef _INTRINS_H_STUB
#define _INTRINS_H_STUB
static inline void _nop_(void) {}
static inline unsigned char _crol_(unsigned char val, unsigned char n) { return (val << n) | (val >> (8-n)); }
static inline unsigned char _cror_(unsigned char val, unsigned char n) { return (val >> n) | (val << (8-n)); }
#endif
