/* Stub for Keil C51 <reg51.h> - 8051 SFR definitions */
#ifndef _REG51_H_STUB
#define _REG51_H_STUB

/* Port registers */
static volatile unsigned char P0, P1, P2, P3;

/* Timer registers */
static volatile unsigned char TH0, TL0, TH1, TL1;
static volatile unsigned char TMOD, TCON;

/* Serial port */
static volatile unsigned char SCON, SBUF, SCON0;

/* Interrupt enable register IE bits */
static volatile unsigned char IE, IP;
static volatile unsigned char EA_bit;
#define EA EA_bit
static volatile unsigned char ET0, ET1, ET2, EX0, EX1, ES;

/* Timer control bits (from TCON) */
static volatile unsigned char TF0, TF1, TR0, TR1, TF2;
static volatile unsigned char IT0, IT1, IE0, IE1;

/* Serial control bits */
static volatile unsigned char TI, RI;

/* Timer2 */
static volatile unsigned char T2CON, RCAP2L, RCAP2H, TL2, TH2, TR2;

/* Stack pointer, accumulator, etc. */
static volatile unsigned char SP, ACC, B, PSW, DPH, DPL;

/* Power control */
static volatile unsigned char PCON;

/* Additional SFRs */
static volatile unsigned char AUXR, AUXR1;

/* XBYTE for absolute memory access */
extern unsigned char __xdata_stub[];
#define XBYTE __xdata_stub

#endif
