/*
 * LCDTest.c
 *
 * Created: 02/02/2017 20:26:37
 *  Author: Xavi
 */ 

/*LCD_RS -> A5
LCD_WR -> A4
LCD_RD -> 3V3
DB8 -> D0
DB9 -> D1
DB10 -> D2
Touch_CLK -> A1
DB11 -> D3
Touch_CS -> GND
DB12 -> D4
Touch_DIN -> A0
DB13 -> D15
DB14 -> D6
LCD_CS -> A3*/

/*
	PORTC0 (SDA) -> A3 -> LCD_CS
	PORTC1 (SCL) -> A4 -> LCD_WR
	PORTC2 (RXD) -> A5 -> LCD_RS
*/

//---------------------------------------------------------------
// _SFR_MEM8 and _SFR_MEM16 are defined by including 
//---------------------------------------------------------------
//#define _SFR_MEM8(mem_addr) (*(volatile uint8_t *)(mem_addr))
//#define _SFR_MEM16(mem_addr)(*(volatile uint16_t *)(mem_addr))
//---------------------------------------------------------------

#include <avr/io.h>

#define LCD_I80_PARALLEL_BITS 8
#define PORTC0 (uint8_t) 0x01
#define PORTC1 (uint8_t) 0x02
#define PORTC2 (uint8_t) 0x04
#define PORTC3 (uint8_t) 0x08

#define PORTx_SET_BIT_HIGH(x, y) ((x).OUT = (1 << y))

static void LCDBusWrite(uint8_t reg, uint8_t data);
static void LCDInit(void);

typedef struct t_LCDPinout
{
	volatile uint8_t CS;
	volatile uint8_t WR;
	volatile uint8_t RS;
	
	PORT_t DB[LCD_I80_PARALLEL_BITS];
}TYPE_LCD_PINOUT;

static TYPE_LCD_PINOUT LCDPinout;

void LCDInit(void)
{
	LCDPinout.CS = PORTC0;
	LCDPinout.WR = PORTC1;
	LCDPinout.RS = PORTC2;
	
	PORTC.DIR = LCDPinout.CS | LCDPinout.WR | LCDPinout.RS;
	
	PORTx_SET_BIT_HIGH(PORTC, PORTC0);
}

void LCDBusWrite(uint8_t reg, uint8_t data)
{
	
}

int main(void)
{	
	LCDInit();
	
	PORTR.PIN0CTRL = PORT_OPC_TOTEM_gc;
	
	PORTR.DIR = 0x01;
	
    while(1)
    {
		PORTR.OUT = 0x01;
		
		asm("nop");
        //TODO:: Please write your application code 
    }
}