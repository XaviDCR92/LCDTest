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
#include <util/delay.h>

#define LCD_I80_PARALLEL_BITS 8
#define PORTC0 (uint8_t) PIN0_bp
#define PORTC1 (uint8_t) PIN1_bp
#define PORTC2 (uint8_t) PIN2_bp
#define PORTC3 (uint8_t) PIN3_bp

#define PORTR0 (uint8_t) PIN0_bp

#define PORTx_SET_BIT_HIGH(x, y)		((x).OUTSET = (1 << y))
#define PORTx_SET_BIT_LOW(x,y)			((x).OUTCLR = (1 << y))
#define PORTx_SET_BIT_AS_OUTPUT(x,y)	((x).DIRSET = (1 << y))
#define PORTx_SET_BITS_AS_OUTPUT(x,y)	((x).DIRSET = y)
#define PORTx_SET_BIT_AS_INPUT(x, y)	((x).DIRCLR = (1 << y))
#define PORTx_SET_BITS_AS_INPUT(x, y)	((x).DIRCLR = y)

#define LCD_DRIVER_OUTPUT_CONTROL_REGISTER 0x01
#define LCD_INDEX_REGISTER 0x00
#define LCD_ENTRY_MODE_REGISTER 0x03

static void LCDBusWrite(uint8_t reg, uint16_t data);
static void LCDInit(void);
static void setClockTo32MHz(void);

typedef struct t_LCDPinout
{
	volatile uint8_t CS;
	volatile uint8_t WR;
	volatile uint8_t RS;
	volatile PORT_t* PORT;

	volatile uint8_t DB7_0;
	
}TYPE_LCD_PINOUT;

static TYPE_LCD_PINOUT LCDPinout;

void LCDInit(void)
{
	LCDPinout.CS = PORTC0;
	LCDPinout.WR = PORTC1;
	LCDPinout.RS = PORTC2;
	
	LCDPinout.PORT->DIR |= LCDPinout.CS | LCDPinout.WR | LCDPinout.RS;
}

void LCDBusWrite(uint8_t reg, uint16_t data)
{
	PORTx_SET_BIT_LOW(PORTC, LCDPinout.CS);
	PORTx_SET_BITS_AS_OUTPUT(PORTA, 0xFF /* All bits as output */);
	
	// Now write to register index!
	
	PORTx_SET_BIT_LOW(PORTC, LCDPinout.RS);
	
	PORTA.OUTSET = 0x00; // MSB for index register must be set to 0x00
	// Low pulse
	PORTx_SET_BIT_LOW(PORTC, LCDPinout.WR);
	PORTx_SET_BIT_HIGH(PORTC, LCDPinout.WR);
	
	PORTA.OUTSET = reg;
	// Low pulse
	PORTx_SET_BIT_LOW(PORTC, LCDPinout.WR);
	PORTx_SET_BIT_HIGH(PORTC, LCDPinout.WR);
	
	PORTx_SET_BIT_HIGH(PORTC, LCDPinout.RS); // Finished writing index register
	
	// Now write data!
	
	PORTA.OUTSET = (uint8_t) (data >> 8);
	// Low pulse
	PORTx_SET_BIT_LOW(PORTC, LCDPinout.WR);
	PORTx_SET_BIT_HIGH(PORTC, LCDPinout.WR);
	
	PORTA.OUTSET = (uint8_t) (data & 0x00FF);
	// Low pulse
	PORTx_SET_BIT_LOW(PORTC, LCDPinout.WR);
	PORTx_SET_BIT_HIGH(PORTC, LCDPinout.WR);
	
	PORTx_SET_BIT_HIGH(PORTC, LCDPinout.CS);
}

void LCDBusRead(uint8_t reg, uint16_t data)
{
	// Reading isn't possible if using 2.8" ITDB02 Shield
	// RD pin is set to a high level, so only write is possible.
}

void setClockTo32MHz(void)
{
	OSC.CTRL |= OSC_RC32MEN_bm | OSC_RC32KEN_bm;  /* Enable the internal 32MHz & 32KHz oscillators */
	while(!(OSC.STATUS & OSC_RC32KRDY_bm));       /* Wait for 32Khz oscillator to stabilize */
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));       /* Wait for 32MHz oscillator to stabilize */
	DFLLRC32M.CTRL = DFLL_ENABLE_bm ;             /* Enable DFLL - defaults to calibrate against internal 32Khz clock */
	CCP = CCP_IOREG_gc;                           /* Disable register security for clock update */
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;              /* Switch to 32MHz clock */
	OSC.CTRL &= ~OSC_RC2MEN_bm;                   /* Disable 2Mhz oscillator */
}

void UTFTStartup(void)
{
	LCDBusWrite(0xE5, 0x78F0); // set SRAM internal timing
	LCDBusWrite(0x01, 0x0100); // set Driver Output Control
	LCDBusWrite(0x02, 0x0200); // set 1 line inversion
	LCDBusWrite(0x03, 0x1030); // set GRAM write direction and BGR=1.
	LCDBusWrite(0x04, 0x0000); // Resize register
	LCDBusWrite(0x08, 0x0207); // set the back porch and front porch
	LCDBusWrite(0x09, 0x0000); // set non-display area refresh cycle ISC[3:0]
	LCDBusWrite(0x0A, 0x0000); // FMARK function
	LCDBusWrite(0x0C, 0x0000); // RGB interface setting
	LCDBusWrite(0x0D, 0x0000); // Frame marker Position
	LCDBusWrite(0x0F, 0x0000); // RGB interface polarity
	//*************Power On sequence ****************//
	LCDBusWrite(0x10, 0x0000); // SAP, BT[3:0], AP, DSTB, SLP, STB
	LCDBusWrite(0x11, 0x0007); // DC1[2:0], DC0[2:0], VC[2:0]
	LCDBusWrite(0x12, 0x0000); // VREG1OUT voltage
	LCDBusWrite(0x13, 0x0000); // VDV[4:0] for VCOM amplitude
	LCDBusWrite(0x07, 0x0001);
	_delay_ms(200); // Dis-charge capacitor power voltage
	LCDBusWrite(0x10, 0x1690); // SAP, BT[3:0], AP, DSTB, SLP, STB
	LCDBusWrite(0x11, 0x0227); // Set DC1[2:0], DC0[2:0], VC[2:0]
	_delay_ms(50); // Delay 50ms
	LCDBusWrite(0x12, 0x000D); // 0012
	_delay_ms(50); // Delay 50ms
	LCDBusWrite(0x13, 0x1200); // VDV[4:0] for VCOM amplitude
	LCDBusWrite(0x29, 0x000A); // 04  VCM[5:0] for VCOMH
	LCDBusWrite(0x2B, 0x000D); // Set Frame Rate
	_delay_ms(50); // Delay 50ms
	LCDBusWrite(0x20, 0x0000); // GRAM horizontal Address
	LCDBusWrite(0x21, 0x0000); // GRAM Vertical Address
	// ----------- Adjust the Gamma Curve ----------//
	LCDBusWrite(0x30, 0x0000);
	LCDBusWrite(0x31, 0x0404);
	LCDBusWrite(0x32, 0x0003);
	LCDBusWrite(0x35, 0x0405);
	LCDBusWrite(0x36, 0x0808);
	LCDBusWrite(0x37, 0x0407);
	LCDBusWrite(0x38, 0x0303);
	LCDBusWrite(0x39, 0x0707);
	LCDBusWrite(0x3C, 0x0504);
	LCDBusWrite(0x3D, 0x0808);
	//------------------ Set GRAM area ---------------//
	LCDBusWrite(0x50, 0x0000); // Horizontal GRAM Start Address
	LCDBusWrite(0x51, 0x00EF); // Horizontal GRAM End Address
	LCDBusWrite(0x52, 0x0000); // Vertical GRAM Start Address
	LCDBusWrite(0x53, 0x013F); // Vertical GRAM Start Address
	LCDBusWrite(0x60, 0xA700); // Gate Scan Line
	LCDBusWrite(0x61, 0x0001); // NDL,VLE, REV
	LCDBusWrite(0x6A, 0x0000); // set scrolling line
	//-------------- Partial Display Control ---------//
	LCDBusWrite(0x80, 0x0000);
	LCDBusWrite(0x81, 0x0000);
	LCDBusWrite(0x82, 0x0000);
	LCDBusWrite(0x83, 0x0000);
	LCDBusWrite(0x84, 0x0000);
	LCDBusWrite(0x85, 0x0000);
	//-------------- Panel Control -------------------//
	LCDBusWrite(0x90, 0x0010);
	LCDBusWrite(0x92, 0x0000);
	LCDBusWrite(0x07, 0x0133); // 262K color and display ON
}

int main(void)
{	
	setClockTo32MHz();
	
	LCDInit();
	
	PORTx_SET_BIT_AS_OUTPUT(PORTR, PORTR0);
	
	
	
	/*LCDBusWrite(LCD_DRIVER_OUTPUT_CONTROL_REGISTER, 0x0100);
	
	LCDBusWrite(LCD_ENTRY_MODE_REGISTER, 0x0030); // I/D[1:0] = 11b
	
	LCDBusWrite(0x0C, 0x0003);  // We are NOT using RGB interface!*/
	
	UTFTStartup();
	
	
    while(1)
    {
		_delay_ms(500);
		PORTx_SET_BIT_HIGH(PORTR, PORTR0);
		//LCDBusWrite(0x07, 0x0000);
		_delay_ms(500);
		PORTx_SET_BIT_LOW(PORTR, PORTR0);
		//LCDBusWrite(0x07, 0x0002);
    }
}