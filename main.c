/*****************************************************************************************************
 * Include
 ****************************************************************************************************/
// standard C
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
// inc
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
// driverlib
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
// utils
//#include "utils/uartstdio.h"

/*****************************************************************************************************
 * Function prototypes
 ****************************************************************************************************/
//void DataLoggingON(void);
//void DataLoggingOFF(void);
void Init(void);

/*****************************************************************************************************
 * Defines
 ****************************************************************************************************/
// Loopback slave address
//#define SLAVE_ADDRESS 0x3C
// Tg
#define SLAVE_ADDRESS_TG 0x4B
#define TG_REG_READ 0x00
#define TG_REG_LOWPOW 0x01
// RTC
#define SLAVE_ADDR_RTC 0x68
#define SEC 0x00
#define MIN 0x01
#define HRS 0x02
#define DAY 0x03
#define DATE 0x04
#define MONTH 0x05
#define YEAR 0x06
#define CNTRL 0x07
// Va ADC slave address
#define SLAVE_ADDR_VA 0x48
#define VA_REG_READ 0x00
// SWITCHES
#define SW_PORT 	GPIO_PORTC_BASE
#define SW_ON 		GPIO_PIN_5
#define SW_SD 		GPIO_PIN_6
#define SW_1 		GPIO_PIN_7
// LEDS
#define LED_PORT 	GPIO_PORTE_BASE
#define LED_ON 		GPIO_PIN_0
#define LED_1 		GPIO_PIN_1
#define LED_2 		GPIO_PIN_2
// Timer0
#define TOGGLE_FREQUENCY 1
// I2C3
#define GPIO_PD0_I2C3SCL        0x00030003
#define GPIO_PD1_I2C3SDA        0x00030403

/*****************************************************************************************************
 * Global variables
 ****************************************************************************************************/
//Datos a mantener durante la hibernacion
//0: sensor_flag
//1: hibernate_flag
//2: Tg y RTC
unsigned long ulNVData[3] = { 1, 0, 0};
//
//static unsigned long g_ulDataRx, MSB,LSB, Sign;
static unsigned long Tg_Raw, Tg_MSB,Tg_LSB, Tg_Sign;
unsigned char sec,min,hour,day,date,month,year;
float Tg=0;
//static stirng Dias[7] = {'Lunes', 'Martes', 'Miercoles', 'Jueves', 'Viernes', 'Sabado', 'Domingo'};

/*****************************************************************************************************
 * IntGPIOFHandler
 ****************************************************************************************************/
void IntGPIOFHandler (void){
	GPIOPinIntClear(GPIO_PORTF_BASE,GPIO_PIN_4);
	//Flag de que estoy en la interrupcion por GPIO
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x04);//
	SysCtlDelay(8000000);
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
	SysCtlDelay(4000000);
	ulNVData[2]=1;


}

/*****************************************************************************************************
 * Init
 ****************************************************************************************************/
void Init(void){
	//
	// Clock
	//
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	//
	// IHM
	//
	//Salidas
	//Habilito el puerto C
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlDelay(10);
	//PC5,PC6PC7 como entradas
	GPIOPinTypeGPIOInput(SW_PORT, SW_ON|SW_SD|SW_1);
	//Enable pull-up resistor
	GPIOPadConfigSet(SW_PORT,SW_ON,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(SW_PORT,SW_SD,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(SW_PORT,SW_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	//
	//Entradas
	//Habilito el puerto E
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlDelay(10);
	//PE0,PE1 y PE2 salidas
	GPIOPinTypeGPIOOutput(LED_PORT, LED_ON|LED_1|LED_2);

}

/*****************************************************************************************************
 * InitI2C0
 ****************************************************************************************************/
//initialize I2C module 0 & 3
//Slightly modified version of TI's example code
void InitI2C0(void)
{
	//
	// The I2C0 peripheral must be enabled before use.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
	//
	// For this example I2C3 is used with PortB[0:1].  The actual port and
	// pins used may be different on your part, consult the data sheet for
	// more information.  GPIO port D needs to be enabled so these pins can
	// be used.
	// TODO: change this to whichever GPIO port you are using.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	//
	// Configure the pin muxing for I2C3 functions on port D0 and D1.
	// This step is not necessary if your part does not support pin muxing.
	// TODO: change this to select the port/pin you are using.
	//
	GPIOPinConfigure(GPIO_PD0_I2C3SCL);
	GPIOPinConfigure(GPIO_PD1_I2C3SDA);
	//
	// Select the I2C function for these pins.  This function will also
	// configure the GPIO pins pins for I2C operation, setting them to
	// open-drain operation with weak pull-ups.  Consult the data sheet
	// to see which functions are allocated per pin.
	// TODO: change this to select the port/pin you are using.
	//
	GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
	//
	// Enable loopback mode.  Loopback mode is a built in feature that helps
	// for debug the I2Cx module.  It internally connects the I2C master and
	// slave terminals, which effectively lets you send data as a master and
	// receive data as a slave.  NOTE: For external I2C operation you will need
	// to use external pull-ups that are faster than the internal pull-ups.
	// Refer to the datasheet for more information.
	//
	//HWREG(I2C3_BASE + I2C_O_MCR) |= 0x01;
	//
	// Enable the I2C3 interrupt on the processor (NVIC).
	//
	//IntEnable(I2C0_IRQHandler);
	//
	// Configure and turn on the I2C3 slave interrupt.  The I2CSlaveIntEnableEx()
	// gives you the ability to only enable specific interrupts.  For this case
	// we are only interrupting when the slave device receives data.
	//
	//I2CSlaveIntEnableEx(I2C0_BASE, I2C_SLAVE_INT_DATA);
	//
	// Enable and initialize the I2C3 master module.  Use the system clock for
	// the I2C0 module.  The last parameter sets the I2C data transfer rate.
	// If false the data rate is set to 100kbps and if true the data rate will
	// be set to 400kbps.  For this example we will use a data rate of 100kbps.
	//
	I2CMasterInitExpClk(I2C3_BASE, SysCtlClockGet(), false);
	//
	// Enable the I2C3 slave module.
	//
	//I2CSlaveEnable(I2C3_BASE);

	//
	// Set the slave address to SLAVE_ADDRESS.  In loopback mode, it's an
	// arbitrary 7-bit number (set in a macro above) that is sent to the
	// I2CMasterSlaveAddrSet function.
	//
	//I2CSlaveInit(I2C3_BASE, SLAVE_ADDRESS);

	//
	// Tell the master module what address it will place on the bus when
	// communicating with the slave.  Set the address to SLAVE_ADDRESS
	// (as set in the slave module).  The receive parameter is set to false
	// which indicates the I2C Master is initiating a writes to the slave.  If
	// true, that would indicate that the I2C Master is initiating reads from
	// the slave.
	//
	//I2CMasterSlaveAddrSet(I2C3_BASE, SLAVE_ADDRESS, false);
	//
	// Enable interrupts to the processor.
	//
	//IntMasterEnable();
}

/*****************************************************************************************************
 * I2CSend
 ****************************************************************************************************/
// Sends an I2C command to the specified slave
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...)
{
	// Tell the master module what address will place on the bus when
	// communicating with the slave.
	I2CMasterSlaveAddrSet(I2C3_BASE, slave_addr, false);

	// Stores list of variable number of arguments
	va_list vargs;
	// Specifies the va_list to "open" and the last fixed argument
	//so vargs knows where to start looking
	va_start(vargs, num_of_args);

	// Put data to be sent into FIFO
	I2CMasterDataPut(I2C3_BASE, va_arg(vargs, uint32_t));
	// If there is only one argument, we only need to use the
	//single send I2C function
	if(num_of_args == 1)
	{
		//Initiate send of data from the MCU
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
		// Wait until MCU is done transferring.
		SysCtlDelay(500);
		while(I2CMasterBusy(I2C3_BASE));
		//"close" variable argument list
		va_end(vargs);
	}
	// Otherwise, we start transmission of multiple bytes on the
	//I2C bus
	else
	{
		// Initiate send of data from the MCU
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		// Wait until MCU is done transferring.
		SysCtlDelay(500);
		while(I2CMasterBusy(I2C3_BASE));
		// Send num_of_args-2 pieces of data, using the
		//BURST_SEND_CONT command of the I2C module
		unsigned char i;
		for(i = 1; i < (num_of_args - 1); i++)
		{
			// Put next piece of data into I2C FIFO
			I2CMasterDataPut(I2C3_BASE, va_arg(vargs, uint32_t));
			// Send next data that was just placed into FIFO
			I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
			// Wait until MCU is done transferring.
			SysCtlDelay(500);
			while(I2CMasterBusy(I2C3_BASE));
		}
		// Put last piece of data into I2C FIFO
		I2CMasterDataPut(I2C3_BASE, va_arg(vargs, uint32_t));
		// Send next data that was just placed into FIFO
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
		// Wait until MCU is done transferring.
		SysCtlDelay(500);
		while(I2CMasterBusy(I2C3_BASE));
		//"close" variable args list
		va_end(vargs);
	}
}

/*****************************************************************************************************
 * I2CReceive
 ****************************************************************************************************/
// Read specified register on slave device
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg)
{
	// Specify that we are writing (a register address) to the
	//slave device
	I2CMasterSlaveAddrSet(I2C3_BASE, slave_addr, false);
	//specify register to be read
	I2CMasterDataPut(I2C3_BASE, reg);
	//send control byte and register address byte to slave device
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	//wait for MCU to finish transaction
	SysCtlDelay(500);
	while(I2CMasterBusy(I2C3_BASE));
	//specify that we are going to read from slave device
	I2CMasterSlaveAddrSet(I2C3_BASE, slave_addr, true);
	//send control byte and read from the register we
	//specified
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
	//wait for MCU to finish transaction
	SysCtlDelay(500);
	while(I2CMasterBusy(I2C3_BASE));
	//return data pulled from the specified register
	return I2CMasterDataGet(I2C3_BASE);
}
/*****************************************************************************************************
 * dec2bcd
 ****************************************************************************************************/
//decimal to BCD conversion
unsigned char dec2bcd(unsigned char val)
{
	return (((val / 10) << 4) | (val % 10));
}
// convert BCD to binary
unsigned char bcd2dec(unsigned char val)
{
	return (((val & 0xF0) >> 4) * 10) + (val & 0x0F);
}

/*****************************************************************************************************
 * SetTimeDate
 ****************************************************************************************************/
//Set Time
void SetTimeDate(unsigned char sec, unsigned char min, unsigned char hour,unsigned char day, unsigned char date, unsigned char month,unsigned char year)
{
	I2CSend(SLAVE_ADDR_RTC,8,SEC,dec2bcd(sec),dec2bcd(min),dec2bcd(hour),dec2bcd(day),dec2bcd(date),dec2bcd(month),dec2bcd(year));
}

/*****************************************************************************************************
 * GetClock
 ****************************************************************************************************/
//Get Time and Date
unsigned char GetClock(unsigned char reg)
{
	unsigned char clockData = I2CReceive(SLAVE_ADDR_RTC,reg);
	return bcd2dec(clockData);
}

/*****************************************************************************************************
 * main
 *
 * Using DS1307 clock and calendar
 * Adding Tg
 *
 ****************************************************************************************************/
void main(void)
{
	//Init
	Init();
	// Set the clocking to run directly from the external crystal/oscillator.
	//SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ);
	// Initialize I2C module 0
	InitI2C0();

	// TODO Set time and date
	//SetTimeDate(00,49,19,1,30,9,16);		// Se queda acá adentro.
	// Será por la tension del DS?
	//
	SysCtlDelay(100);


	while(1)
	{

		SysCtlDelay(1000000);
		//
		// RTC
		//
		sec = GetClock(SEC);
		min = GetClock(MIN);
		hour = GetClock(HRS);
		//day = GetClock(DAY);
		date = GetClock(DATE);
		month = GetClock(MONTH);
		year = GetClock(YEAR);
		SysCtlDelay(SysCtlClockGet()/10*3);
		//
		// Tg
		//
		I2CMasterSlaveAddrSet(I2C3_BASE,SLAVE_ADDRESS_TG, false);
		I2CMasterDataPut(I2C3_BASE, TG_REG_READ);
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		while(!I2CMasterBusy(I2C3_BASE));
		while(I2CMasterBusy(I2C3_BASE));
		I2CMasterSlaveAddrSet(I2C3_BASE, SLAVE_ADDRESS_TG, true);
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
		while(!I2CMasterBusy(I2C3_BASE));
		while(I2CMasterBusy(I2C3_BASE));
		// Most significant bits
		//Tg_MSB = I2CMasterErr(I2C3_MASTER_BASE);
		Tg_MSB = I2CMasterDataGet(I2C3_BASE) << 8;
		// Less significant bits
		I2CMasterControl (I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
		while(!I2CMasterBusy(I2C3_BASE));
		while(I2CMasterBusy(I2C3_BASE));
		Tg_LSB = I2CMasterDataGet(I2C3_BASE);
		// Low power mode on
		I2CMasterSlaveAddrSet(I2C3_BASE,SLAVE_ADDRESS_TG, false);
		I2CMasterDataPut(I2C3_BASE, TG_REG_LOWPOW);
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		while(!I2CMasterBusy(I2C3_BASE));
		//while(I2CMasterBusy(I2C3_BASE));
		// Final value
		Tg_Raw = (Tg_MSB+ Tg_LSB) >> 3;
		Tg_Sign = Tg_Raw & 0x1000;
		Tg = Tg_Raw*0.0625;

	}
}
