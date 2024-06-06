/*  

	Written by: Bernice Lien
	Student Number: 400382544

*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

void PortM_Init(void){
	//Use PortM pins (PM0-PM3) for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0x0F;        								// configure Port M pins (PM0-PM3) as input
  GPIO_PORTM_AFSEL_R &= ~0x0F;     								// disable alt funct on Port M pins (PM0-PM3)
  GPIO_PORTM_DEN_R |= 0x0F;        								// enable digital I/O on Port M pins (PM0-PM3)
																									// configure Port M as GPIO
  GPIO_PORTM_AMSEL_R &= ~0x0F;     								// disable analog functionality on Port M	pins (PM0-PM3)	
	return;
}

void PortN_Init(void){
	//Use PortN onboard LED	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};	// allow time for clock to stabilize
	GPIO_PORTN_DIR_R |= 0x02;        								// make PN0 out (PN0 built-in LED1)
 	GPIO_PORTN_AFSEL_R &= ~0x02;     								// disable alt funct on PN0
  	GPIO_PORTN_DEN_R |= 0x02;        								// enable digital I/O on PN0
																									// configure PN1 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF0F)+0x00000000;
  GPIO_PORTN_AMSEL_R &= ~0x02;     								// disable analog functionality on PN0		
	

}

void PortF_Init(void){
	//Use PortPF4 for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;				// activate clock for Port F
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){};	// allow time for clock to stabilize
	GPIO_PORTF_DIR_R |= 0x10;        								// configure Port F4 as output
	GPIO_PORTF_AFSEL_R &= ~0x10;     								// Disable alt funct on PN0 and PN1
  GPIO_PORTF_DEN_R |= 0x10;        								// enable digital I/O on Port F4
																									// configure Port F as GPIO
	GPIO_PORTF_AMSEL_R &= ~0x10;  	

	return;
}

void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	
	GPIO_PORTH_DIR_R |= 0xF;												
	GPIO_PORTH_AFSEL_R &= ~0xF;		 								
	GPIO_PORTH_DEN_R |= 0xF;																												
	GPIO_PORTH_AMSEL_R &= ~0xF;		 								
	return;
}

// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}

// Global variable visible in Watch window of debugger
// Increments at least once per button press
volatile unsigned long FallingEdges = 0;

// Initialize Port J
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x03;    										// Make PJ1 input 
  GPIO_PORTJ_DEN_R |= 0x03;     										// Enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000FF;	 								//  Configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;											//  Disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x03;													//	Enable weak pull up resistor on PJ1

}

void PortJ_Interrupt_Init(void){
	
		FallingEdges = 0;             		// Initialize counter

		GPIO_PORTJ_IS_R &= ~0x03;     						// (Step 1) PJ1 is Edge-sensitive 
		GPIO_PORTJ_IBE_R &= ~0x03;    						//     			PJ1 is not triggered by both edges 
		GPIO_PORTJ_IEV_R &= ~0x03;   						//     			PJ1 is falling edge event 
		GPIO_PORTJ_ICR_R =   0x03;  					// 					Clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTJ_IM_R = 0x03;      					// 					Arm interrupt on PJ1 by setting proper bit in IM register
    
		NVIC_EN1_R =  0x00080000;           					// (Step 2) Enable interrupt 51 in NVIC (which is in Register EN1)
	
		NVIC_PRI12_R =  0xA0000000;									// (Step 4) Set interrupt priority to 5

		EnableInt();																	// (Step 3) Enable Global Interrupt. lets go!
}
int scanOn = 0;
int rotate = 0;
int direction = 1;
int depth = 0;
int count = 0;
int tot_steps = 512;
void GPIOJ_IRQHandler(void){
  FallingEdges = FallingEdges + 1;	
		// toggle additional LED and scanON
    if((GPIO_PORTJ_DATA_R & 0b00000001) == 0) {		
			GPIO_PORTN_DATA_R ^= 0b00000010;
			scanOn = !scanOn;
			//UART_printf("scanOn");
			GPIO_PORTJ_ICR_R =0x01;
    }
    // Check if PM1 triggered the interrupt
    if((GPIO_PORTJ_DATA_R & 0b00000010) == 0) {
			GPIO_PORTN_DATA_R ^= 0b00000001;
	    rotate = !rotate;
			//UART_printf("rotate");
				GPIO_PORTJ_ICR_R =0x02; 
	} 	
 				
}

void FlashPF4(int count) 
{
		while (count--)
			{
			GPIO_PORTF_DATA_R ^= 0b00010000; 								// hello world!
			SysTick_Wait1ms(8);														// 0.1s delay
			GPIO_PORTF_DATA_R ^= 0b00010000;			
			}
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

void rotate_direction(int delay, int direction) {

	if(direction == 1){
		GPIO_PORTM_DATA_R = 0b00001100;
		SysTick_Wait1ms(delay);
		GPIO_PORTM_DATA_R = 0b00000110;
		SysTick_Wait1ms(delay);
		GPIO_PORTM_DATA_R = 0b00000011;
		SysTick_Wait1ms(delay);
		GPIO_PORTM_DATA_R = 0b00001001;
		SysTick_Wait1ms(delay);
	}
	else if (direction == -1) {
		GPIO_PORTM_DATA_R = 0b00001001;
		SysTick_Wait1ms(delay);
		GPIO_PORTM_DATA_R = 0b00000011;
		SysTick_Wait1ms(delay);
		GPIO_PORTM_DATA_R = 0b00000110;
		SysTick_Wait1ms(delay);
		GPIO_PORTM_DATA_R = 0b00001100;
		SysTick_Wait1ms(delay);
		}
}
//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortM_Init();
	PortF_Init();			
	PortJ_Init();
	PortH_Init();
	
	UART_printf("Program Begins\r\n");
	UART_printf(printf_buffer);

	status = VL53L1X_GetSensorId(dev, &wordData);


	// 1 Wait for device booted
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* 2 Initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);
	
	PortJ_Interrupt_Init();
	int input = 0;
	while(1)
		{
			input = UART_InChar();
			if (input == 's')
				break;
		}
	while (1) {
		SysTick_Wait10ms(5);
		if (scanOn == 1) { // If data capture is on
			status = VL53L1X_StartRanging(dev);   
			status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
			status = VL53L1X_GetDistance(dev, &Distance);
			if (rotate == 1) {
				for(int i = 0; i < 512; i++) {
				while (dataReady == 0){
					status = VL53L1X_CheckForDataReady(dev, &dataReady);
							//FlashLED3(1);
							FlashLED4(1);
							
							VL53L1_WaitMs(dev, 5);
				}
			if (rotate == 0)
				GPIO_PORTM_DATA_R = 0x00;
				dataReady = 0;
				if (i % 16 == 0){ 
					status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
					status = VL53L1X_GetDistance(dev, &Distance);
					FlashPF4(1);

					status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

					sprintf(printf_buffer,"%u \r\n", Distance); 
					UART_printf(printf_buffer); // transmits distance measurement

				}
				rotate_direction(2,direction); // rotates stepper motor 1 step
				if (scanOn == 0 || rotate ==0) { // If interrupt triggered, exits loop
					break; }
			}
					VL53L1X_StopRanging(dev);
					UART_printf("Data capture has stopped\r\n");
					scanOn = 0;
					rotate = 0;
					direction *= -1;
					GPIO_PORTM_DATA_R = 0x00;
					GPIO_PORTN_DATA_R = 0x00;
		}
		
	}
	
		if (scanOn == 0) { // If data capture is off
			while (scanOn == 0) {
				SysTick_Wait10ms(1);
				GPIO_PORTH_DATA_R ^= 0b00000001;
				if (scanOn == 1) break; 
				if(rotate == 1) {
					rotate_direction(2,direction);
					direction *=-1;

				}
				if(rotate == 0)
					GPIO_PORTM_DATA_R =0x00;
	
			}
		}

	}
}
