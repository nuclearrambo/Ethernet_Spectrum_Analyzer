
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "driverlib/adc.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "drivers/pinout.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/ssi.h"
#include "utils/lwiplib.h"
#include "arm_math.h"
#define SYSTICK_INT_PRIORITY    0x80
#define ETHERNET_INT_PRIORITY   0xC0
#define SAMPLE_LENGTH 2048
#define NETWORK_SAFE_SIZE 1400
void ClockInit();
void ADCInit();
int ReadADC();
void udp_initialize();
uint8_t testString[] = {"This is a test string"};
typedef struct  {
	float input[SAMPLE_LENGTH-1];
	float output[SAMPLE_LENGTH/2-1];
	char output_string[10*SAMPLE_LENGTH/2-1];
	struct pbuf *p;
} Signal;
Signal processor;

typedef struct {
	uint32_t pui32IPArray;
	uint8_t pui8MACArray[8];
	struct pbuf *p;
	struct udp_pcb *upcb;
	struct ip_addr server_ip, local_ip;
}Ethernet;
Ethernet eth0;
uint32_t buffer[1];
uint32_t ui32User0, ui32User1;
uint8_t pui8MACArray[8] = {0x00, 0x1A, 0xB6, 0x02, 0xC9, 0x6D};
uint32_t pui32IPArray = (192<<24)|(168<<16)|(1<<8)|(33);

uint32_t sysClock;

void
SysTickIntHandler(void)
{
    //
    // Call the lwIP timer handler.
    //
    lwIPTimer(500);
}

void
lwIPHostTimerHandler(void)
{

}
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void
ConfigureSSI(void){
	SysCtlPeripheralReset(SYSCTL_PERIPH_SSI2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);
	GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);
	GPIOPinConfigure(GPIO_PD2_SSI2FSS);
	GPIOPinConfigure(GPIO_PD3_SSI2CLK);

	SSIConfigSetExpClk(SSI2_BASE, sysClock, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 7500000, 16);
	SSIEnable(SSI2_BASE);
}

void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, sysClock);
}


int
main(void)
{
    //
    // Run from the PLL at 120 MHz.
    //
    sysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                SYSCTL_CFG_VCO_480), 120000000);

    //
    // Configure the device pins.
    //
    PinoutSet(true, false);

    //
    // Enable the GPIO pins for the LED D1 (PN1).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);

    //
    // Initialize the UART.
    //
    ConfigureUART();

    //
    // Configure ADC
    //
    ADCInit();

    lwIPInit(sysClock, pui8MACArray, pui32IPArray, 0, 0, IPADDR_USE_STATIC);
    MAP_IntPrioritySet(INT_EMAC0, ETHERNET_INT_PRIORITY);
	MAP_IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY);
    udp_initialize();
    int i;
    while(1){
		for(i = 0; i<1024; i++){
			processor.input[2*i] = (float)ReadADC();
			processor.input[2*i+1] = 0.0;			//memcpy(&testInput[2*i], &uintADCBuff, sizeof(float32_t));
		}

		arm_cfft_radix4_instance_f32 S;

		arm_cfft_radix4_init_f32(&S, 1024, 0, 1);
		arm_cfft_radix4_f32(&S, processor.input);
		arm_cmplx_mag_f32(processor.input, processor.output,
								1024);
		processor.output[0] = 0.0; //make the DC bin
		for(i = 0; i<1024; i++){
			processor.output[i] = (int)round(processor.output[i]);
		}
		for(i=0; i<512; i++){
				sprintf(processor.output_string + strlen(processor.output_string), "%.2f\n", processor.output[i]);
		}
		uint16_t output_string_size = strlen(processor.output_string);
		uint16_t chunks = output_string_size/NETWORK_SAFE_SIZE;
		uint16_t chunk = 0;
		while(chunk <= chunks){
			processor.p = pbuf_alloc(PBUF_TRANSPORT, NETWORK_SAFE_SIZE, PBUF_RAM);
			memcpy(processor.output_string, "SOF", 3);
			memcpy(processor.p->payload, processor.output_string + 3 + chunk*1400, 1400); //offset 3 to compensate the SOF marker
			udp_send(eth0.upcb, processor.p);
			pbuf_free(processor.p);
			i=0;
			chunk++;
		}
		//clear the output string memory
		memset(processor.output_string, 0, sizeof(processor.output_string));
    }
}

void ADCInit(){
	 SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	 GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); //PE_3 on launchpad
	 ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, 6);
	 ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	 ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
	                              ADC_CTL_END);
	 ADCSequenceEnable(ADC0_BASE, 3);
	 ADCIntClear(ADC0_BASE, 3);

	 //UART
	 SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	 GPIOPinConfigure(GPIO_PA0_U0RX);
	 GPIOPinConfigure(GPIO_PA1_U0TX);
	 GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	 UARTConfigSetExpClk(UART0_BASE, sysClock, 1000000, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	 UARTEnable(UART0_BASE);

}

int ReadADC()
{

	 ADCProcessorTrigger(ADC0_BASE, 3);
	 while(!ADCIntStatus(ADC0_BASE, 3, false));
	 ADCIntClear(ADC0_BASE, 3);
	 ADCSequenceDataGet(ADC0_BASE, 3, buffer);
	 //SysCtlDelay(SysCtlClockGet() / 2);
	 return buffer[0];
}

void udp_rcvCallback(void *arg, struct udp_pcb *udp_pcb, struct pbuf *p, struct ip_addr *addr, uint16_t port){
	if(p != NULL){
		//parse the commands coming from remote dest
		memcpy(p->payload, " RECEIVED UDP COMMAND", 20);
		udp_send(udp_pcb, p);
	}
	pbuf_free(p);
}

void udp_initialize(void){
	eth0.upcb = udp_new();

	IP4_ADDR(&eth0.server_ip, 192,168,1,27);
	IP4_ADDR(&eth0.local_ip, 192,168,1,33);

	udp_bind(eth0.upcb, &eth0.local_ip, 2222);
	eth0.p = pbuf_alloc(PBUF_TRANSPORT, sizeof(processor.output), PBUF_RAM);
	udp_connect(eth0.upcb, &eth0.server_ip, 2222);
	udp_recv(eth0.upcb, udp_rcvCallback, NULL);
}

