#define F_CPU 16000000UL
#define F_SCL 100000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/twi.h>

void Pin_Config(uint8_t Port_Select, uint8_t Pin_Select, uint8_t Mode_Select, uint8_t Pull_Select);

void ADC_Init();

void PWM_Init_OC0();
void PWM_Control_OC0(uint8_t A_Channel, uint8_t B_Channel);
void PWM_Cycle_Init_OC1();
void PWM_Cycle_Control_OC1(uint8_t A_Channel, uint8_t B_Channel);

void PWM_Cycle_Init_OC0();
void PWM_Cycle_Control_OC0(uint8_t A_Channel, uint8_t B_Channel);


void External_Interrupt(uint8_t interrupt);

void Timer_2_CTC_Init();

void USART_Init(unsigned int ubrr);
void USART_Transmit(unsigned char data);
unsigned char USART_Receive(void);
void Serial_Println(unsigned char *buffer);
void Serial_Print(unsigned char *buffer);

void TWI_Init();
uint8_t TWI_Start(uint8_t Write_Address);
uint8_t TWI_Repeated_Start(uint8_t Read_Address);
void TWI_Stop();
uint8_t TWI_Write(uint8_t data);
uint8_t TWI_Read_Ack();
uint8_t TWI_Read_Nack();
uint8_t decimal_to_bcd(uint8_t val);
void TWI_Slave_Init(uint8_t address);
uint8_t TWI_Slave_Read();
void TWI_Slave_Write(uint8_t data);
int Ultrassonic_Read();