#include "GPIO_Config.h"
#include <util/delay.h>
#include <Wire.h>

#define BAUDRATE 9600
#define MYUBRR F_CPU/16/BAUDRATE-1

struct Data {
  byte Temp;
  byte Inc;
  byte Vel_MA;
  byte Vel_MB;
  byte T_Lvl;
  byte Person;
  byte Prod;
};

struct Data data;

volatile uint16_t adc0_val = 0;
volatile uint16_t adc1_val = 0;
volatile uint8_t current_channel = 0;
int timer_count = 0;
bool stop_prod = false;
byte command = 0;
int click_stop_count = 0;

ISR(INT1_vect){
  _delay_us(100);
  PORTC ^= 0b00001000;  // ou PORTC ^= 0x10;
  stop_prod ^= 1;
}
  


ISR(ADC_vect){
  if(timer_count == 1000 && stop_prod == false){
      timer_count = 0;
      char buffer[255];
    if (current_channel == 0) {
        adc0_val = (ADC * 1023) / 255;
        ADMUX = (ADMUX & 0xF0) | 0x00; // Muda para canal 0
        current_channel = 1;
  
    } else {
        current_channel = 0;
        adc1_val = (ADC * 1023) / 255;
        ADMUX = (ADMUX & 0xF0) | 0x01; // Muda para canal 1
    }
  }

}

ISR(TIMER2_COMPA_vect) //interrupção de TC2(8 bits) a cada 1ms =(249+1)/(16MHz/64)
{
  if(stop_prod == false){
    timer_count += 1;
  }
}

ISR(USART_RX_vect){
    char received;
    received = UDR0;
}


int main(){

  Pin_Config('C', 0, 0, 1);
  Pin_Config('C', 1, 0, 1);
  Pin_Config('C', 2, 1, 1);
  Pin_Config('D', 3, 0, 1);
  External_Interrupt(1);
  Timer_2_CTC_Init();
  ADC_Init();
  USART_Init(MYUBRR);
  Wire.begin(8); 
  Wire.onRequest(Send_Data);
  Wire.onReceive(Command_Received);
  sei();
  while(1){
  }
  return 1;
}

void Command_Received(){
  char buffer[256];
  if(Wire.available()){
      command = Wire.read();
      if(command == 0){
        for (byte i = 0; i < sizeof(data); i++) {
          *((byte*)&data + i) = Wire.read();
        }
        itoa(data.Temp, buffer, 10);
        Serial_Print("Temperatura: "); Serial_Println(buffer);
        if(data.Inc == 1){
          Serial_Println("Madeira fora do eixo!!!");
        }else{
          Serial_Println("Status madeira: Normal");
        }
        itoa(adc0_val, buffer, 10);
        Serial_Print("Motor A: "); Serial_Println(buffer);
        itoa(adc1_val, buffer, 10);
        Serial_Print("Motor B: "); Serial_Println(buffer);
        itoa(data.T_Lvl, buffer, 10);
        Serial_Print("Nivel do Tanque: "); Serial_Println(buffer);
        itoa(data.Person, buffer, 10);
        if(data.Person == 1){
          Serial_Println("Presença detectada!!!");
        }else{
          Serial_Print("Status sensor de presença: Normal");
        }
        itoa(data.Prod, buffer, 10);
        Serial_Print("Blocos cortados: "); Serial_Println(buffer);
      }else if(command == 4 && stop_prod == false){
        stop_prod = true;
        Serial_Println("Parada Realizada com sucesso!!!");
      }else if(command == 5){
        stop_prod = false;
        Serial_Println("Reinicializando Produção!!!");
      }        
    
  }
}

void Send_Data(){
  if(command == 1  && stop_prod == false){ //Enviar valores dos potenciometros
    Wire.write((adc0_val >> 8) & 0xFF);
    Wire.write(adc0_val & 0xFF);

    Wire.write((adc1_val >> 8) & 0xFF);
    Wire.write(adc1_val & 0xFF);
  }if(command == 2){ //Solicitação de parada de emergencia
    Wire.write(stop_prod);
  }else{
    Wire.write(0);
  }
}