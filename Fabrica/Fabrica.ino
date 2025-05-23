#include <LiquidCrystal_I2C.h>

#include "GPIO_Config.h"
#include <util/delay.h>
#include <Wire.h>

#define BAUDRATE 9600
#define MYUBRR F_CPU/16/BAUDRATE-1
#define SLAVE_ADDR 0X34
#define RPS 1.8 // 3.2 rotações por segundo para 9v ou seja RPS5V = 5*3.2/9 = 1.777777777777778
#define Vin 5.0 // define a tensão de entrada igual a 5.0
#define T0 298.15 // define constante igual a 298.15 Kelvin
#define Rt 10000 // Resistor do divisor de tensão
#define R0 10000 // Valor da resistência inicial do NTC
#define T1 273.15 // [K] no datasheet 0º C
#define T2 373.15 // [K] no datasheet 100° C
#define RT1 35563 // [ohms] Resistência em T1
#define RT2 549 // [ohms] Resistência em T2
float beta = 0.0; // parâmetros iniciais [K]
float Rinf = 0.0; // parâmetros iniciais [ohm]
float TempCelsius = 0.0; // variable output
float Vout = 0.0; // Vout em A0
float Rout = 0.0; // Rout em A0

volatile uint16_t adc0_val = 0; //Valor do sensor de presenção (LDR) - Pino A1
volatile uint16_t adc1_val = 0; //Valor do sensor de temperatura NTC 10K - Pino A0
volatile uint8_t current_channel = 0; //Canal atual do ADC
int timer_count = 0; // Contador de 1 segundo
int timer_count_info = 0; // Contador para o envio das informações para o supervisor
bool stop_prod = false; //Variavel de controle de produção
int count_temp = 0; // Numero de amostras de temperatura coletadas
int nsamples = 2; // Limite da coleta de amostras de temperatura
int plustemp = 50000; // Valor da soma das amostras (Começando de 5000 para a primeira amostra não dar valor 0 ou negativo
int click_stop_count = 0; // Contagem de clicks de parada da interrupção externa - 0 para desligar quando clicar e 1 para ligar
int Vel_MA = 255; // Velocidade do motor
int Vel_MB = 255; 
int MA_Rot_Count = 0; //Contador de rotações
int MB_Rot_Count = 0;
int Total_RA = 0; // Soma de rotações
int Total_RB = 0;

//Struct de dados para o envio das informações para o supervisor
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

void Request_Motor_Velocity(); // Função para solicitar valores do potenciometros do supervisor
void Stop_Emergency_Checkout(); // Função para verificar parada de emergencia
void Send_Infos(); // Função para enviar a informação
void Send_Stop_Info(); // Função para enviar comando de Stop de produção

LiquidCrystal_I2C lcd(0x27, 16, 2); // Configuração do LCD

ISR(INT0_vect){ //interrupção externa - Pino D2
  _delay_us(100);
  if(click_stop_count == 0){
    stop_prod = true; // Muda a variavel de controle de produção
    PWM_Control_OC0(0, 0); // Para os motores
    PORTC = 0b00001000; // Liga o Led vermelho e desliga o verde
    click_stop_count++;
    Serial_Println("Parada Realizada com sucesso");

  }else{
    Serial_Println("Reiniciando produção!!");
    click_stop_count = 0;
    stop_prod = false;
    PORTC = 0b00000100;
    PWM_Control_OC0(255, 255);
  }
}
  

ISR(ADC_vect) //Interrupção do ADC - Fluxo principal do codigo
{
  if(timer_count == 1000 && stop_prod == false){
    int distance = Ultrassonic_Read(); // Leitura do ultrassonico(Sensor de nivel) - Trigger: Pino D8 / Echo: Pino D12
    data.T_Lvl = distance; // Salvando dado na struct
    char buffer_ultrassonic[255];
    itoa(distance, buffer_ultrassonic, 10); //Convertendo para texto
    Serial_Print("Distancia: ");
    Serial_Print(buffer_ultrassonic);
    Serial_Println(" cm");
    
    PWM_Cycle_Control_OC1(60, 210); // Configura o servo motor da esteira para a posição normal
    char buffer[255];
    if (current_channel == 0) {
        adc0_val = ADC; // Salva leitura do canal 0
        ADMUX = (ADMUX & 0xF0) | 0x00; // Muda para canal 1
        current_channel = 1;
        itoa(adc0_val, buffer, 10); 
        Serial_Print("Presença: ");
        Serial_Println(buffer);
        if(adc0_val > 500){ //Valore maiores que 500 igual a presença detectada / Converte para 0 ou 1 nessa condicional
          PWM_Control_OC0(0, 0);
          data.Person = 1;
        }else{
          data.Person = 0;
        }
    } else {
        count_temp += 1;
        plustemp += adc1_val;
        if(count_temp == nsamples){
          current_channel = 0;
          adc1_val = ADC; // Salva leitura do canal 1
          ADMUX = (ADMUX & 0xF0) | 0x01; // Muda para canal 0
          count_temp = 0;
          plustemp = 0;
          Vout = Vin * ((float)adc1_val / 1024.0); // cálculo de V0 e leitura de A0
          Rout = (Rt * Vout / (Vin - Vout)); // cálculo de Rout
          TempCelsius = (beta / log(Rout / Rinf)) - 273.15; // cálculo da temp. em Celsius
          if(TempCelsius < 10 || TempCelsius > 40){ //Verifica valores criticos da temperatura
            click_stop_count = 0;
            stop_prod = true;
            PORTC = 0b00001000;
            PORTD |= (1<<PD4);
            Serial_Println("Temperatura Critica!!!");
          }
          data.Temp = TempCelsius;
          Serial_Print("Temperatura: ");
          itoa(TempCelsius, buffer, 10);
          Serial_Println(buffer);
        }
    }
    if(PIND & (1 << PD7)){ // Verifica valor do sensor que indica inclinação da madeira
      data.Inc = 0;
    }else{
      Serial_Println("Madeira fora do eixo!!!");
      PWM_Cycle_Control_OC1(240, 210); // Sensor detecta e move o servo motor para concertar o angulo
      data.Inc = 1;
    }
    timer_count = 0;
    //Calculo de rotações para saber o numero exato de toras cortadas
    MA_Rot_Count = (RPS * Vel_MA)/255;
    MB_Rot_Count = (RPS * Vel_MB)/255;
    Total_RA += MA_Rot_Count;
    Total_RB += MB_Rot_Count;
    if(Total_RB >= 10 && Total_RA >= 25){ //Verificação de rotação dos motores verticais e orisontais de acordo com as especificações do RPS necessario
      Total_RA -= 25;
      Total_RB -= 10;
      data.Prod += 1;
    }
  }
  
}

ISR(TIMER2_COMPA_vect) //interrupção de TC2(8 bits) a cada 1ms =(249+1)/(16MHz/64)
{
  if(stop_prod == false){
    timer_count += 1;
    timer_count_info += 1;
  }
}

ISR(USART_RX_vect){ // Receptor do monitor serial
    char received;
    received = UDR0;
    

}

int main(void){
    //Porta: A,B,C ou D | Pino: 0 a 7 | Modo: Saida(1) ou Entrada(0) | Pull: Up(1) ou Down(0)
    Pin_Config('B', 0, 1, 0);
    Pin_Config('B', 1, 1, 0);
    Pin_Config('B', 2, 1, 0);
    Pin_Config('B', 3, 1, 0);
    Pin_Config('B', 4, 0, 0);

    Pin_Config('C', 0, 0, 1);
    Pin_Config('C', 1, 0, 1);
    Pin_Config('C', 2, 1, 0);

    Pin_Config('D', 2, 0, 1);
    Pin_Config('D', 4, 1, 0);
    Pin_Config('D', 5, 1, 0);
    Pin_Config('D', 6, 1, 0);
    Pin_Config('D', 7, 0, 0);
    ADC_Init(); //Inicialização do ADC
    PWM_Init_OC0(); // Inicialização dos motores de corte
    PWM_Cycle_Init_OC1(); // Inicialização do servo
    External_Interrupt(0); // Interrupção INT0
    Timer_2_CTC_Init(); // Inicializando Timer 2
    USART_Init(MYUBRR); // Inicializando monitor serial
    PWM_Control_OC0(255, 255); // Ligando motores
    sei(); //Ativando interrupção global
    Wire.begin();
    lcd.begin(16,2);
    lcd.backlight();
    PORTC = 0b00000100; //Led verde inicia ligado
    //Cálculos para temperatura
    beta = (log(RT1 / RT2)) / ((1 / T1) - (1 / T2)); // cálculo de beta
    Rinf = R0 * exp(-beta / T0); // cálculo de Rinf
    while(1){
      if(timer_count_info >= 3000 && stop_prod == 0){
        Send_Infos();
        _delay_ms(500);
        timer_count_info = 0;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Contagem de toras:");
        char buffer_lcd[256];
        itoa(data.Prod, buffer_lcd, 10);
        lcd.setCursor(0,1);
        lcd.print(buffer_lcd);
      }
      if(stop_prod == 0){
        Request_Motor_Velocity();
        _delay_ms(500);
        Stop_Emergency_Checkout();
        _delay_ms(500);
        Send_Stop_Info();
        _delay_ms(500);
      }
      Stop_Emergency_Checkout();
      _delay_ms(500);
    }
    return 0;
}

void Send_Infos(){
  Wire.beginTransmission(8);
  Wire.write(0);
  Wire.write((byte*)&data, sizeof(data)); //Envio da struct de dados
  Wire.endTransmission();
  _delay_ms(1); // Pequeno delay para garantir que escravo esteja pronto

}

void Send_Stop_Info(){
  if(stop_prod == true){
    Wire.beginTransmission(8);
    Wire.write(4); // Comando para pedir os dois inteiros
    Wire.endTransmission();

    _delay_us(50); // Pequeno delay para garantir que escravo esteja pronto
  }
}

void Request_Motor_Velocity(){
  Wire.beginTransmission(8);
  Wire.write(1); // Comando para pedir os dois inteiros
  Wire.endTransmission();

  _delay_us(50); // Pequeno delay para garantir que escravo esteja pronto

  Wire.requestFrom(8, 4); // Dois inteiros = 4 bytes
  char Abuffer[256];
  char Bbuffer[256];
  if (Wire.available() >= 4) { //Verificando recebimento dos dados do s motores
    int Vel_MA = Wire.read() << 8 | Wire.read();
    Serial_Print("Velocidade Motor A: ");
    itoa(Vel_MA, Abuffer, 10);
    Serial_Println(Abuffer);
    int Vel_MB = Wire.read() << 8 | Wire.read();
    Serial_Print("Velocidade Motor B: ");
    itoa(Vel_MB, Bbuffer, 10);
    Serial_Println(Bbuffer);
    PWM_Control_OC0(Vel_MA, Vel_MB);
  }
}

void Stop_Emergency_Checkout(){
  Wire.beginTransmission(8);
  Wire.write(2); // Comando para verificar parada de emergência
  Wire.endTransmission();

  _delay_us(50); // Pequeno delay para garantir que escravo esteja pronto

  Wire.requestFrom(8, 1); // Espera 1 byte como resposta
  if (Wire.available() && stop_prod == false) {
    int message = Wire.read();
    char buffer[256];
    itoa(message, buffer, 10);
    Serial_Print("Recebido STOP: "); Serial_Println(buffer);
    if (message) {
      Serial_Print("Parada de emergência: ");
      Serial_Println("Realizando parada!");
      stop_prod = true;
      PWM_Control_OC0(0, 0);
      PORTC = 0b00001000;
      click_stop_count++;
      Serial_Println("Parada realizada com sucesso!");
    }
  }else if(Wire.available() && stop_prod == false){
    int message = Wire.read();
    if(message == 0){
      stop_prod = false;
      PORTC = 0b00000100;
      click_stop_count = 0;
      Serial_Println("Reiniciando...");
    }
  }
}

