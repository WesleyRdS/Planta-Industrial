#include "GPIO_Config.h"


void Pin_Config(uint8_t Port_Select, uint8_t Pin_Select, uint8_t Mode_Select, uint8_t Pull_Select) {
    // Direção: 1 = saída, 0 = entrada
    if (Mode_Select == 1) {
        // Configura como saída
        switch (Port_Select) {
            case 'B': DDRB |= (1 << Pin_Select); break;
            case 'C': DDRC |= (1 << Pin_Select); break;
            case 'D': DDRD |= (1 << Pin_Select); break;
            default: break;
        }
    } else {
        // Configura como entrada
        switch (Port_Select) {
            case 'B': DDRB &= ~(1 << Pin_Select); break;
            case 'C': DDRC &= ~(1 << Pin_Select); break;
            case 'D': DDRD &= ~(1 << Pin_Select); break;
            default: break;
        }
    }
    
    // Configura pull-up ou pull-down
    if (Mode_Select == 0) {
        switch (Pull_Select) {
            case 1: // Habilita pull-up
                switch (Port_Select) {
                    case 'B': PORTB |= (1 << Pin_Select); break;
                    case 'C': PORTC |= (1 << Pin_Select); break;
                    case 'D': PORTD |= (1 << Pin_Select); break;
                    default: break;
                }
                break;
            case 2: // Habilita pull-down 
                switch (Port_Select) {
                    case 'B': PORTB &= ~(1 << Pin_Select); break;
                    case 'C': PORTC &= ~(1 << Pin_Select); break;
                    case 'D': PORTD &= ~(1 << Pin_Select); break;
                    default: break;
                }
                break;
            default:
                break;
        }
    }
}

void ADC_Init(){
    // Usa AVcc como referência (bit 6 = 1), e começa no canal 0 (bits 3:0 = 0000)
    ADMUX = (1 << REFS0) | (0 << MUX0); // AVcc como ref, ADC0

    // Habilita ADC, interrupção, modo auto trigger (free running), prescaler 128
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADATE) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    ADCSRB = 0x00; // Free Running Mode

    // Desabilita buffers digitais dos canais 0 e 1 (opcional)
    DIDR0 = (1 << ADC0D) | (1 << ADC1D);
}

void PWM_Init_OC0(){
    TCCR0A = 0b10100011; // PWM rapido não invertido nos pinos OC0A e OC0B
    TCCR0B = 0b00000011; //Liga TC0, presclar: 64, fPWM = f0sc/(256*prescaler) = 16MHz/(256*64) = 976Hz
}

void PWM_Control_OC0(uint8_t A_Channel, uint8_t B_Channel){
    OCR0A = A_Channel; //Controle do ciclo ativo do PWM 0C0A(PD6)
    OCR0B = B_Channel; //Controle do ciclo ativo do PWM OC0B(PD5)
}

void PWM_Cycle_Init_OC1(){
    //TOP = (F_CPU/(PRSCL*fPWM)) - 1, com PRSCL DE 8 e fPWM = 50hz
    ICR1 = 39999; //Configura o periodo do PWM(20ms)

    TCCR1A = 0b10100011; // PWM rapido não invertido via ICR1
    TCCR1B = 0b00000011; //Liga TC0, presclar: 8
    OCR1A = 60; //Controle do ciclo ativo do PWM 0C1A(PB1) 5% = 0 graus = 30us
    OCR1B = 60; //Controle do ciclo ativo do PWM OC1B(PB2) 10% = 180 graus = 10us
}

void PWM_Cycle_Control_OC1(uint8_t A_Channel, uint8_t B_Channel){
    OCR1A = A_Channel; //Controle do ciclo ativo do PWM 0C1A(PB1) 5% = 0 graus = 30us
    OCR1B = B_Channel; //Controle do ciclo ativo do PWM OC1B(PB2) 10% = 180 graus = 10us
}


void PWM_Cycle_Init_OC0(){
    //TOP = (F_CPU/(PRSCL*fPWM)) - 1, com PRSCL DE 8 e fPWM = 50hz
    ICR1 = 39999; //Configura o periodo do PWM(20ms)

    TCCR0A = 0b10100011; // PWM rapido não invertido via ICR1
    TCCR0B = 0b00000011; //Liga TC0, presclar: 8
    OCR0A = 60; //Controle do ciclo ativo do PWM 0C1A(PB1) 5% = 0 graus = 30us
    OCR0B = 60; //Controle do ciclo ativo do PWM OC1B(PB2) 10% = 180 graus = 10us
}

void PWM_Cycle_Control_OC0(uint8_t A_Channel, uint8_t B_Channel){
    OCR0A = A_Channel; //Controle do ciclo ativo do PWM 0C1A(PB1) 5% = 0 graus = 30us
    OCR0B = B_Channel; //Controle do ciclo ativo do PWM OC1B(PB2) 10% = 180 graus = 10us
}

void External_Interrupt(uint8_t interruptNum) {
    switch(interruptNum){
        case 0: //PD2
            EICRA |= (1 << ISC01);
            EICRA &= ~(1 << ISC00);
            EIMSK |= (1 << INT0);
            break;
        case 1: //PD3
            EICRA |= (1 << ISC11);
            EICRA &= ~(1 << ISC10);
            EIMSK |= (1 << INT1);
            break;
        default:
            break;
    }
}

void Timer_2_CTC_Init(){
    //Timer
    TCCR2A = 0b00000010; //Habilita modo CTC do TC2
    TCCR2B = 0b00000011; // Preescaler: 64
    OCR2A = 249; //Ajusta comparador para 249
    TIMSK2 = 0b00000010; // Habilita interrupção de comparação com OCR2A. Interrupção a cada 1ms
}

void USART_Init(unsigned int ubrr){
    UBRR0H = (unsigned char)(ubrr>>8);//Ajusta a taxa de transmssão
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //Habilita o transmisso e o receptor
    UCSR0C = (1<<USBS0)|(3<<UCSZ00); //Ajusta o formato do frame(StopBit: 2, DataBit: 8)
}

void USART_Transmit(unsigned char data){
    while(!(UCSR0A & (1<<UDRE0))); //Espera a limpeza do registrador de transmissão
    UDR0 = data; // Coloca o dado no registrador e envia.
}

void Serial_Println(unsigned char  * buffer){
    int i = 0;
    while(buffer[i]!= '\0'){
        USART_Transmit(buffer[i]);
        i++;
    }
    USART_Transmit('\n');
}

void Serial_Print(unsigned char  * buffer){
    int i = 0;
    while(buffer[i]!= '\0'){
        USART_Transmit(buffer[i]);
        i++;
    }
}

unsigned char USART_Receive(void){
    while(!(UCSR0A & (1<<RXC0))); //Espera o dado ser recebido
    return UDR0; //Lê o dado recebido e retorna
}




// Converte decimal para BCD
uint8_t decimal_to_bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

// Inicialização do TWI (I2C)
void TWI_Init() {
    TWSR = 0; // Prescaler = 1
    uint8_t prescaler = TWSR & ((1 << TWPS0) | (1 << TWPS1));
    uint8_t scaler = 1 << ((prescaler >> TWPS0) * 2);  // 4^x = 1<<(2x)
    TWBR = ((F_CPU / F_SCL) - 16) / (2 * scaler);      // Calcula o valor ideal
    TWCR = (1 << TWEN); // Habilita TWI
}

// Envia START + SLA+W
uint8_t TWI_Start(uint8_t Write_Address) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))); // Espera conclusão do START

    if ((TWSR & 0xF8) != TW_START) return 1; // Falha no START

    TWDR = Write_Address; // Envia endereço + bit de escrita
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    if ((TWSR & 0xF8) != TW_MT_SLA_ACK) return 2; // Falha no ACK de SLA+W

    return 0; // Sucesso
}

// Envia Repeated START + SLA+R
uint8_t TWI_Repeated_Start(uint8_t Read_Address) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    if ((TWSR & 0xF8) != TW_REP_START) return 1;

    TWDR = Read_Address; // Endereço + leitura
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    if ((TWSR & 0xF8) != TW_MR_SLA_ACK) return 2;

    return 0;
}

// Envia condição de parada
void TWI_Stop() {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while (TWCR & (1 << TWSTO)); // Aguarda fim do STOP
}

// Escreve um byte no barramento
uint8_t TWI_Write(uint8_t data) {
    TWDR = decimal_to_bcd(data); // Converte para BCD e escreve
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    if ((TWSR & 0xF8) != TW_MT_DATA_ACK) return 1;

    return 0;
}

// Lê com ACK (mais dados a seguir)
uint8_t TWI_Read_Ack() {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

// Lê com NACK (último byte)
uint8_t TWI_Read_Nack() {
    TWCR = (1 << TWINT) | (1 << TWEN); // Sem TWEA
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}


void TWI_Slave_Init(uint8_t address) {
    TWAR = address << 1; // Endereço I2C (7 bits)
    TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT); // Habilita TWI, ACK e limpa flag
}

uint8_t TWI_Slave_Read() {
    // Aguarda ser endereçado com SLA+W
    while (~(TWCR & (1 << TWINT)));

    if ((TWSR & 0xF8) == 0x60) { // SLA+W recebido
        TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT); // Pronto para receber dados
        while (~(TWCR & (1 << TWINT))); // Aguarda dado
        uint8_t received;
        if ((TWSR & 0xF8) == 0x80) { // Dado recebido e ACK enviado
            received = TWDR;
        }

        TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT); // Fica pronto para nova comunicação
        return received;
    }
}

void TWI_Slave_Write(uint8_t data) {
    while (!(TWCR & (1 << TWINT))); // Espera evento TWI

    if ((TWSR & 0xF8) == 0xA8) { // SLA+R recebido, ACK enviado
        TWDR = data; // Prepara dado para envio
        TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA); // Libera dado com ACK

        while (!(TWCR & (1 << TWINT))); // Espera mestre ler
    }

    // Prepara para nova comunicação
    TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT);
}

int Ultrassonic_Read(){
// Pulso de Trigger (10us)
  PORTB &= ~(1 << PB0);
  _delay_us(2);
  PORTB |= (1 << PB0);
  _delay_us(10);
  PORTB &= ~(1 << PB0);

  int distance = 0;
  unsigned long count = 0;
  unsigned long timeout = 30000;
  while (!(PINB & (1 << PB4))) {
    if (count > timeout) {
      Serial_Println("Timeout: echo não subiu");
      break;
    }
    count += 1;
  }
  unsigned long start = 0;

  // Espera Echo descer

  count = 0;
  while (PINB & (1 << PB4)) {
    if (count > timeout) {
      Serial_Println("Timeout: echo não caiu");
      break;
    }
    count += 1;
    start+= 1;
  }

  // Calcula distância
  unsigned long duration = start;
  distance = duration * 0.0343 / 2;
  return distance;
}