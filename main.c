//-------------------------------------------------------------
// Author      : Kim Mølholm Hejlesen
// Target AVR  : Atmega 328P
// Clock speed : 20 MHz external oscillator
// VCC         : 5V
//-------------------------------------------------------------

//-------------------------------------------------------------
// include
//-------------------------------------------------------------
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <inttypes.h>
#include "spi.h"
#include "lcd.h"

//-------------------------------------------------------------
// define
//-------------------------------------------------------------
#define F_CPU 20000000UL // 8 MHz clock
#define ADMUX_SETTINGS             0x60
#define BUTTON_DEBOUNCE_TIME       0.01   // debounce time
#define BUTTON_SAMPLE_FREQUENCY    200   // button input sample frequency
#define BUTTON_INTEGRATOR_MAXIMUM (BUTTON_DEBOUNCE_TIME * BUTTON_SAMPLE_FREQUENCY)

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU/(USART_BAUDRATE*16UL)))-1)

#define STEPPER_MOTOR_TARGET_SPEED  100
#define STEPPER_MOTOR_INITIAL_SPEED 200

//-------------------------------------------------------------
// globals
//-------------------------------------------------------------
uint16_t system_timer_counter = 0;
uint16_t adc_buffer[2];
//uint8_t button_input[1], button_output[1], button_integrator[1];
uint16_t step_array[3] = {0, 0, 0};


//-------------------------------------------------------------
// function prototypes
//-------------------------------------------------------------
void initialize(void);
void adc_init(void);
void system_timer_init(void);
void uart_init(void);
void pwm_init(void);
uint8_t  move_steps(uint8_t);
void start_step(void);
void stop_step(void);

//-------------------------------------------------------------
// interrupt services
//-------------------------------------------------------------
ISR(ADC_vect) // ADC interrupt
{
  
  uint8_t adc_reading = ADCH;

  switch (ADMUX) {
    case ADMUX_SETTINGS:
      OCR2B = adc_reading;
      ADMUX = ADMUX_SETTINGS;
      break;
    case ADMUX_SETTINGS + 1:
      adc_buffer[1] = adc_reading;
      ADMUX = ADMUX_SETTINGS;
      break;
    default:
      ADMUX = ADMUX_SETTINGS;
      break;
  } ADCSRA |= (1 << ADSC) ; // start new conversion

}

ISR (TIMER1_COMPA_vect) // stepper timer
{
  static uint16_t interrupt_counter = 0;
  static uint8_t  current_speed = 200;
  interrupt_counter++;
  PORTB &= ~(1 << PB1); // set step signal low to driver 
  
  if (interrupt_counter == current_speed)
  {
    PORTB |= (1 << PB1); // set step signal high to driver
    interrupt_counter = 0;
    
    
    if (STEPPER_MOTOR_TARGET_SPEED < current_speed)
    {
      current_speed--;
    }
  }
}

//-------------------------------------------------------------
// main
//-------------------------------------------------------------
int main(void)
{
  initialize();
  
  while (1) {
    
    //LCD_goto_XY(0, 0);
    //LCD_write_decimal(current_speed);
    //LCD_goto_XY(0, 1);
    //LCD_write_decimal(target_speed);
    
    
    //_delay_ms(5000);
    //stop_step();
    
    wdt_reset(); // reset watchdog timer
  }
  
  return 0; // end program
  
}

//-------------------------------------------------------------
// functions
//-------------------------------------------------------------
void initialize(void)
{
  
  cli();                 // disable interrupts
  
  DDRB   |= (1 << PB1);                     // PortB1
  
  SPI_master_init();     // initialize SPI communication
  //adc_init();            // initialize analog to digital converter
  LCD_init();            // initialize LCD display
  system_timer_init();   // initialize system timer, 1 ms
  //uart_init();           // initialize UART
  DDRD   |= (1 << PD4);
  wdt_enable(WDTO_1S);   // enable 1s watchdog timer
  
  sei();                 // enable interrupts
  
  //ADCSRA |= (1 << ADSC); // start ADC conversions
  
}

void adc_init(void)
{
  // Initialize ADC
  ADCSRA |= ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));   // Prescaler
  // REFS1 = 0 && REFS0 = 1 to use AVcc as referencer, ADLAR = 1 to left center adc reading (ADCH = 8 MSB)
  ADMUX  = ADMUX_SETTINGS;                         // REFS0 = 1
  //ADCSRB &= ~((1<<ADTS2)|(1<<ADTS1)|(1<<ADTS0));  // ADTS2 = 0 ADTS1 = 0 ADTS0 = 0   => ADC in free-running mode
  //ADCSRA |= (1<<ADATE);                           // Signal source, in this case is the free-running
  //ADCSRA |= (1<<ADEN);                            // Power up the ADC
  ADCSRA |= (1 << ADIE);
  ADCSRA |= (1 << ADEN);
  
}

void system_timer_init(void)
{
  // 10 us interrupt
  OCR1A   = 199;
  TCCR1A  = 0;                  
  TCCR1B |= (1 << WGM12); // timer compare mode
  TCCR1B |= (1 << CS10); // prescaler = 1, enables timer
  TIMSK1 |= (1 << OCIE1A);
}

void uart_init(void)
{
  // 9600 baud, 8 bit, 1 stop bit, parity disabled
  UCSR0B |= (1<<RXEN0)  | (1<<TXEN0);
  UCSR0C |= (1<<UCSZ00) | (1<<UCSZ01);
  UBRR0H  = (BAUD_PRESCALE >> 8);
  UBRR0L  = BAUD_PRESCALE;
}


uint8_t move_steps(uint8_t steps)
{
  return 0;
}

void start_step(void)
{
  TCCR1B |= (1 << CS10); // prescaler = 1, enables timer
}

void stop_step(void)
{
    TCCR1B &= ~(1 << CS10); // no prescaler, disables timer
}
