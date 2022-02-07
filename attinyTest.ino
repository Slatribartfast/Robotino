//Includes
#include <avr/io.h>
#include <avr/interrupt.h>

#define INTERRUPTPIN PCINT1 //this is PB1 per the schematic
#define PCINT_VECTOR PCINT0_vect  //this step is not necessary
#define DATADIRECTIONPIN DDB1 //Page 64 of data sheet
#define PORTPIN PB1 //Page 64
#define READPIN PINB1 //page 64


#define OUTPUT_PIN_A 3
#define OUTPUT_PIN_B 4
#define LED_PIN 2

#define ENCODER_PIN_B 0

/*
 * Alias for the ISR: "PCINT_VECTOR" (Note: There is only one PCINT ISR. 
 * PCINT0 in the name for the ISR was confusing to me at first, 
 * hence the Alias, but it's how the datasheet refers to it)
 */

volatile int8_t counter;

void setup() {
    cli();//disable interrupts during setup
        
    //pinMode
    DDRB |= (1 << OUTPUT_PIN_A);      
    DDRB |= (1 << OUTPUT_PIN_B);
    DDRB |= (1 << LED_PIN);
    DDRB &= ~(1 << ENCODER_PIN_B);

    PCMSK |= (1 << INTERRUPTPIN); //sbi(PCMSK,INTERRUPTPIN) also works but I think this is more clear // tell pin change mask to listen to pin2 /pb3 //SBI
    GIMSK |= (1 << PCIE);   // enable PCINT interrupt in the general interrupt mask //SBI

    DDRB &= ~(1 << DATADIRECTIONPIN); //cbi(DDRB, DATADIRECTIONPIN);//  set up as input  - pin2 clear bit  - set to zero
    PORTB |= (1<< PORTPIN); //cbi(PORTB, PORTPIN);// disable pull-up. hook up pulldown resistor. - set to zero
    sei(); //last line of setup - enable interrupts after setup

    PORTB |= (1 << LED_PIN);
    delay(200);
    PORTB &= ~(1 << LED_PIN);
}

void loop() {
  
  if(counter >= 16){
    PORTB |= (1 << OUTPUT_PIN_B);
    counter = counter - 16;
    delayMicroseconds(50);
    PORTB |= (1 << OUTPUT_PIN_A);
  }
  if(counter <= -16){
    PORTB &= ~(1 << OUTPUT_PIN_B);
    counter = counter + 16;
    delayMicroseconds(50);
    PORTB |= (1 << OUTPUT_PIN_A);
  }
  delayMicroseconds(150);
  PORTB &= ~(1 << OUTPUT_PIN_A);
}


//this is the interrupt handler
ISR(PCINT_VECTOR)
{
  if ((PINB& B00000010) == 0)
  {
    if((PINB& B00000001) == 0){
      counter++;
    }
    else{
      counter--;
    }
  }
}
