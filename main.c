/******************************************************************************
* Hochschule für Angewandte Wissenschaften Hamburg						      *
* Fakultät DMI															      *
* Department Medientechnik											 	      *
* Veranstaltung: Informatik & Elektronik                                      *
*******************************************************************************
* Schriftdarstellung auf dem TFT-Display:    							      *
* Das Display wird im Querformat 180° gedreht betrieben.    			      *
* Der Hintergrund wird weiß gefärbt. Es werden folgende Wörter in 3 Zeilen auf*
* dem Display dargestellt:                                                    *
* Media Systems                                                               *
* Informatik                                                                  *
* & Elektronik                                                                *
* Dipl.-Ing. M. Berens													      *
******************************************************************************/

#define F_CPU 16000000UL
#include <xc.h>
#include "spi.h"
#include "tft.h"
#include <avr/interrupt.h>

volatile uint16_t counter;
volatile uint16_t motorCountDown = 0;
volatile const uint16_t motorTimeS = 9;
volatile uint16_t timer0Counter = 0;
volatile const uint16_t timer0CounterMax = 60;
volatile const uint16_t motorPulseWidth = 48;

volatile char countDownText[] = "0";
volatile char onText[] = "on ";
volatile char offText[] = "off";
volatile char startText[] = "Start:\0";
volatile char motorText[] = "Motor:\0";


void updateDisplay() {
    PORTC = (0xF0 & PORTC) | (0x0F & motorCountDown);
}

void updateTFT() {
    
    //convert count down to ascii
    countDownText[0] = 48 + (char) motorCountDown;
    
    TFT_Print(countDownText, 120, 20, 2, TFT_16BitRed, TFT_16BitWhite, TFT_Landscape180);

    if(motorCountDown == 0) {
        TFT_Print(offText, 120, 60, 2, TFT_16BitRed, TFT_16BitWhite, TFT_Landscape180);
    } else {
        TFT_Print(onText, 120, 60, 2, TFT_16BitRed, TFT_16BitWhite, TFT_Landscape180);
    }
    
    TFT_Print(startText, 20, 20, 2, TFT_16BitBlue, TFT_16BitWhite, TFT_Landscape180);
    TFT_Print(motorText, 20, 60, 2, TFT_16BitBlue, TFT_16BitWhite, TFT_Landscape180);
}

ISR(PCINT0_vect) {
    motorCountDown = motorTimeS;
    timer0Counter = timer0CounterMax;
}

ISR(TIMER1_COMPA_vect){
	counter++;	
}

ISR(TIMER2_COMPA_vect){
    if(timer0Counter < timer0CounterMax) {
        timer0Counter++;
    } else {
        timer0Counter = 0;
        updateDisplay();
        updateTFT();
        if(motorCountDown > 0) {
            OCR0B = motorPulseWidth;
            if(motorCountDown == 1) {
                PORTB |= (1<<PORTB0);
            }
            motorCountDown--;
        } else {
            OCR0B = 0;
            PORTB &= ~(1<<PORTB0);
        }
        
    }
}

void Waitms(const uint16_t msWait){
	static uint16_t aktTime, diff;
	uint16_t countertemp;
	cli();              //da 16 bit Variablen könnte ohne cli() sei() sich der Wert
	aktTime = counter;  //von counter ändern, bevor er komplett aktTime zugewiesen wird.
	sei();              //Zuweisung erfolgt wg. 8 bit controller in 2 Schritten. 
	do {
			cli();
			countertemp = counter;
			sei();
			  diff = countertemp + ~aktTime + 1;
	  } while (diff	< msWait); 	
}

void PCINT1_init() {
    PCICR |= (1<<PCIE0);
    PCMSK0 |= (1<<PCINT1);
}

void Timer2_init(){
    TCCR2B |= (1<<CS22) | (1<<CS21) | (1<<CS20);    //prescalar 1024
    TIMSK2 |= (1<<OCIE2A);
    OCR2A = 157;                        //100Hz funktioniert noch nicht
}

void Timer1_init(){
	TCCR1B |= (1<<CS10) | (1<<WGM12);	// TimerCounter1ControlRegisterB Clock Select |(1<<CS10)=>prescaler = 1; WGM12=>CTC mode
	TIMSK1 |= (1<<OCIE1A);				// TimerCounter1 Interrupt Mask Register: Output Compare Overflow Interrupt Enable
	OCR1A = 15999;						// direkte Zahl macht Sinn; overflow register OCR1A berechnet mit division 64 => unlogischer Registerwert
}

void Display_init(void) {
	const uint16_t InitData[] ={
		//Initialisierungsdaten fuer 16 Bit Farben Modus
		0xFDFD, 0xFDFD,
		//pause
		0xEF00, 0xEE04, 0x1B04, 0xFEFE, 0xFEFE,
		0xEF90, 0x4A04, 0x7F3F, 0xEE04, 0x4306,
		//pause
		0xEF90, 0x0983, 0x0800, 0x0BAF, 0x0A00,
		0x0500, 0x0600, 0x0700, 0xEF00, 0xEE0C,
		0xEF90, 0x0080, 0xEFB0, 0x4902, 0xEF00,
		0x7F01, 0xE181, 0xE202, 0xE276, 0xE183,
		0x8001, 0xEF90, 0x0000,
		//pause
		0xEF08,	0x1806,	0x1200, 0x1583,	0x13AF,
		0x1600 	//Querformat um 180°gedreht; 132 x 176 Pixel
	};
	Waitms(300);
	PORTD &= ~(1<<Reset);	//Reset-Eingang des Displays auf Low => Beginn Hardware-Reset
	Waitms(75);
	PORTB |= (1<<CS);		//SSEL auf High
	Waitms(75);
	PORTD |= (1<<D_C);		//Data/Command auf High
	Waitms(75);
	PORTD |= (1<<Reset);	//Reset-Eingang des Displays auf High => Ende Hardware Reset
	Waitms(75);
	SendCommandSeq(&InitData[0], 2);
	Waitms(75);
	SendCommandSeq(&InitData[2], 10);
	Waitms(75);
	SendCommandSeq(&InitData[12], 23);
	Waitms(75);
	SendCommandSeq(&InitData[35], 6);
}

void PWM_init() {
    TCCR0B |= (1<<CS01) | (1<<WGM02);
    TCCR0A |= (1<<WGM00) | (1<<COM0B1);
    OCR0A = 80;
    OCR0B = motorPulseWidth;
}

int main(void){
    uint16_t i;
    DDRB &= ~(1<<DDB1);
    PORTB |= (1<<PORTB1);
    
    
	DDRD |= (1<<D_C)|(1<<Reset);		//output: PD2 -> Data/Command; PD3 -> Reset
    DDRD |= (1<<DDD5);                  //Pin D5 -> ouput (PWM for motor)
    DDRB |= (1<<DDB0);
    DDRC |= (1<<DDC0) | (1<<DDC1) | (1<<DDC2) | (1<<DDC3);
	Timer1_init();
	SPI_init();
	sei();
	Display_init();
    PWM_init();
    Timer2_init(); 
    PCINT1_init();
    
	//Display-Hintergrund weiß "färben"
	for(i=0; i<23232; i++){
		SPISend8Bit(0xFF);
		SPISend8Bit(0xFF);
	}
    
    updateTFT();
    
    while(1){;}
}