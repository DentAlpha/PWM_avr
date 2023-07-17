#include <avr/io.h>

volatile uint8_t flag = 0;
unsigned long prevtime = 0;


/*
Catatan:
Fast PWM dengan WGM[3..0] = 15, COM1A1 = 0, dan COM1A0 = 1 ==> Fast PWM Toggle Mode dengan DT fixed 50%
Frekuensi sinyal = 16e6/(2 * prescaler * (1 + OCR1A))
Untuk Fast PWM Non-inverted Mode:
Frekuensi sinyal = 16e6/(prescaler * (1 + OCR1A))
Persen DT = (OCR1B/OCR1A) * 100
*/


void init_timer(){
  // Mode Fast PWM dengan batas atas berupa isi register OCR1A, WGM13 = WGM12 = WGM11 = WGM10 = 1
  // Toggle OC1A
  TCCR1A = (1 << COM1A0) | (1 << WGM11) | (1 << WGM10);

  // Set prescaler ke 1, CS10 = 1, CS11 = CS12 = 0
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);

  // Enable output compare A dengan macro OCIE1A, vektornya pake TIMER1 COMPA
  //TIMSK1 = (1 << OCIE1A);

  // Set nilai TOP awal, nilai TOP atau isi register OCR1A = (OCR1AH << 8) + OCR1AL
  OCR1AH = 1599 >> 8;
  OCR1AL = 1599 & 0xFF;

  // Enable global interrupt
  //SREG = (1 << 7);
}


ISR(TIMER1_COMPA_vect);


void setup() {
  // Set pin RX (output)
  DDRD = 0b00000001;

  // Set pin D9/PB1 (output)
  DDRB = 0b00000010;

  // Set kondisi awal
  PORTD = (1 << PD0);
  PORTB = (0 << PB1);

  // Inisialisasi timer
  init_timer();
}

void loop() {
  // put your main code here, to run repeatedly:

}
