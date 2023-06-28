#include <avr/io.h>

volatile uint8_t flag = 0;
unsigned long prevtime = 0;


/*  ===Catatan Perhitungan Angka untuk Register OCR1A===

Frekuensi Arduino Nano adalah 16 MHz.
Bagi frekuensi tersebut dengan prescaler (atur dengan CS pada register TCCR) untuk memperoleh frekuensi maksimum yang dapat digunakan.
Bagi frekuensi maksimum tersebut dengan frekuensi yang diinginkan untuk memperoleh count yang akan digunakan.

Contoh:
Dengan prescaler 1024, misalkan diinginkan periode 1 ms (atau 1 kHz).
Frekuensi maksimum yang dapat digunakan adalah 16E6/1024 = 15625 Hz.
Count yang digunakan untuk memperoleh periode sebesar 1 ms adalah 15625/1000 = 15.625 atau _approximately_ 16 .
*/


void init_timer(){
  // Mode CTC dengan batas atas berupa isi register OCR1A, WGM12 = 1, WGM13 = WGM11 = WGM10 = 0
  TCCR1A = 0;
  TCCR1B = (1 << WGM12);

  // Set prescaler ke 256, CS12 = 1, CS11 = CS10 = 0
  TCCR1B = (1 << CS12);

  // Enable output compare A dengan macro OCIE1A, vektornya pake TIMER1 COMPA
  TIMSK1 = (1 << OCIE1A);

  // Inisialisasi counter, counter = (TCNT1H << 8) + TCNT1L
  TCNT1H = 0x0;
  TCNT1L = 0x0;

  // Set nilai TOP awal yaitu 124 (2 ms), nilai TOP atau isi register OCR1A = (OCR1AH << 8) + OCR1AL
  OCR1AH = 0;
  OCR1AL = 0b01111100;

  // Enable global interrupt
  SREG = (1 << 7);
}


ISR(TIMER1_COMPA_vect){
  if(flag){
    // Mematikan LED
    flag = 0;
    PORTB &= ~(1 << PB5);

    // Set OCR1A ke 124 (2 ms)
    OCR1AH = 0;
    OCR1AL = 0b01111100;

  }
  else{
    // Menyalakan LED
    flag = 1;
    PORTB |= 1 << PB5;

    // Set OCR1A ke 124 (2 ms)
    OCR1AH = 0;
    OCR1AL = 0b01111100;
  }
}


void setup() {
  // Set pin D13 menjadi output
  DDRD = 0b00000001;
  DDRB = 0b00100000;

  // Set kondisi awal pin D13 (atau PB5) menjadi LOW
  PORTB = ~(1 << PB5);

  // Setting PWM
  init_timer();
}


void loop() {
  // Komentar
}
