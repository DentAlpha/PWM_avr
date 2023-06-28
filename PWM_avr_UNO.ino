#include <avr/io.h>

volatile uint8_t flag = 0;
unsigned long prevtime = 0;
float data = 0;


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

  // Set prescaler ke 8, CS11 = 1, CS11 = CS10 = 0
  TCCR1B |= (1 << CS11);

  // Enable output compare A dengan macro OCIE1A, vektornya pake TIMER1 COMPA
  TIMSK1 = (1 << OCIE1A);

  // Inisialisasi counter, counter = (TCNT1H << 8) + TCNT1L
  TCNT1H = 0x0;
  TCNT1L = 0x0;

  // Set nilai TOP awal yaitu 3999 (2 ms), nilai TOP atau isi register OCR1A = (OCR1AH << 8) + OCR1AL
  OCR1AH = 0b00001111;
  OCR1AL = 0b10011111;

  // Enable global interrupt
  SREG = (1 << 7);
}


ISR(TIMER1_COMPA_vect){
  if(flag){
    // Mematikan LED
    flag = 0;
    PORTB = ~(1 << PB5);
    PORTD = ~(1 << PD3);

    // Set OCR1A ke 3999 (2 ms)
    OCR1AH = 0b00001111;
    OCR1AL = 0b10011111;
  }
  else{
    // Menyalakan LED
    flag = 1;
    PORTB = 1 << PB5;
    PORTD = 1 << PD3;

    // Set OCR1A ke 3999 (2 ms)
    OCR1AH = 0b00001111;
    OCR1AL = 0b10011111;
  }
}


void setup() {
  // Set pin RX & pin D3
  DDRD = 0b00001001;

  // Set pin Builtin LED (D13/PB5 menjadi output, sisanya dont care)
  DDRB = 0b00100000;

  // Set pin analog untuk pembacaan tegangan (A1PC1 menjadi input, sisanya dont care)
  DDRC = 0x0;

  // Set kondisi awal pin D13 (atau PB5) menjadi LOW
  PORTD = ~(1 << PD3);
  PORTB = ~(1 << PB5);
  PORTC = ~(1 << PC1);

  // Setting PWM
  init_timer();

  Serial.begin(9600);
}


void loop() {
  // Komentar
  if(millis() - prevtime > 1){
    prevtime = millis();
    data = ((float)analogRead(A1)) * 5.0 / 1023.0;
    Serial.println(data, 4);
  }
}
