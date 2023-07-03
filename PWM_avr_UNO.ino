#include <LCD_I2C.h>
#include <avr/io.h>

volatile uint8_t flag = 0;
unsigned long prevtime = 0;
float data = 0;
int counter_on = 0, counter_off = 0;
const int freq = 5000;
const int konstanta = (int)(1.6e7 / (100.0 * freq));
int DTp = 50;   //Ingat kalau Duty Cycle output/input = 1 - DTp kalau pake PMOS
const int max_debounce_count = 2000;    //1250 = 5 ms
int debounce_count = 0;

LCD_I2C myLCD(0x27, 16, 2);


/*  ===Catatan Perhitungan Angka untuk Register OCR1A===

Frekuensi Arduino Nano adalah 16 MHz.
Bagi frekuensi tersebut dengan prescaler (atur dengan CS pada register TCCR) untuk memperoleh frekuensi maksimum yang dapat digunakan.
Bagi frekuensi maksimum tersebut dengan frekuensi yang diinginkan untuk memperoleh count yang akan digunakan.

Contoh:
Dengan prescaler 1024, misalkan diinginkan periode 1 ms (atau 1 kHz).
Frekuensi maksimum yang dapat digunakan adalah 16E6/1024 = 15625 Hz.
Count yang digunakan untuk memperoleh periode sebesar 1 ms adalah 15625/1000 = 15.625 atau _approximately_ 16 .
*/


void init_int(){
  //                            ==============================Bagian Internal Interrupt==============================
  // Mode CTC dengan batas atas berupa isi register OCR1A, WGM12 = 1, WGM13 = WGM11 = WGM10 = 0
  TCCR1A = 0;
  TCCR1B = (1 << WGM12);

  // Set prescaler ke 1, CS10 = 1, CS11 = CS12 = 0
  TCCR1B |= (1 << CS10);

  // Enable output compare A dengan macro OCIE1A, vektornya pake TIMER1 COMPA
  TIMSK1 = (1 << OCIE1A);

  // Inisialisasi counter, counter = (TCNT1H << 8) + TCNT1L
  TCNT1H = 0x0;
  TCNT1L = 0x0;

  // Set nilai TOP awal, nilai TOP atau isi register OCR1A = (OCR1AH << 8) + OCR1AL
  counter_off = konstanta * DTp - 1;
  OCR1AH = counter_off >> 8;
  OCR1AL = counter_off && 0xFF;

  // Enable global interrupt
  SREG = (1 << 7);

  //                            ==============================Bagian External Interrupt==============================
  // Set Rising Edge pada INT0 (ISC01 & ISC00) dan INT1 (ISC11 & ISC10)
  EICRA = (1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00);

  // Enable INT0 dan INT1
  EIMSK = (1 << INT1) | (1 << INT0);
}


ISR(INT0_vect){
  // Note: 97% sepertinya sudah HIGH semua (dilihat dari serial dengan sampling 25 us)
  if(debounce_count >= max_debounce_count){
    debounce_count = 0;
    if(DTp >= 95){
      DTp = 95;
    }
    else{
      DTp += 1;
    }    
  }
}

ISR(INT1_vect){
  // Note: 3% sepertinya sudah LOW semua (dilihat dari serial dengan smapling 25 us)
  if(debounce_count >= max_debounce_count){
    debounce_count = 0;
    if(DTp <= 5){
      DTp = 5;
    }
    else{
      DTp -= 1;
    }     
  }
}


ISR(TIMER1_COMPA_vect){
  if(debounce_count < max_debounce_count){
    debounce_count++;
  }
  counter_on = konstanta * (100 - DTp) - 1;
  counter_off = konstanta * DTp - 1;
  if(flag){
    // Mematikan LED
    flag = 0;
    PORTB = ~(1 << PB5);
    PORTD = ~(1 << PD4);

    // Set OCR1A
    OCR1AH = counter_on >> 8;
    OCR1AL = counter_on & 0xFF;
  }
  else{
    // Menyalakan LED
    flag = 1;
    PORTB = 1 << PB5;
    PORTD = 1 << PD4;

    // Set OCR1A
    OCR1AH = counter_off >> 8;
    OCR1AL = counter_off & 0xFF;
  }
}


void setup() {
  // Set pin RX (output), pin D4 (output), pin D3 (input), dan pin D2 (input)
  DDRD = 0b00010001;

  // Set pin Builtin LED (D13/PB5 menjadi output, sisanya dont care)
  DDRB = 0b00100000;

  // Set pin analog untuk pembacaan tegangan (A1/PC1 menjadi input, sisanya dont care)
  DDRC = 0x0;

  // Set kondisi awal pin D13 (atau PB5) menjadi LOW
  PORTD = ~(1 << PD4);
  PORTB = ~(1 << PB5);
  PORTC = ~(1 << PC1);

  // Setting PWM
  init_int();

  // Setting LCD
  myLCD.begin();
  //myLCD.backlight();

  Serial.begin(115200);
}


void loop() {
  // Komentar
  if(micros() - prevtime > 25){
    prevtime = micros();
    data = ((float)analogRead(A1)) * 5.0 / 1023.0;
    Serial.println(data, 4);
    myLCD.setCursor(0, 0);
    myLCD.print("DT: ");
    myLCD.print((float)(100 - DTp), 1);
    myLCD.print("%/");
    myLCD.print((float)(debounce_count), 1);

    myLCD.setCursor(0, 1);
    myLCD.print((float)counter_on, 1);
    myLCD.print("/");
    myLCD.print((float)counter_off, 1);
  }
}
