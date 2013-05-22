#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdlib.h>

#define CLOCK (1<<PB1)
#define DATA (1<<PB2)
#define BLANK (1<<PB5)
#define LATCH (1<<PB6)

#define NUM_TLC5947 5

#define ANGLE 120

volatile uint8_t need_to_run, set_white;
volatile uint8_t j;
volatile uint8_t buffer[120];
volatile uint8_t music_buffer[180];
volatile uint8_t translated_buffer[180];
volatile uint8_t last_rpm[3];
volatile uint8_t last_rpm_index = 0;
volatile uint8_t angle_choose = 0;

volatile uint8_t counter = 0;
volatile uint8_t music_buffer_ptr = 0;
volatile uint8_t transmission_flag = 0;
volatile uint8_t init_seq[5] = {};

volatile uint8_t display_flag = 0;

inline void send_led(uint8_t a) {
    SPDR = a;
    // czekaj az sie wysle bufor
    while(!(SPSR & (1<<SPIF)));
}

inline void commit() {
    PORTB |= LATCH | BLANK;
    PORTB &= ~(LATCH | BLANK);
}

inline uint16_t translate(uint8_t a) {
    return (a * 4095) / 255;
}

void send_debug_number_16bit( uint16_t num){
    uint16_t i;
    char num_s[6];
    ultoa(num, num_s, 10);
    for (i=0; i<strlen(num_s); ++i) {
        USART_Transmit(num_s[i]);
    }
    USART_Transmit('\n');
    USART_Transmit('\r');
}

void send_debug_number( uint8_t num) {
    char num_s[4];
    itoa(num, num_s, 10);
    for (uint8_t i=0; i<strlen(num_s); ++i) {
        USART_Transmit(num_s[i]);
    }
    USART_Transmit('\n');
    USART_Transmit('\r');
    
}
void USART_Transmit( uint8_t data ) {
    while ( !( UCSR3A & (1<< UDRE3 )))    ;
    UDR3  = data;
}
uint8_t USART_Receive() {
  while ( !( UCSR3A & (1<< RXC3 )) );
  return UDR3 ;
}

void send_text(const char* text){
    for(uint8_t i=0; i<strlen(text); ++i)
        USART_Transmit(text[i]);
    USART_Transmit('\n');
    USART_Transmit('\r');
}  

inline uint8_t calc_rpm_avg(){
    return (last_rpm[0] + last_rpm[1] + last_rpm[2]) / 3 ;
}

void send_translate() {
    
    static uint8_t mode = 1, to_send = 0;
    uint16_t tmp=0;
    uint8_t i=0;
    
    for (i = 0; i < 120; i++) {
        tmp = translate(buffer[i]);
        // wysylka prawych 8 bitow
        if (mode) {
            send_led((uint8_t) (tmp >> 4));
            
            to_send = ((tmp << 12) >> 8);
        } else {
            to_send |= tmp >> 8;
            
            send_led(to_send);
            
            send_led((tmp << 8) >> 8);
        }
        mode = !mode;
    }
}

void send_translated_buffer(uint8_t* t_buffer){
    uint8_t i;
    for (i=0; i<180; ++i) {
        send_led(t_buffer[i]);
    }
}

void translate_buffer(uint8_t* buffer) {
    static uint8_t mode = 1, to_translate = 0;
    uint16_t tmp=0;
    uint8_t i=0, j=0;

    for (i=0, j=0; i < 120; i++, j++) {
        tmp = translate(buffer[i]);
        // wysylka prawych 8 bitow
        if (mode) {
            translated_buffer[j] = (uint8_t) (tmp >> 4);
            to_translate = ((tmp << 12) >> 8);
        } else {
            to_translate |= tmp >> 8;
            translated_buffer[j] = to_translate;
            ++j;
            translated_buffer[j] = ((tmp << 8) >> 8);
        }
        
        mode = !mode;
    }
}

ISR(INT7_vect) {
    last_rpm[last_rpm_index]++;
    TCNT2 = 0;
}

ISR(TIMER2_COMPA_vect) {
    uint8_t i;
    for (i=0; i<180; ++i) {
        if (angle_choose == 0)
            send_led(255);
        else
            send_led(0);
    }
    commit();
    angle_choose = !angle_choose;
}

ISR(USART3_RX_vect) {

    char read_byte;
    read_byte = UDR3;

    init_seq[counter] = read_byte;

    if (!transmission_flag) {
        uint8_t escape_flag = 0;
        for( uint8_t i=0; i<5; ++i){
            if (init_seq[i] != 0xFF){
                escape_flag = 1;
                break;
            }
        }
        if (!escape_flag)    
            transmission_flag = 1;
    } else {

        music_buffer[music_buffer_ptr] = read_byte;
        music_buffer_ptr++;
        if (music_buffer_ptr == 180) {
            music_buffer_ptr = 0;
            transmission_flag = 0;// ending transmission
            display_flag = 1;
        }
    }

    counter++;
    counter %= 5;
}

/*
ISR(USART3_RX_vect) {
    char read_byte;
    read_byte = UDR3;
    display_flag = 1;
}
*/
int main(void)
{
    sei();
    
    UCSR3B = (1<< RXEN3) | (1<< TXEN3 ) | (1<<RXCIE1);// | (1<<TXCIE1); //0x18;      //reciever enable , transmitter enable
    UBRR3H = 0;
    UBRR3L = 8;
   
    // outputy + SS
    DDRB |= (CLOCK | DATA | BLANK | LATCH | (1<<PB0));
    // stan niski
    PORTB &= ~(CLOCK | DATA | BLANK | LATCH);
    
    // SPI
    SPCR = (1<<SPE) | (1<<MSTR); //| (1<<DORD);
    // max predkoscq
    SPSR = (1<<SPI2X);
    
    // blank na wysoki, diody gasna
    PORTB |= BLANK;
    // czekamy dwie sekundy
    //_delay_ms(2000);
    // blank na niski
    PORTB &= ~BLANK;
    
    DDRD |= (1<<PD7);
    DDRJ &= ~(1<<PJ5 | 1<<PJ6);

    for (uint8_t i = 0; i < 180; i++)
        send_led(0);
    commit();


    while (1) {
        if(display_flag){
            for(uint8_t i=0; i<180; ++i) {
                send_led(music_buffer[i]);
            }
            commit();
            display_flag = 0;
        }  
    }

    uint8_t i;
     
    for (i=0; i < 120; i++) {
       buffer[i] = (i%3==0) ? 255 : 0;
    }
    
    translate_buffer(buffer);


    while(1){
      if (display_flag) {
          send_translated_buffer(translated_buffer);
          commit();
          
          send_text("Lights on");


          display_flag = 0;
          _delay_ms(200);

      } else {
          for (i = 0; i < 180; i++)
              send_led(0);
          commit();
      }


    }

    return 0;
}

