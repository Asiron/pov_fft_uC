#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdlib.h>

// Definy portów I/O - podłaczenia diod do mikroprocesora
#define CLOCK (1<<PB1)
#define DATA (1<<PB2)
#define BLANK (1<<PB5)
#define LATCH (1<<PB6)

#define NUM_TLC5947 5

#define scale 180/40

volatile uint8_t buffer[120];
volatile uint8_t music_buffer[180];
volatile uint8_t translated_buffer[180];

volatile uint8_t counter = 0;
volatile uint8_t music_buffer_ptr = 0;
volatile uint8_t transmission_flag = 0;
volatile uint8_t init_seq[5] = {};

volatile uint8_t display_flag = 0;

volatile uint8_t new_buffer[40];
volatile uint8_t new_buffer_ptr = 0;

// Wyslanie pojedynczego bajtu po interfejsie SPI
inline void send_led(uint8_t a) {
    SPDR = a;
    // czekaj az sie wysle bufor
    while(!(SPSR & (1<<SPIF)));
}

// Commit - zatwierdzenie wyslania bajtow na diody
inline void commit() {
    PORTB |= LATCH ;
    PORTB &= ~(LATCH );
}

// Interpolacja liniowa z 8 bitowego koloru do 12 bitowego
inline uint16_t translate(uint8_t a) {
    return (a * 4095) / 255;
}

// Procedura wyslania 16 bitowego numeru po UARTcie w sposob przyjazny dla czlowieka
// Zamien liczbę na stringa i wyślij po koleji, zakoncz znakiem nowej linii i powrotu karetki
void send_debug_number_16bit( uint16_t num){
    char num_s[6];
    ultoa(num, num_s, 10);
    for (uint8_t i=0; i<strlen(num_s); ++i) {
        USART_Transmit(num_s[i]);
    }
    USART_Transmit('\n');
    USART_Transmit('\r');
}

// Procedura wysłania 8 bitowego numeru po UARTcie w sposób przyjazny dla człowieka
// Zamien liczbę na stringa i wyślij po koleji, zakoncz znakiem nowej linii i powrotu karetki
void send_debug_number( uint8_t num) {
    char num_s[4];
    itoa(num, num_s, 10);
    for (uint8_t i=0; i<strlen(num_s); ++i) {
        USART_Transmit(num_s[i]);
    }
    USART_Transmit('\n');
    USART_Transmit('\r');
    
}
// Procedura wysłania po UARTcie - implementacja z aktywnym czekaniem
void USART_Transmit( uint8_t data ) {
    while ( !( UCSR3A & (1<< UDRE3 )))    ;
    UDR3  = data;
}
// Procedura odebrania po UARTcie - implementacja z aktywnym czekaniem
uint8_t USART_Receive() {
  while ( !( UCSR3A & (1<< RXC3 )) );
  return UDR3 ;
}

// Procedura wysłania tekstu po UARTcie 
// Wyslij tekst, znak po znaku i zakoncz znakiem nowej linii i powrotu karetki 
void send_text(const char* text){
    for(uint8_t i=0; i<strlen(text); ++i)
        USART_Transmit(text[i]);
    USART_Transmit('\n');
    USART_Transmit('\r');
}  

// Wyslanie nie przeksztalconego bufora
// Zamiast 180 bajtow przekazujemy 120 bajtow ( 40 diod x 8 bitow)
// I interpolujemy liniowo do 40 diod x 12 bitow = 180 bajtow
// poniewaz PWM sterownikow są 12 bitowe
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

//Wysylka juz przetlumaczonego bufora 
void send_translated_buffer(uint8_t* t_buffer){
    uint8_t i;
    for (i=0; i<180; ++i) {
        send_led(t_buffer[i]);
    }
}

// Translacja samego bufora bez wysylki 
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
/*
// Procedura przerwania odbiornika UART
ISR(USART3_RX_vect) {

    // Zapisz odebrany bajt do zmiennej i tym samym opróżnij bufor
    char read_byte;
    read_byte = UDR3;

    //Zapisz zmienną do sekwencji inicjalizującej transmisje w poprawne miejsce
    init_seq[counter] = read_byte;


    // Nie ma transmisji w tym czasie - sprawdzamy init_seq
    if (!transmission_flag) {
        // Jesli 5 bajtow w init_seq zostalo ustawione na 255
        // czyli odebralismy 5 kolejnych 0xFF to inicjalizujemy transmisje
        // W przeciwnym razie nie robimy nic - czekamy na 5 kolejnych 0xFF
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
        // Transmisja własnie trwa
        // Zapisz odczytany bajt do bufora cyklicznego
        // Zwieksz indeks zapisu do bufora
        // Sprawdz czy nie dotarlismy do konca
        // Jesli tak to wyzeruj indeks, zakoncz transmisje, ustaw flag gotowosci do wyswietlenia danych
        
        music_buffer[music_buffer_ptr] = read_byte;
        music_buffer_ptr++;
        if (music_buffer_ptr == 180) {
            music_buffer_ptr = 0;  
            transmission_flag = 0; // ending transmission
            display_flag = 1;
        }
    }

    // Licznik zapisu danych do sekwencji inicjalizujacej - bufor cykliczny 5 bajtowy
    counter++;
    counter %= 5;
}
*/

ISR(USART3_RX_vect) {

    // Zapisz odebrany bajt do zmiennej i tym samym opróżnij bufor
    char read_byte;
    read_byte = UDR3;

    //Zapisz zmienną do sekwencji inicjalizującej transmisje w poprawne miejsce
    init_seq[counter] = read_byte;


    // Nie ma transmisji w tym czasie - sprawdzamy init_seq
    if (!transmission_flag) {
        // Jesli 5 bajtow w init_seq zostalo ustawione na 255
        // czyli odebralismy 5 kolejnych 0xFF to inicjalizujemy transmisje
        // W przeciwnym razie nie robimy nic - czekamy na 5 kolejnych 0xFF
        uint8_t escape_flag = 0;
        for( uint8_t i=0; i<5; ++i){
            if (init_seq[i] != 0xFE){
                escape_flag = 1;
                break;
            }
        }
        if (!escape_flag)    
            transmission_flag = 1;    
    } else {
        // Transmisja własnie trwa
        // Zapisz odczytany bajt do bufora cyklicznego
        // Zwieksz indeks zapisu do bufora
        // Sprawdz czy nie dotarlismy do konca
        // Jesli tak to wyzeruj indeks, zakoncz transmisje, ustaw flag gotowosci do wyswietlenia danych
        
        new_buffer[new_buffer_ptr] = read_byte;
        new_buffer_ptr++;
        if (new_buffer_ptr == 40) {
            new_buffer_ptr = 0;  
            transmission_flag = 0; // ending transmission
            display_flag = 1;
        }
    }

    // Licznik zapisu danych do sekwencji inicjalizujacej - bufor cykliczny 5 bajtowy
    counter++;
    counter %= 5;
}


int main(void)
{
    // Globalne właczenie przerwan
    sei();
    
    // Inicjalizacja UART'u, właczamy nadajnik i odbiornik oraz przerwanie odbiornika
    UCSR3B = (1<< RXEN3) | (1<< TXEN3 ) | (1<<RXCIE1);       //reciever enable , transmitter enable
    // Usart baud rate register - zsynchronizowanie UARTU do czestotliwosci mikroprocesora
    UBRR3H = 0;
    UBRR3L = 8;
   

    // Ustawienie kierunków portów I/O PB0 jest SS (Chip Select) domyslnie ustawiony
    DDRB |= (CLOCK | DATA | BLANK | LATCH | (1<<PB0));
    // stan niski
    PORTB &= ~(CLOCK | DATA | BLANK | LATCH);
    
    // Inicjalizacja interfacu - Serial Peripheral Interface
    SPCR = (1<<SPE) | (1<<MSTR); //| (1<<DORD);
    // max predkoscq
    SPSR = (1<<SPI2X);
    
    // Wyślij 'szpilkę' na BLANK'a, żeby zatwierdzic wysłane bajty po SPI,
    // i wyświetlic diody
    // blank na wysoki, diody gasna
    PORTB |= BLANK;
    // blank na niski
    PORTB &= ~BLANK;
    
    DDRD |= (1<<PD7);
    DDRJ &= ~(1<<PJ5 | 1<<PJ6);

    // Zgas wszystkie diody
    for (uint8_t i = 0; i < 180; i++)
        send_led(0);
    commit();

    while (1) {
        // Jesli bufor jest pełny to wyświetla zawartość bufora i zaneguj flage,
        // żeby nie wyświetlic jej jeszcze raz, czekaj ponownie na flage bufora
        if(display_flag){
            /*
            // wyświetl wszystkie diody
            for(uint8_t i=0; i<180; ++i) {
                send_led(music_buffer[i]);
            }
            commit();
            display_flag = 0;
            */

            for(uint8_t i=0; i<40; ++i) {
                for(uint8_t j=0; j < 180 - scale*new_buffer[i]; ++j){
                    send_led(0);
                }    
                for(uint8_t j=0; j < scale*new_buffer[i]; ++j){
                    send_led(255);
                }
                commit();
                _delay_ms(20);
            }
            display_flag = 0;
        }  
    }

    return 0;
}

