/*
 * main.c
 *
 *  Created on: 25 maj 2021
 *      Author: Jakub Szymkowiak
 */
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/sfr_defs.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "HD44780.h"


#ifndef _BV
#define _BV(bit)				(1<<(bit))
#endif

#ifndef inb
#define	inb(addr)			(addr)
#endif

#ifndef outb
#define	outb(addr, data)	addr = (data)
#endif

#ifndef sbi
#define sbi(reg,bit)		reg |= (_BV(bit))
#endif

#ifndef cbi
#define cbi(reg,bit)		reg &= ~(_BV(bit))
#endif

#ifndef tbi
#define tbi(reg,bit)		reg ^= (_BV(bit))
#endif

/*
 *  Gotowe zaimplementowane:
 #define 	bit_is_set(sfr, bit)   (_SFR_BYTE(sfr) & _BV(bit))
 #define 	bit_is_clear(sfr, bit)   (!(_SFR_BYTE(sfr) & _BV(bit)))
 #define 	loop_until_bit_is_set(sfr, bit)   do { } while (bit_is_clear(sfr, bit))
 #define 	loop_until_bit_is_clear(sfr, bit)   do { } while (bit_is_set(sfr, bit))

 */

// MIN/MAX/ABS macros
#define MIN(a,b)			((a<b)?(a):(b))
#define MAX(a,b)			((a>b)?(a):(b))
#define ABS(x)				((x>0)?(x):(-x))

/*
 * Tablica przechowuje zakodowane w systemie dwójkowym cyfry,
 * dziêki czemu mo¿na je wyœwietlic na wyœwietlaczu siedmiosegmentowym
 */
char cyfra[11] = { 0b1111110, 0b0110000, 0b1101101, 0b1111001, 0b0110011,
		0b1011011, 0b1011111, 0b1110000, 0b1111111, 0b1111011, 0b0000001 };
volatile uint16_t hour;   //godzina
volatile uint16_t minute;  //minuta
volatile uint16_t second;  //sekunda
volatile uint16_t decisecond;  //0,1 sekundy
char text[20];                  //bufor do wyswietlania tekstu na LCD
volatile uint16_t count=0;      //licznik przydatny w funkcjach stopera i zegara

/*
 * Zmienne pomocnicze s³u¿¹ce do obs³ugi logiki programu
 * stop - zazwyczaj s³u¿y do zatrzymania akcji bez wychodzenia z funkcji
 * end - sygnalizuje koniec dzia³ania funkcji
 * start - sygnalizuje pocz¹tek realizacji zadania (stoper)
 */
volatile uint8_t stop=0, end=0, start=0;
volatile uint8_t modeClock=0; //tryb pracy 'zegar'
volatile uint8_t modeStopwatch=0; //tryb pracy 'stoper'

/*
 * Funkcja konfiguruje przetwornik A/D
 * Ustawia odpowiednie rejestry
 */
void ADC_init(){
	//konfiguracja napiêcia referencyjnego - wybrano AVCC
	sbi(ADMUX, REFS0);
	//konfiguracja dzielnika czestotliwosci - mniej niz 100kHz
	sbi(ADCSRA, ADPS2);
	//uruchomienie uk³adu przetwornika
	sbi(ADCSRA, ADEN);
}

/*
 * Funkcja mierzy napiêcie i zwraca wartoœc rejestru ADC
 */
uint16_t ADC_10bit(){
	//uruchomienie pojedynczego pomiaru
	sbi(ADCSRA, ADSC);
	//oczekiwanie na wyzerowanie bitu - zakonczenie pomiaru
	do{}while(bit_is_clear(ADCSRA,ADSC));
	//zwraca wartosc w rejestrze ADC
	return ADC;
}

/*
 * Funkcja przelicza wartosc z rejestru ADC na wolty
 * Zwraca zmienn¹ typu float - napiêcie w woltach
 */
float ADC_measure(uint16_t result){
	//zwraca wartosc przeliczon¹ na wolty
	return 0.49*result;
}//Inicjalizacja Timer1 do wywolywania przerwania z czêstotliwoœci¹ 2Hz

/*
 * Funkcja wywo³uje przerwanie odpowiedni¹ czêstotliwosci¹
 * W tym przypadku jest to 10Hz
 */
void TimerInit() {
	//Wybranie trybu pracy CTC z TOP OCR1A
	sbi(TCCR1B, WGM12);
	//Wybranie dzielnika czestotliwosci
	sbi(TCCR1B, CS11);
	//Zapisanie do OCR1A wartosci odpowiadajacej 0,1s
	OCR1A = 12500;
	//Uruchomienie przerwania OCIE1A
	sbi(TIMSK, OCIE1A);
}

/*
 * Inicjalizacja portow do obs³ugi wyswietlacza 7 segmentowego
 */
void seg7Init() {
	//Inicjalizacja segmentu
	DDRC = 0xFF;
}

/*
 * Funkcja wy³¹cza wyœwietlacz siedmiosegmentowy
 */
void seg7Off(){
	//Wy³aczenie segmentu
	DDRC=0x00;
}

/*
 * Wyswietla na wyswietlaczu 7 segmentowym cyfre z argumentu
 */
void seg7ShowCyfra(uint8_t cyfraDoWyswietlenia) {
	//W rejestrze PORTX ustawiane s¹ bity na odpowiednie wartosci
	PORTC = cyfra[cyfraDoWyswietlenia];
}

/*
 * Funkcja sprawdza, czy liczba jest parzysta
 * Zwraca 1 - prawda lub 0 - fa³sz
 */
int even(uint16_t number){
	if(number%2){
		return 1; //jezeli parzysta
	}
	return 0; //jezeli nieparzysta
}

/*
 * Funkcja sprawdza, czy liczba jest pierwsza
 * Zwraca 1 - prawda lub 0 - fa³sz
 */
int prime(uint16_t number){
	//liczby 0 i 1 nie sa pierwsze
	if(number==0 || number==1){
		return 0;
	}
	//liczba 2 - najmniejsza liczba pierwsza
	if(number==2){
		return 1;
	}
	//wystarczy sprawdzic, czy reszta z dzielenia liczby przez liczbe mniejsza/równ¹ pierwiastkowi jest równa 0
	int sq = sqrt(number);
	for(int i=2;i<=sq; i++){
		if(number%i==0){ return 0;}
	}
	return 1; //jezeli nie wykryto dzielników - liczba jest pierwsza
}

/*
 * Funkcja ustawia przyciski jako wejœcia
 * oraz ustawia wartosci w rejestrze PORTX
 * Tak by by³y wrazliwe na nacisniecie */

void buttonsInit(){
	//ustawienie przycisków jako wejœcie
	cbi(DDRA, PA1);
	cbi(DDRA, PA2);
	cbi(DDRA, PA3);
	cbi(DDRA, PA4);
	//uwra¿liwienie przycisków na nacisniecie
	sbi(PORTA, PA1);
	sbi(PORTA, PA2);
	sbi(PORTA, PA3);
	sbi(PORTA, PA4);
}

/*
 * Funkcja ustawia diody jako wyjscia
 */
void diodesInit(){
	//ustawienie diód jako wyjscia
	sbi(DDRD, PD5);
	sbi(DDRD, PD4);
	sbi(DDRD, PD3);
}

/*
 * Funkcja wy³acza wszystkie diody,
 * jezeli ktoras jest w³aczona
 */
void diodesOff(){
	//wy³¹czenie diód
	cbi(PORTD, PD5);
	cbi(PORTD, PD4);
	cbi(PORTD, PD3);
}

/*
 * Funkcja realizuje zadanie 'Liczby'
 */
void numbers(){
	seg7Init(); //inicjalizacja wyswietlacza 7 segmentowego
	uint16_t number=0; //inicjalizacja zmiennej
	while(!stop){
		LCD_GoTo(0,0);
		sprintf(text, "%d", number);
		LCD_WriteText(text); //wyswietlenie aktualnej liczby na LCD
		if(number<10){
			seg7ShowCyfra(number); //wyswietlenie liczby na 7 segmentowym
		}else{
			seg7ShowCyfra(10); //wyswietlenie znaku '-'
		}
		//Jezeli nacisnieto UP liczba rosnie o 1
		if(bit_is_clear(PINA, PA1)){
			number++;
			_delay_ms(500);
		}
		//Jezeli nacisnieto DOWN liczba maleje o 1
		if(bit_is_clear(PINA, PA2)){
			//Przypadek, gdy przechodzimy z 10 na 9
			if(number==10){
				LCD_Clear();
			}
			number--;
			_delay_ms(500);
		}
		//Wyswietlac mozna liczby w zakresie 0-50
		if(number>50){
			LCD_Clear();
			number=0;
		}
		//Sprawdzenie, czy parzysta
		//Jezeli tak - swieci sie odpowiednia dioda
		if(even(number)){
			sbi(PORTD, PD3);
			cbi(PORTD, PD4);
			cbi(PORTD, PD5);
		}else{
			//Jezeli nieparzysta:
			sbi(PORTD, PD4);
			cbi(PORTD, PD3);
		}
		//Jezeli pierwsza:
		if(prime(number)){
			sbi(PORTD, PD5);
		}else{
			cbi(PORTD, PD5);
		}
		//Jezeli uzytkownik nacisnie X nastepuje koniec dzialania funkcji
		if(bit_is_clear(PINA, PA4)){
			stop=1;
			_delay_ms(500);
		}
	}
	LCD_Clear(); //czyszczenie LCD
	seg7Off();   //wy³¹czenie 7 segmentowego
	diodesOff(); //wy³¹czenie diód
}

/*
 * Funkcja realizuje zadanie miernika napiêcia
 */
void measure(){
	ADC_init(); //inicjalizacja przetwornika i miernika
	uint16_t result; //wynik z rejestru ADC
	float voltage; //napiecie w woltach
	stop=0;
	while(!stop){
		LCD_GoTo(0,0);
		result = ADC_10bit(); //pomiar napiecia
		voltage = ADC_measure(result)/100;  //przeliczenie na wolty
		int tempInt1 = voltage; //zmienna pomocnicza
		float tempFr = voltage-tempInt1; //zmienna pomocnicza
		int tempInt2 = trunc(tempFr*100); //zmienna pomocnicza
		LCD_GoTo(0,0);
		sprintf(text, "%d", result);
		LCD_WriteText(text);
		LCD_GoTo(0,1);
		sprintf(text, "U =  %d.%02d V", tempInt1, tempInt2); //wyswietlenie w odpowiedniej postaci
		LCD_WriteText(text);
		//Jezeli nacisnieto X funkcja konczy swoje dzialanie
		if(bit_is_clear(PINA, PA4)){
			stop=1;
			_delay_ms(500);
		}
	}
	LCD_Clear(); //czyszczenie LCD
}

/*
 * Funkcja przerwania wewnetrznego
 * Obs³uguje odliczanie czasu
 * Mozliwe dwa tryby pracy - zegar lub stoper
 */
ISR(TIMER1_COMPA_vect){
	/*
	 * Dopóki stop!=1
	 * count%10 - licznik pracuje na 10Hz, wiêc konieczne jest przeliczenie do 1 Hz
	 * modeClock - tryb pracy 'zegar'
	 */
	if(!stop && count%10==0 && modeClock){
			LCD_GoTo(0, 0);
			//wyswietlenie godziny
			if(hour<10){
				sprintf(text,"0%d",hour);
				LCD_WriteText(text);
			}else{
				sprintf(text,"%d",hour);
				LCD_WriteText(text);
			}
			//wyswietlenie minuty
			LCD_WriteText(":");
			if(minute<10){
				sprintf(text,"0%d",minute);
				LCD_WriteText(text);
			}else{
				sprintf(text,"%d",minute);
				LCD_WriteText(text);
			}
			//wyswietlenie sekundy
			LCD_WriteText(":");
			if(second<10){
				sprintf(text,"0%d", second);
				LCD_WriteText(text);
				seg7ShowCyfra(second);
			}else{
				sprintf(text,"%d",second);
				LCD_WriteText(text);
				seg7ShowCyfra(10);
			}
			second++;
			//zmiana minuty
			if(second==60){
				second=0;
				minute++;
			}
			//zmiana godziny
			if(minute==60){
				minute=0;
				hour++;
			}
			//nowy dzien
			if(hour==24){
				hour=0;
			}
	}
	/*
	 * tryb pracy: Stoper
	 * Dopóki stop != 1
	 */
	if(!stop && modeStopwatch){
		//zmiana stanu diody co sekunde
		if(count%10==0){
			tbi(PORTD, PD3);
		}
		LCD_GoTo(0, 0);
		//wyswietlenie liczby minut
		if(minute<10){
		sprintf(text,"0%d",minute);
		LCD_WriteText(text);
		}else{
			sprintf(text,"%d",minute);
			LCD_WriteText(text);
		}
		//wyswietlenie liczby sekund
		LCD_WriteText(":");
		if(second<10){
			sprintf(text,"0%d",second);
			LCD_WriteText(text);
		}else{
			sprintf(text,"%d",second);
			LCD_WriteText(text);
		}
		//wyswietlenie liczby decysekund
		LCD_WriteText(".");
		sprintf(text,"%d", decisecond);
		LCD_WriteText(text);
		decisecond++;
		//zmiana sekundy
		if(decisecond==10){
		decisecond=0;
		second++;
		}
		//zmiana minuty
		if(second==60){
			second=0;
			minute++;
		}
	}
	count++;
}

/*
 * Realizuje zadanie 'Zegar'
 */
void clock(){
	seg7Init(); //inicjalizacja 7 segmentowego
	hour=12; //godzina 12
	minute=0; //wyzerowanie minut
	second=0; //wyzerowanie sekund
	count=0; //licznik
	modeClock=1; //tryb pracy: Zegar
	stop=0;
	sei();  //uruchomienie przerwania
	while(!stop){
		//oczekiwanie na nacisniecie X
		if(bit_is_clear(PINA, PA4)){
			stop=1;
			_delay_ms(500);
		}
		//zapalanie diody co sekunde na 200ms
		if(count%10==0){
			tbi(PORTD, PD5);
			_delay_ms(200);
			tbi(PORTD, PD5);
		}
	}
	modeClock=0; //wyzerowanie trybu pracy
	LCD_Clear(); //czyszczenie LCD
	seg7Off();  //wy³¹czenie 7 segmentowego
	diodesOff(); //wy³aczenie diód
}

/*
 * Realizuje zadanie 'Stoper'
 */
void stopwatch(){
	LCD_WriteText("00:00.0"); //wyswietla wyzerowany czas
	start=0;
	//oczekiwanie na nacisniecie OK
	while(!start){
		if(bit_is_clear(PINA, PA3)){
			start=1;
			_delay_ms(200);
		}
	}
	minute=0;
	second=0;
	decisecond=0;
	modeStopwatch=1;  //tryb pracy: Stoper
	stop=0; end=0;
	sei();   //uruchomienie przerwania
	//oczekiwanie na nacisniecie OK - zatrzymanie stopera
	while(!stop){
		if(bit_is_clear(PINA, PA3)){
			stop=1;
		}
	}
	modeStopwatch=0; //wyzerowanie trybu pracy
	//oczekiwanie na nacisniecie X - wy³aczenie stopera
	while(!end){
		if(bit_is_clear(PINA, PA4)){
			end=1;
		}
	}
	LCD_Clear(); //czyszczenie LCD
	diodesOff(); //wy³¹czenie diód
}

/*
 * Wyswietla komunikat powitalny
 */
/*
 * Funkcja wyswietla komunikat powitalny
 */
void info(){
	LCD_GoTo(0,0);
	LCD_WriteText("Projekt PTM 2021");
	LCD_GoTo(0,1);
	LCD_WriteText("252868");
	_delay_ms(4000);
	LCD_Clear();
}

/*
 * Program g³ówny
 */
int main(){
	TimerInit(); //licznik
	buttonsInit(); //przyciski
	diodesInit();  //diody
	LCD_Initalize(); //LCD
	LCD_Home();
	info();  //wyswietlenie komunikatu powitalnego
	uint8_t option=1;  //zmienna option - wybor uzytkownika
	while(1){
		LCD_GoTo(0,0);
		//W zaleznosci od uzytkownika wyswietlaja sie propozycje
		switch(option){
		case 1:
			LCD_WriteText("1. Liczby");
			//oczekiwanie na wcisniecie OK
			if(bit_is_clear(PINA, PA3)){
				_delay_ms(500);
				LCD_Clear();
				numbers();
			}
			break;
		case 2:
			LCD_WriteText("2. Stoper");
			//oczekiwanie na wcisniecie OK
			if(bit_is_clear(PINA, PA3)){
				_delay_ms(500);
				LCD_Clear();
				stopwatch();
			}
			break;
		case 3:
			LCD_WriteText("3. Zegar");
			//oczekiwanie na wcisniecie OK
			if(bit_is_clear(PINA, PA3)){
				_delay_ms(500);
				LCD_Clear();
				clock();
			}
			break;
		case 4:
			LCD_WriteText("4. Miernik");
			//oczekiwanie na wcisniecie OK
			if(bit_is_clear(PINA, PA3)){
				_delay_ms(500);
				LCD_Clear();
				measure();
			}
		}
		//jezeli UP to jedŸ w górê
		if(bit_is_clear(PINA, PA1)){
			_delay_ms(500);
			option++;
			LCD_Clear();
		}
		//jezeli DOWN to jedŸ w dó³
		if(bit_is_clear(PINA, PA2)){
			_delay_ms(500);
			option--;
			LCD_Clear();
		}
		//maksymalny numer opcji to 4
		if(option==5){
			option=1;
		}
		//minimalny numer opcji to 1
		if(option==0){
			option=4;
		}
	}

}
