// include the library code:
#include "functions.h"
#include <ACS712.h> //Bibliotek innehållande kod för omvandling av data från ACS712
#include <LiquidCrystal.h> //Standardbibliotek för arduino inneållande drivrutiner för LCD-skärm.

float adcVoltage;
float sensValue;

float powerMoment;
float powerAvg;
float prevpowerAvg;
float powerTot;
bool iterator = false;

float timerValue;

uint16_t timer0_counter = 0;
uint32_t timer1_counter = 0;
int v1, v2, v3, v4, v5;
int* values[5] = { &v1,&v2,&v3,&v4,&v5 };

const uint8_t buttonPin = 10;
const uint8_t buttonvoltagePin = 11;
volatile uint8_t buttonState;


const uint8_t rs = 12, en = 9, d4 = 5, d5 = 4, d6 = 3, d7 = 2; //Sätter div. portar på LCDn.
ACS712 currentSensor(A0); //Initierar mätning på ACS712-sensor på pin A0 via konstruktor.
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); //Initierar LCDn via konstruktor. 

#define BUTTON_PRESSED digitalRead(buttonPin) == HIGH
#define BUTTON_NPRESSED digitalRead(buttonPin) == LOW
#define LCD_BACKL_ON digitalWrite(8,HIGH)
#define LCD_BACKL_OFF digitalWrite(8,LOW)
#define CURRENT_AMPS currentAvg>=1000
#define CURRENT_MAMPS currentAvg<1000
#define POWER_WATTS powerMoment >=1
#define POWER_MWATTS powerMoment <1

#define TIMER0_ELAPSED_MS 5000 // Timer 0 löper ut efter en minut.
#define TIMER0_MAXV (uint32_t)(TIMER0_ELAPSED_MS / 0.128 + 0.5) // Här definierar vi maxvärdet för Timer0.

#define TIMER1_ELAPSED_MS 30 // Timer 1 löper ut efter 30ms.
#define TIMER1_MAXV (uint16_t)(TIMER1_ELAPSED_MS / 0.128 + 0.5) // Här definierar vi maxvärdet för Timer1.

void setup() {

	Serial.begin(9600); //Initierar seriell kommunikation i 9600kbps.

	noInterrupts(); // disable all interrupts

	TCCR0B = (1 << CS01);
	TIMSK0 = (1 << TOIE0);

	TCCR1B = (1 << CS11) | (1 << WGM12);
	OCR1A = 256;
	TIMSK1 = (1 << OCIE1A);

	PCICR = (1 << PCIE0);
	PCMSK0 |= (1 << PCINT2);
	interrupts(); // enable all interrupts

	pinMode(A2, INPUT);
	pinMode(8, OUTPUT); //Ettställer pin 8 i DDRB-registret, som strömförser backlighten på LCDn.
	pinMode(buttonPin, INPUT_PULLUP);
	pinMode(buttonvoltagePin, OUTPUT);
	digitalWrite(buttonvoltagePin, HIGH);

	LCD_BACKL_OFF; //Sätter pin åttas output till låg, vilket innebär att skärmen är av vid start.
	lcd.begin(16, 2); // set up the LCD's number of columns and rows.
}

ISR(PCINT0_vect) {

	PCMSK0 &= ~(1 << PCINT2); //Stänger av interrupts på knappen.
	TIMSK1 |= (1 << OCIE1A); //Initierar timer 1.

	if (BUTTON_NPRESSED) print_power();
	else print_voltage_amps();

	interrupts();
}

ISR(TIMER0_OVF_vect) {

	if (++timer0_counter >= TIMER0_MAXV) {
		measure_power_avg();
	}
}

ISR(TIMER1_COMPA_vect) {

	if (++timer1_counter >= TIMER1_MAXV) {

		timer1_counter = 0; //Nollställer timer-räknaren.

		PCICR = (1 << PCIE0); //Slår återigen på interrupts på knappen.
		PCMSK0 |= (1 << PCINT2);
	}
}


float get_voltage() {
	adcVoltage = (analogRead(A2) * 5);
	return ((adcVoltage / 1023) * 5) * 1.15;
}

long get_current() {

	sensValue = currentSensor.mA_DC();
	if (sensValue < 0.9) { //Filtrerar skräpvärden. 
		sensValue = 0;
	}

	for (int i = 0; i < 5; i++) {
		*values[i] = (sensValue / 10);
		delay(20);
	}
	return ((v1 + v2 + v3 + v4 + v5) / 5);
	//^Plussar ihop 10 mätvärden och dividerar med 10 för att få medelvärde.
}

void print_voltage_amps() {

		for (int i = 0; i < 5; i++) {

			float voltage = get_voltage();
			long currentAvg = get_current();

			LCD_BACKL_ON; //Sätter pin åttas output till hög, dvs 5v (Närmare 3.5v via USB).		

			lcd.print("                ");//Rensar raden inför uppdatering.
			lcd.setCursor(0, 0);
			lcd.print("Volt: ");
			lcd.print(voltage);
			lcd.print(" V");

			lcd.print("                ");//Rensar raden inför uppdatering.
			lcd.setCursor(0, 1); //Sätter pekaren till rad 2 på LCD-skärmen.
			lcd.print("Amp = ");

			if (CURRENT_MAMPS) { //Ifall mätvärdet är i mA-range printar vi med mA-suffix.
				lcd.print(currentAvg);
				lcd.print(" mA");
			}
			else if (CURRENT_AMPS) { //Ifall mätvärdet överstiger eller = 1000mA använder vi Ampere-suffix.
				lcd.print(currentAvg / 1000);
				lcd.print(" A");
			}
			delay(500);
		}
	return;
}

void print_power() {

	for (int i = 0; i < 5; i++) {

		float voltage = get_voltage();
		long currentAvg = get_current();

		powerMoment = voltage * (currentAvg / 1000);

		LCD_BACKL_ON; //Sätter pin åttas output till hög, dvs 5v (Närmare 3.5v via USB).		

		lcd.print("                ");//Rensar raden inför uppdatering.
		lcd.setCursor(0, 0);
		lcd.print("PowM: ");
		lcd.print(powerMoment);
		lcd.print(" W");

		lcd.print("                ");//Rensar raden inför uppdatering.
		lcd.setCursor(0, 1); //Sätter pekaren till rad 2 på LCD-skärmen.
		lcd.print("PowAvg = ");

		if (POWER_WATTS) { //Ifall mätvärdet överstiger 999 mW använder vi W-enheten.
			lcd.print(powerTot/1000);
			lcd.print(" W");
		}
		else if (POWER_MWATTS) { //Ifall mätvärdet är på under 1W använder vi mW-enheten.
			lcd.print(powerTot);
			lcd.print(" mW");
		}

		delay(500);
	}
	return;
}

void measure_power_avg() {
	
	float voltage = get_voltage();
	long currentAvg = get_current();
	powerAvg = voltage * (currentAvg/10);

	if (iterator = false) {
		prevpowerAvg = powerAvg;
		iterator = !iterator;
	}
	
	powerTot = ((prevpowerAvg + powerAvg) / 2);

	return;
}

void lcd_sleep() {

	lcd.setCursor(0, 0);
	lcd.print("                ");
	lcd.setCursor(0, 1);
	lcd.print("                ");
	LCD_BACKL_OFF;

	return;
}

void loop() {

}