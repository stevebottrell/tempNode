

/*
 * Transmitter.ino
 *
 * Created: 10/26/2014 8:32:56 PM
 * Author: Steve Bottrell
 */ 

#include <OneWire.h>
#include <ATTinyWatchdog.h>
#include <DallasTemperature.h>
#include <RF24.h>
#include <SPI.h>
#include <nRF24L01.h>
#include "printf.h"
#include <avr/sleep.h>
#include <avr/power.h>
#define CE_PIN   7 //pin 9 on 8mhz sensor pin 7 on 16mhz sensor
#define CSN_PIN 8
#define BLINK_PIN 2
struct MyData{
	char	cmd;
	byte ID[8];
	float batteryVoltage;
	float temperature;
	unsigned long  timer;
	unsigned int charge;
};
const uint64_t pipe[2] = {0xE8E8F0F0E1LL,0xE8E8F0F0E2LL}; // Define the transmit pipe

RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

int i[11];  // 11 element array holding integers
int upInteger = 0;
int dnInteger = 0;
MyData Payload;
OneWire i2cBus(6);// DATAPIN OF DS18B20// Pin 6 on 16mhz sensor pin 2 on 8mhz sensor
DallasTemperature dTemp(&i2cBus);
DeviceAddress tempSensorAddress;

void setup()
{
	//Serial.begin(19200);
	//Serial.print("Size of Packet = ");
	//Serial.println(sizeof(pipe));
	analogReference(INTERNAL);
	//pinMode(A1,OUTPUT);//Charge sensing Voltage
	radio.begin();
	printf_begin();
	dTemp.begin();
	dTemp.getAddress(tempSensorAddress,0);
	for (byte i=0;i<8;i++)
	{
		Payload.ID[i]=tempSensorAddress[i];
	}
	
	radio.setDataRate(RF24_250KBPS);
	
	radio.openWritingPipe(pipe[1]);
	radio.openReadingPipe(1,pipe[0]);
	radio.printDetails();
	
	// Watchdog timeout values
	// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
	// 6=1sec, 7=2sec, 8=4sec, 9=8sec
	ATTINYWATCHDOG.setup(8);
	blink();
	blink();
}
void blink(){
	
	bitSet(PIND,2);
	delay(200);
	bitSet(PIND,2);
}
void loop()
{
	Payload.timer = ++upInteger;
	dTemp.requestTemperatures();
	delay(10);
	Payload.temperature = dTemp.getTempCByIndex(0);
	Payload.batteryVoltage = readVcc()/1000.0;
	//pinMode(A1,INPUT);
	Payload.charge = analogRead(A1);
	//pinMode(A1,OUTPUT);
	radio.powerUp();
	radio.stopListening();
	if (radio.write(&Payload, sizeof(Payload)))
	{
		blink();
		Serial.println("Transmit OK");
	}
	else
	{
		
		Serial.println("Transmit FAIL");
		blink();
		blink();
	}
	//radio.txStandBy();
	//radio.startListening();
	
	
	//delay(500);
	radio.powerDown();
	delay(10);
	do_sleep();  /*delay(500);*/
}
void wakeUp()
{
	
	sleep_disable();
	while (radio.available())
	{
		radio.read(&i,sizeof(i));
	}
	delay(50);
}

void do_sleep(void)
{
	//set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
	//sleep_enable();
	attachInterrupt(1,wakeUp,LOW);
	ATTINYWATCHDOG.sleep(75);
	//sleep_mode();                        /*System sleeps here*/
	// The WDT_vect interrupt wakes the MCU from here
	//sleep_disable();                     // System continues execution here when watchdog timed out
	detachInterrupt(1);
	
}

long readVcc() {
	// Read 1.1V reference against AVcc
	// set the reference to Vcc and the measurement to the internal 1.1V reference
	#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
	ADMUX = _BV(MUX5) | _BV(MUX0);
	#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
	ADMUX = _BV(MUX3) | _BV(MUX2);
	#else
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#endif

	delay(2); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Start conversion
	while (bit_is_set(ADCSRA,ADSC)); // measuring

	uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
	uint8_t high = ADCH; // unlocks both

	long result = (high<<8) | low;

	result = 1215300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
	//calibrate ((1.1 * Actual Voltage)/ ReadVCC Result) * 1023 * 1000
	return result; // Vcc in millivolts
}