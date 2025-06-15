/**
 * ______________________________________________________________________________________________________
 * @author		:		HITESH BHOYAR
 * @file    	:		transmitter.ino
 * @brief   	:		This file includes the rover code and its functions
 * ______________________________________________________________________________________________________
 */

/* Includes ----------------------------------------------------------------------------------*/
#include <SPI.h>                // FOR SPI COMMUNICATION OF NRF24L01 WITH ARDUINO
#include <nRF24L01.h>           // FOR NRF24L01
#include <RF24.h>               // FOR NRF24L01
#include <Wire.h>               // FOR I2C COMMUNICATION with OLED
#include <Arduino.h>
#include <U8x8lib.h>

/* Initialize nrf24l01 with CE CSN pins ------------------------------------------------------*/
RF24 radio(9, 10);              // nRF24L01 (CE, CSN)
const byte addresses[][6] = {"00001", "00002"};  //PIPE ADDRESS

/* Definitions -------------------------------------------------------------------------------*/
#define j1 7                    // JOYstick_STICK BUTTON
#define j2 8
#define t1 5                    // TOGGLE SWITCH 
#define t2 6 

U8X8_SH1106_128X64_NONAME_HW_I2C display(/* reset=*/ U8X8_PIN_NONE);

unsigned long time0 = 0;
unsigned long time1 = 0;

/* Structure Definition ----------------------------------------------------------------------*/
struct data_package             // MAX SIZE OF THIS STRUCT IS 32 BYTES - NRF24L01 BUFFER LIMIT
{
	byte joystick_1X;
	byte joystick_1Y;
	byte joystick_1button;

	byte joystick_2X;
	byte joystick_2Y;
	byte joystick_2button;

	byte toggle_1;
	byte toggle_2;
};
data_package data;              // VARIABLE IS data

struct gps_package              // STRUCTURE FOR GPS DATA MAX 32 BYTES
{
	float latitude;
	float longitude;
	float altitude;
	float speed;
	byte  satellite;
	byte  is_working;
};
gps_package gpsdata;            // VARIABLE IS gpsdata

/**
 * @brief  Initialize the Transmitter
 * @retval No return value
 */
void setup() 
{
	Serial.begin(9600);

	radio.begin();                            // SET RADIO
	radio.openWritingPipe(addresses[1]);      // 00002
	radio.openReadingPipe(1, addresses[0]);   // 00001
	radio.setAutoAck(false);
	radio.setDataRate(RF24_250KBPS);
	radio.setPALevel(RF24_PA_LOW);

	pinMode(j1, INPUT_PULLUP);                // Activate the Arduino internal pull-up resistors
	pinMode(j2, INPUT_PULLUP);
	pinMode(t1, INPUT_PULLUP);
	pinMode(t2, INPUT_PULLUP);

	set_default();                            // SET INITIAL DEFAULT VATUES OF INPUTS

	display.begin();
	display.setPowerSave(0);  
}

/**
 * @brief  Loop function 
 * @retval No return value
 */
void loop() 
{
	/* SET AS TRANSMITTER ------------------------------------------------------------------------*/

	radio.stopListening();

	/* READ ANALOG INPUTS ------------------------------------------------------------------------*/
	data.joystick_1X = map(analogRead(A0), 1023, 0, 0, 255);  // Convert the analog read value from 0 to 1023 into a BYTE value from 0 to 255
	data.joystick_1Y = map(analogRead(A1), 1023, 0, 0, 255);
	data.joystick_2X = map(analogRead(A2), 1023, 0, 0, 255);
	data.joystick_2Y = map(analogRead(A3), 1023, 0, 0, 255);

	/* READ DIGITAL INPUTS -----------------------------------------------------------------------*/
	data.joystick_1button = digitalRead(j1);
	data.joystick_2button = digitalRead(j2);
	data.toggle_1  = digitalRead(t1);
	data.toggle_2  = digitalRead(t2);

	/* Serial Print Joystick read data -----------------------------------------------------------*/
	// serial_print_data();                                  // SERIAL PRINT Transmitted DATA

	radio.write( &data, sizeof(data_package) );           // Send the data from the structure to the receiver

	delay(5);

	/* SET AS RECEIVER ---------------------------------------------------------------------------*/
	radio.startListening();                               // SET AS RECEIVER FOR OLED
	if ( radio.available() ) 
	{
		radio.read( &gpsdata, sizeof(gps_package) );
		time0 = millis();
	}
	time1 = millis();

	/* Serial Print Received GPS Data ------------------------------------------------------------*/
	serial_print_gpsdata();                               // SERIAL PRINT GPS DATA

	OLED_gpsdata();

	delay(5);
}

/**
 * @brief  Serial Print Joystick Data
 * @retval No return value
 */
void serial_print_data()
{
	Serial.print("joy1 = ");
	Serial.print(data.joystick_1X);	      Serial.print("  ");
	Serial.print(data.joystick_1Y);	      Serial.print("  ");
	Serial.print(data.joystick_1button);	Serial.print("  |  ");
	Serial.print("joy2 = ");
	Serial.print(data.joystick_2X);	      Serial.print("  ");
	Serial.print(data.joystick_2Y);	      Serial.print("  ");
	Serial.print(data.joystick_2button);	Serial.print("  |  ");
	Serial.print("toggle1 = ");
	Serial.print(data.toggle_1);	        Serial.print("  |  ");
	Serial.print("toggle2 = ");
	Serial.println(data.toggle_2);
}

/**
 * @brief  Serial Print Received GPS Data
 * @retval No return value
 */
void serial_print_gpsdata()
{
	Serial.print(" LATI: ");  
	Serial.print( gpsdata.latitude,6 );
	Serial.print(" LONG: ");
	Serial.print( gpsdata.longitude,6 );
	Serial.print(" ALTI: ");
	Serial.print( gpsdata.altitude,3 );
	Serial.print(" Speed: ");
	Serial.print( gpsdata.speed,3 );      
	Serial.print(" SAT: ");
	Serial.println( gpsdata.satellite );
	// Serial.print(" receiver data: ");
	// Serial.println( gpsdata.is_working );
}

/**
 * @brief  Display Gps data on oled display
 * @retval No return value
 */
void OLED_gpsdata()
{
	// display.clear();
	display.setFont(u8x8_font_chroma48medium8_r);

	display.setCursor(0, 0);
	display.setInverseFont(1);
	display.print("  GPS LOCATION  ");
	display.setInverseFont(0);
	display.setCursor(0, 1);
	display.print("LATI: ");                  // LATITUDE
	display.print(gpsdata.latitude,  7 );
	display.print(" ");
	display.setCursor(0, 2);
	display.print("LONG: ");                  // LONGITUDE
	display.print(gpsdata.longitude, 7 );
	display.print(" ");
	display.setCursor(0, 3);
	display.print("ALTI: ");                  // ALTITUDE
	display.print(gpsdata.altitude,  3 );
	display.print("     ");
	display.setCursor(0, 4);
	display.print("Speed:");                  // SPEED KMPH
	display.print(gpsdata.speed, 3 );
	display.print(" KMPH");
	display.setCursor(0, 5);
	display.print("SAT : ");                  // NUM OF SATELLITE TRACKING
	display.println(gpsdata.satellite );
}

/**
 * @brief  Set data to default value
 * @retval No return value
 */
void set_default()
{
	data.joystick_1X    = 127;
	data.joystick_1Y    = 127;
	data.joystick_1button = 1;

	data.joystick_2X    = 127;
	data.joystick_2Y    = 127;
	data.joystick_2button = 1;

	data.toggle_1 = 1;
	data.toggle_2 = 1;
}