/**
 * ______________________________________________________________________________________________________
 * @author		:		HITESH BHOYAR
 * @file    	:		rover.ino
 * @brief   	:		This file includes the rover code and its functions
 * ______________________________________________________________________________________________________
 */

/* Includes ----------------------------------------------------------------------------------*/
#include <SPI.h>                  // FOR SPI COMMUNICATION OF NRF24L01 WITH ARDUINO
#include <nRF24L01.h>             // FOR NRF24L01
#include <RF24.h>                 // FOR NRF24L01
#include <TinyGPS++.h>            // FOR GPS
#include <NewPing.h>              // FOR ULTRASONIC SENSORS
#include <NewTone.h>              // FOR BUZZER

/* Initialize nrf24l01 with CE CSN pins ------------------------------------------------------*/
RF24 radio(48, 49);                               // nRF24L01 (CE, CSN)
const byte addresses[][6] = {"00001", "00002"};   // PIPE ADDRESS

unsigned long time0 = 0;                          // Time tracking variables
unsigned long time1 = 0;

/* Definitions -------------------------------------------------------------------------------*/
#define motor1_forward   7
#define motor1_backward  6
#define motor2_forward   9
#define motor2_backward  8
#define motor3_forward   3
#define motor3_backward  2
#define motor4_forward   5
#define motor4_backward  4

#define neogps Serial1                            // SET SERIAL PORT1 AS GPS RECEIVING PORT Serial1 on pins 19 (RX) and 18 (TX)
TinyGPSPlus gps;                                  // TINYGPS set gps as function

#define maxdist 400                               // FOR ULTRASONIC SENSORS
NewPing sonar_back (22 , 23 , maxdist);          // Ultrasonic sensor trig, echo, maxdist
NewPing sonar_front(24 , 25 , maxdist);
NewPing sonar_right(26 , 27 , maxdist);
NewPing sonar_left (28 , 29 , maxdist);

int flag_front=0 , flag_back=0 , flag_left=0 , flag_right=0;

const int buzzerPin = 10; 
const int freq1 = 2000; 
const int freq2 = 4000; 

const int led_pin = 11;  // LED pin
const int ldr_pin = A0;  // LDR pin
const int ldr_intensity = 500; 

/* Structure Definition ----------------------------------------------------------------------*/
struct data_package               // STRUCTURE FOR CONTROLLER
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
data_package data;                // VARIABLE IS data

struct gps_package                // STRUCTURE FOR GPS DATA
{
	float latitude;
	float longitude;
	float altitude;
	float speed;
	byte  satellite;
	byte  is_working;
};
gps_package gpsdata;              // VARIABLE IS gpsdata

/**
 * @brief  Initialize the rover
 * @retval No return value
 */
void setup()
{
	Serial.begin(9600);
	neogps.begin(9600);                               // BEGIN SERIAL1 COMMUNICATION WITH Neo6mGPS

	radio.begin();                                    // SET RADIO
	radio.openWritingPipe( addresses[0]);             // 00001
	radio.openReadingPipe(1, addresses[1]);           // 00002
	radio.setAutoAck(false);
	radio.setDataRate(RF24_250KBPS);
	radio.setPALevel(RF24_PA_LOW);

	pinMode(motor1_forward  , OUTPUT);
	pinMode(motor1_backward , OUTPUT);
	pinMode(motor2_forward  , OUTPUT);
	pinMode(motor2_backward , OUTPUT);
	pinMode(motor3_forward  , OUTPUT);
	pinMode(motor3_backward , OUTPUT);
	pinMode(motor4_forward  , OUTPUT);
	pinMode(motor4_backward , OUTPUT);

	pinMode(buzzerPin, OUTPUT);
	// pinMode(LED_BUILTIN, OUTPUT);
	pinMode(led_pin, OUTPUT);
	resetdata();

}

/**
 * @brief  Loop function 
 * @retval No return value
 */
void loop()
{
	/* SET AS RECEIVER ---------------------------------------------------------------------------*/
	radio.startListening(); 
	if (radio.available()){
		radio.read(&data, sizeof(data_package));
		time0 = millis();
	}
	time1 = millis();
	/* FAILSAFE ----------------------------------------------------------------------------------*/
	if ( time1 - time0 > 1000 )
	{	
		resetdata();
		stop_motors();
	}
	/* Serial Print Received Data ----------------------------------------------------------------*/
	serial_print_received_data();

	/* Serial Print Ultrasonic sensor values -----------------------------------------------------*/
	// serial_print_ultrasonic();

	/* Flags for Spatial awareness and collision avoidance ---------------------------------------*/
	if( sonar_front.ping_cm() < 30 )	flag_front = 1;
	if( sonar_back.ping_cm () < 30 )	flag_back  = 1;
	if( sonar_left.ping_cm () < 30 )	flag_left  = 1;
	if( sonar_right.ping_cm() < 30 )	flag_right = 1;

	if(flag_front + flag_back + flag_left + flag_right >= 3){                   // HORN CODE
		if(!data.joystick_2button)
			NewTone(buzzerPin, freq1, 100);
	}
	else{
		if(!data.joystick_2button)
			NewTone(buzzerPin, freq2, 100);
	}

	/* LED LIGHT Code ----------------------------------------------------------------------------*/
	int   ldr_value = analogRead(ldr_pin);
	// Serial.println(ldr_value);
	if (ldr_value <= ldr_intensity) {
		digitalWrite(led_pin, LOW);
	}
	else {
		digitalWrite(led_pin, HIGH);
	}

	/* MOVEMENTS OF THE ROVER --------------------------------------------------------------------*/
	// if ((data.joystick_1Y > 150)&&flag_front==0 )
		if (data.joystick_1Y > 150 )
	{                                                                           // FORWARD
		digitalWrite(motor1_forward, HIGH);  digitalWrite(motor1_backward, LOW);
		digitalWrite(motor2_forward, HIGH);  digitalWrite(motor2_backward, LOW);
		digitalWrite(motor3_forward, HIGH);  digitalWrite(motor3_backward, LOW);
		digitalWrite(motor4_forward, HIGH);  digitalWrite(motor4_backward, LOW);
	}
	// else if ((data.joystick_1Y < 104)&&flag_back==0 )
		else if (data.joystick_1Y < 104 )
	{                                                                           // BACKWARD
		digitalWrite(motor1_forward, LOW);  digitalWrite(motor1_backward, HIGH);
		digitalWrite(motor2_forward, LOW);  digitalWrite(motor2_backward, HIGH);
		digitalWrite(motor3_forward, LOW);  digitalWrite(motor3_backward, HIGH);
		digitalWrite(motor4_forward, LOW);  digitalWrite(motor4_backward, HIGH);
	}
	else if (data.joystick_2X < 104 )
	{                                                                           // ANTI CLOCKWISE
		digitalWrite(motor1_forward, LOW);   digitalWrite(motor1_backward, HIGH);
		digitalWrite(motor2_forward, HIGH);  digitalWrite(motor2_backward, LOW);
		digitalWrite(motor3_forward, HIGH);  digitalWrite(motor3_backward, LOW);
		digitalWrite(motor4_forward, LOW);   digitalWrite(motor4_backward, HIGH);
	}
	else if (data.joystick_2X > 150 )
	{                                                                           // CLOCKWISE
		digitalWrite(motor1_forward, HIGH);  digitalWrite(motor1_backward, LOW);
		digitalWrite(motor2_forward, LOW);   digitalWrite(motor2_backward, HIGH);
		digitalWrite(motor3_forward, LOW);   digitalWrite(motor3_backward, HIGH);
		digitalWrite(motor4_forward, HIGH);  digitalWrite(motor4_backward, LOW);
	}
	else                                                                        // STOP MOVING
		stop_motors();

	flag_front=0 ; flag_back=0 ; flag_left=0 ; flag_right=0;

	delay(5);

	/* SET AS TRANSMITTER ------------------------------------------------------------------------*/
	radio.stopListening();                      // SET AS TRANSMITTER

	while (neogps.available() > 0){             // GET GPS DATA
		if (gps.encode(neogps.read()))
		{
			if ( gps.location.isValid() == 1 )
			{
				gpsdata.latitude  = gps.location.lat();
				gpsdata.longitude = gps.location.lng();
				gpsdata.altitude  = gps.altitude.meters();
				gpsdata.speed     = gps.speed.kmph();
				gpsdata.satellite = gps.satellites.value();
			}
			// else
			// { gpsdata.is_working = 1; }
		}
    // Serial.write(neogps.read());
  }

	radio.write( &gpsdata, sizeof(gps_package) );

	/* SERIAL PRINT GPS DATA ---------------------------------------------------------------------*/
	// serial_print_gpsdata();
	delay(5);
}

/**
 * @brief  Serial Print Received Data
 * @retval No return value
 */
void serial_print_received_data()
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
 * @brief  Serial Print GPS Coordinates
 * @retval No return value
 */
void serial_print_gpsdata()
{
	Serial.print(" LATI: ");  
	Serial.print( gpsdata.latitude );
	Serial.print(" LONG: ");
	Serial.print( gpsdata.longitude );
	Serial.print(" ALTI: ");
	Serial.print( gpsdata.altitude );
	Serial.print(" Speed: ");
	Serial.print( gpsdata.speed );      
	Serial.print(" SAT: ");
	Serial.print( gpsdata.satellite );
	Serial.print(" receiver data: ");
	Serial.println( gpsdata.is_working );
}

/**
 * @brief  Serial Print Ultrasonic Sensor values
 * @retval No return value
 */
void serial_print_ultrasonic()
{
	Serial.print("front-> ");  
	Serial.print( sonar_front.ping_cm()  );
	Serial.print(" | back-> ");  
	Serial.print( sonar_back.ping_cm()  );
	Serial.print(" | left-> ");  
	Serial.print( sonar_left.ping_cm()  );
	Serial.print(" | right-> ");  
	Serial.println( sonar_right.ping_cm()  );

}

/**
 * @brief  Reset data to default value
 * @retval No return value
 */
void resetdata() // RESET DATA PACAKAGE OF TRANSMITTER
{
	data.joystick_1X = 127;
	data.joystick_1Y = 127;
	data.joystick_1button = 1;
	data.joystick_2X = 127;
	data.joystick_2Y = 127;
	data.joystick_2button = 1;
	data.toggle_1 = 1;
	data.toggle_2 = 1;
}

/**
 * @brief  Function to stop the motors
 * @retval No return value
 */
void stop_motors(){
	digitalWrite(motor1_forward, LOW);  digitalWrite(motor1_backward, LOW);
	digitalWrite(motor2_forward, LOW);  digitalWrite(motor2_backward, LOW);
	digitalWrite(motor3_forward, LOW);  digitalWrite(motor3_backward, LOW);
	digitalWrite(motor4_forward, LOW);  digitalWrite(motor4_backward, LOW);
}
