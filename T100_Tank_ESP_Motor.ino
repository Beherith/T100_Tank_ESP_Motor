//TODO:
//Add Encoder info
//Add PID to set positions
//Add command parser for the following commands:
//Set motor pwm value
//	should obey timout
//set wheel speed (achieve with PID)
//	should obey timout
//set wheel position (govern with PID)
//	should call back when command finished
//set LED colors, with timeout
//	 a timeout of 0 means indefinately
//	a timeout of -1 means return to normal operation
//	Set R1 B1 G1 W1 colors from [0;255]
//set the gyro update rate
//get gyro status
//get VBat
//set command timeout
//set motor PWM frequency

//Possible control states:
//[PWM control] -> [Speed control] -> [velocity control]

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
//#include <ArduinoOTA.h>
//#include <WiFiUdp.h>

#include <ESP8266mDNS.h>
#include <WebSocketsServer.h>
#include <DNSServer.h>
#include <Hash.h>

#include "WEMOS_Motor.h"
#include <NeoPixelBus.h>

#define MOTOR_SHIELD_SCL 5 //D1
#define MOTOR_SHIELD_SDA 4 //D2

#define MPU9255_SDA
#define MPU9255_SCL

#define LENC1
#define LENC2
#define LENC3
#define LENC4
// Rotary encoder info:
//https://github.com/LennartHennigs/ESPRotary
//https://github.com/PaulStoffregen/Encoder/blob/master/Encoder.h

// fast PID: https://github.com/mike-matera/FastPID
class TankTrack{
	public:
		int encApin = -1;
		int encBpin = -1;
		volatile int position = 0;
		bool movingdirection = 0;
		int acceleration = 0;
	void TankTrack(int apin, int bpin){
		encApin = apin;
		encBpin = bpin;
		pinMode(encApin,INPUT_PULLUP);
		pinMode(encBpin,INPUT_PULLUP);
		
	}
	
	
}

struct tankstate{
	uint32_t ax,ay,az;
	uint32_t gx,gy,gz;
	uint32_t mx,my,mz;
	uint32_t ltpos, rtpos;
	uint32_t ltrate, rtrate;
	int32_t lpwm,rpwm;
	uint32_t lastcommand;
	uint32_t drivepwm;
} TS;

#define NEOPIXELBUS 2 //D4
NeoPixelBus<NeoGrbwFeature,NeoEsp8266Uart1800KbpsMethod> strip(8);


RgbwColor white(255,255,255,255);
RgbwColor yellow(255,100,0,50);
RgbwColor red(255,0,0,0);
RgbwColor green(0,255,0,0);
RgbwColor off(0,0,0,0);

void flash(uint32_t t, uint32_t led1, uint32_t led2, RgbwColor c){;
	if ((t >>7)%2 ==0){
		strip.SetPixelColor(led1,off);
		strip.SetPixelColor(led2,off);
	}else {
		strip.SetPixelColor(1,c);
		strip.SetPixelColor(6,c);
	}
	}

uint32_t updateLEDS(){
	uint32_t dt = micros();
	uint32_t now = millis();
	strip.SetPixelColor(0, white);
	strip.SetPixelColor(3, white);
	strip.SetPixelColor(4, red);
	strip.SetPixelColor(7, red);
	
	strip.SetPixelColor(1,off);
	strip.SetPixelColor(2,off);
	strip.SetPixelColor(5,off);
	strip.SetPixelColor(6,off);
	const int turnthres = 10;
	//Movestate Classes:
	// A. Stopped
	// B. Going forward
		// Forward pure
		// Forward left
		// Forward Right
	// C. Going Back
		// Back Pure
		// Back left
		// Back Right
	// D. Turninplace Left
	// E . Turninplace Right
	if (TS.lpwm == 0 && TS.rpwm == 0){ //Stopped 

	}else if(TS.lpwm > 0 && TS.rpwm > 0){ //Going Forward
		strip.SetPixelColor(1,white);
		strip.SetPixelColor(2,white);
		strip.SetPixelColor(5,off);
		strip.SetPixelColor(6,off);
		if((TS.lpwm - TS.rpwm) > turnthres){ // Forward left
			flash(now, 1, 6, yellow);
		} else if((TS.rpwm - TS.lpwm) > turnthres){ //Forward right
			flash(now, 2, 5, yellow);
		} else { //Forward Pure
			
		}
	}else if(TS.lpwm < 0 && TS.rpwm < 0){ //Going Back
		strip.SetPixelColor(1,off);
		strip.SetPixelColor(2,off);
		strip.SetPixelColor(5,white);
		strip.SetPixelColor(6,white);
		if((TS.lpwm - TS.rpwm) < turnthres){ // Back Right
			flash(now, 2, 5, yellow);
		} else if((TS.rpwm - TS.lpwm) > turnthres){ //Forward right
			flash(now, 1, 6, yellow);
		} else { //Back pure
		}
	} else if(TS.lpwm < 0 && TS.rpwm > 0) {//turninplace left
		if((TS.lpwm - TS.rpwm) < turnthres){ // Back Right
			flash(now, 2, 5, yellow);
		} else if((TS.rpwm - TS.lpwm) > turnthres){ //Forward right
			flash(now, 1, 6, yellow);
		}
	} else if(TS.lpwm > 0 && TS.rpwm < 0) {//turninplace right
		if((TS.lpwm - TS.rpwm) < turnthres){ // Back Right
			flash(now, 2, 5, yellow);
		} else if((TS.rpwm - TS.lpwm) > turnthres){ //Forward right
			flash(now, 1, 6, yellow);
		}
	}else { //unknown state
		strip.SetPixelColor(1,green);
		strip.SetPixelColor(2,green);
		strip.SetPixelColor(5,green);
		strip.SetPixelColor(6,green);
	}
	strip.Show();
	return micros()-dt;
}

void addencoder(){};

/*
#LEDS
#8 leds in total, first is fw left outer
#FRONT
0 1 2 3
7 6 5 4
#BACK
#NeoEsp8266Uart1800KbpsMethod 
https://github.com/Makuna/NeoPixelBus/wiki/ESP8266-NeoMethods
on gpio2
#using Neopixelbus
#

#GYRO, Accel, magnetometer

*/
//#define DEBUG_ESP_HTTP_SERVER

const char *ssid = "TigerTank";
const char *password = "";//"TigerTank";

volatile unsigned long next;

const char* mdnsName = "tank"; // Domain name for the mDNS responder

const byte captive_portal=1;
const byte DNS_PORT = 53;
const char* serverIndex = "<form method='POST' action='/upload' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";

extern const char index_html[];

unsigned int alivecount=0;
unsigned long int lastpacket = 0;
bool active = 0;

IPAddress apIP(192, 168, 4, 1);
DNSServer dnsServer;
ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);


void handleRoot() {
 
  server.send_P(200,"text/html", index_html);
  Serial.println("handleroot: Client in.");
  }
 
void handle404() {
 
  server.send(200,"text/html", "<html><title>NO Tank</title><body>No Tank 404</body></html>");
  Serial.println("handle404");
  }
 /*
#define L1 0 //4
#define L2 14 //5
#define R1 4 //0
#define R2 5 //14
*/
int PwmFrequency = 50; //should default to 20k if decay mode is being changed.
Motor MLeft(0x30,_MOTOR_A,PwmFrequency);
Motor MRight(0x30,_MOTOR_B,PwmFrequency);
int lspeed = 0;
int rspeed = 0;

#define redled 12

int speed100 (int motf, int motr){ //returns [-100;0;100] range of motor speed
	if (motf == 255 && motr >0)
		return -1*( 255-motr )*100/255;
	else if (motr == 255 && motf >0)
		return ( 255-motf )*100/255;
	else if (motf == 0 && motr > 0)
		return (motr*100)/255;
	else if (motr == 0 && motf > 0)
		return -1*(motf*100)/255;
	else return 0;
}
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
			digitalWrite(redled,LOW);
			Serial.println("WS DC");
             break;
        case WStype_CONNECTED: {
            // send message to client
            webSocket.sendTXT(num, "Connected");
			digitalWrite(redled,HIGH);
			Serial.println("WS CONN");
        }
            break;
        case WStype_TEXT: {
            if(payload[0]=='g' && payload[1]=='e' && payload[2]=='t'){
/*              msp_analog_t analogdata;
              String senddata="{\"vbat\": ";
              if (msp.request(MSP_ANALOG, &analogdata, sizeof(analogdata))) {
                senddata+=String(analogdata.vbat);
              }
              else
 
               senddata+="0";
              senddata += "}";
              webSocket.sendTXT(num, senddata);
*/            }
			Serial.println("WS_Text");
        }
        break;
        case WStype_BIN: {
			uint32_t tt = micros();
			if (length ==5){
				active = 1;
				lspeed=speed100(payload[0],payload[1]);
				if (lspeed > 0) MLeft.setmotor(_CW,lspeed);
				else if (lspeed <0) MLeft.setmotor(_CCW,-lspeed);
				else MLeft.setmotor(_SHORT_BRAKE);
				
				rspeed=speed100(payload[2],payload[3]);
				if (rspeed > 0) MRight.setmotor(_CW,rspeed);
				else if (rspeed <0) MRight.setmotor(_CCW,-rspeed);
				else MRight.setmotor(_SHORT_BRAKE);
				
				/*
				analogWrite(L2,payload[0]   );
				analogWrite(L1,payload[1]   );
				
				analogWrite(R2,payload[2]   );
				analogWrite(R1,payload[3]  );
				*/
				
				Serial.print("L=");
				Serial.print(lspeed);
				Serial.print(" R=");
				Serial.println(rspeed);
				if (payload[4] & 1) digitalWrite(redled,HIGH);
				else digitalWrite(redled,LOW);
				
				//we have 5 bits of pwmfreq;
				float pwmfactor = (payload[4] & 0b00111110)>>1;
				float newpwmfreq = (20000.0 * pow(0.82,pwmfactor));
				//Serial.println(pwmfactor);
				//Serial.println(newpwmfreq);
				int k = (float) newpwmfreq;
				if (k!= PwmFrequency){
					PwmFrequency = k;
					MLeft.setfreq(k);
					MRight.setfreq(k);
					
				}
				
				/*if (k!= PwmFrequency){
					PwmFrequency = k;
					analogWriteFreq(PwmFrequency);
					
				}*/

				//50/20000 = y^31
				//
				
				lastpacket = millis();
				//Serial.print("lp: ");
			
				//Serial.println(lastpacket,HEX);
			}else{
				Serial.print("Binary packet length !=5: ");
				Serial.println(length);
			}
			for (int i = 0; i < length; i++){
				Serial.print(payload[i]);
			}
			tt = micros()-tt;
			Serial.println(tt);
			Serial.print("us dt");
			//Serial.println(" <-packet");
        }
        break;
    }

}
/*
void toggletest(int pin){
  for (int i = 0; i<32;i++){
    analogWrite(pin,8*i);
    delay(3);
  }
  analogWrite(pin,0);
  delay(500);
}*/
void toggletest(){
  MLeft.setmotor(_CW,50);
  delay(100);
  MLeft.setmotor(_STOP);
  delay(300);
  MLeft.setmotor(_CCW,50);
  delay(100);
  MLeft.setmotor(_STOP);
  delay(300);
  MRight.setmotor(_CW,50);
  delay(100);
  MRight.setmotor(_STOP);
  delay(300);
  MRight.setmotor(_CCW,50);
  delay(100);
  MRight.setmotor(_STOP);
  delay(300);
}
void setupmotorpins(){
  analogWriteRange(255);
  analogWriteFreq(PwmFrequency) ;
  pinMode(redled,OUTPUT);
  digitalWrite(redled,LOW);
  toggletest();
  // initialise pins
 /* pinMode(L1,OUTPUT);
  pinMode(L2,OUTPUT);
  pinMode(R1,OUTPUT);
  pinMode(R2,OUTPUT);
  digitalWrite(L1,LOW);
  digitalWrite(L2,LOW);
  digitalWrite(R1,LOW);
  digitalWrite(R2,LOW);
  toggletest(L1);
  toggletest(L2);
  toggletest(R1);
  toggletest(R2);*/
  pinMode(BUILTIN_LED, OUTPUT); 
}

void startMDNS() { // Start the mDNS responder
  MDNS.begin(mdnsName);                        // start the multicast domain name server
  Serial.print("mDNS responder started: http://");
  Serial.print(mdnsName);
  Serial.println(".local");
}

void setup() {
	//pinMode(DEBUGPIN,OUTPUT);
	setupmotorpins();
	//digitalWrite(DEBUGPIN, !onState); //set the PPM signal pin to the default state (off)
/* You can remove the password parameter if you want the AP to be open. */

//  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
	WiFi.softAP(ssid,password,2);

	IPAddress myIP = WiFi.softAPIP();

	if(captive_portal) dnsServer.start(DNS_PORT, "*", apIP);
	server.onNotFound(handle404);
	server.on("/", handleRoot);

	server.on("/update", HTTP_GET, [](){
		delay(100);    
		server.sendHeader("Connection", "close");
		server.sendHeader("Access-Control-Allow-Origin", "*");
		server.send(200, "text/html", serverIndex);
	});

	server.on("/upload", HTTP_POST, [](){
		server.sendHeader("Connection", "close");
		server.sendHeader("Access-Control-Allow-Origin", "*");
		server.send(200, "text/plain", (Update.hasError())?"FAIL":"OK");
		ESP.restart();
	  },[](){
		  HTTPUpload& upload = server.upload();
		  if(upload.status == UPLOAD_FILE_START){
			WiFiUDP::stopAll();
			uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
			if(!Update.begin(maxSketchSpace)){//start with max available size
	 //         Update.printError(Serial);
			}
		  } 
		  else if(upload.status == UPLOAD_FILE_WRITE){
			if(Update.write(upload.buf, upload.currentSize) != upload.currentSize){
	//          Update.printError(Serial);
			}
		  } 
		  else if(upload.status == UPLOAD_FILE_END){
			if(Update.end(true)){ //true to set the size to the current progress
	 //         Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
			} else {
	//          Update.printError(Serial);
			}
	 //       Serial.setDebugOutput(false);
		  }
		  yield();
		});
	 
	server.begin();
	webSocket.onEvent(webSocketEvent);
	webSocket.begin();
	startMDNS();
	Serial.begin(115200);
	Serial.println("Tank Ready");
	Serial.println(pow(0.82,31.0));
}

unsigned long time_now = 0;
unsigned long last_update =0;
unsigned long update_rate = 20; //ms
void loop() {
	if (millis()>last_update + update_rate){
		last_update += update_rate;
		//update PID controllers for tracks
		
		//
		
		//output gyro data
		
	}else{
		
	}

  webSocket.loop();
//  ArduinoOTA.handle();
  if(captive_portal)
    dnsServer.processNextRequest();
  server.handleClient();
  if (active && ( millis() - lastpacket>750)){
		Serial.println("Timeout after 300ms");
		MLeft.setmotor(_STOP);
		MRight.setmotor(_STOP);
		/*analogWrite(L2,0  );
		analogWrite(L1,0  );
		analogWrite(R2,0  );
		analogWrite(R1,0  );*/
		active = 0;

  }

  yield();
}
