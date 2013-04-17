

#include <Max3421e.h>
#include <Usb.h>
#include <AndroidAccessory.h>
//#include <Motor.h>
#define  LED_PIN  13

//Protocol Messages
#define SYNC 's'

#define	BATTERY_LEVEL 'b'
#define SPEED 'v'
#define TURN 't'
#define DRIVE 'm'

#define ENABLED 'e'
#define DISABLED 'd'

//Event frequencies (Hz)
#define CONTROL_FREQ 100
#define SPEED_FREQ 10
#define COMM_FREQ 200
#define BATT_FREQ 0.5



#define leftSpeed    2
#define rightSpeed   4
#define leftDir      3
#define rightDir     5







// ADK Config
AndroidAccessory acc("X3R",
					 "Rover",
					 "Remote control rover",
					 "0.1",
					 "http://www.bovbel.com",
					 "0000000012345678");

//Motor Initialize
//Motor(int pinfwd, int pinbwd, int pinpwm, int pindir, int pinenc, bool biasdir)
//Motor motorR(9, 8, 10, 15, 2, false);
//Motor motorL(5, 6, 4, 14, 3, true);

//Main Timer
long timer_batt = millis();
long timer_pid = millis();
long timer_speed = millis();
long timer_comm = millis();

//Vehicle parameters
int vehicle_speed;
int vehicle_turn;

//Battery
int batt_measure;
int batt_filtered;
int batt_prev = 0;

void setup()
{

	// Serial Communication
	Serial.begin(9600);
	Serial.print("Hello World!\n");
	pinMode(LED_PIN, OUTPUT);

	// ADK Config
	acc.powerOn();

	// Setup encoder interrupts
	//attachInterrupt(motorR.getPinEnc(), ticR, CHANGE);
	//attachInterrupt(motorL.getPinEnc(), ticL, CHANGE);

	//Turn off motors
	//Motor::off();
	//motorL.setSpeed(0);
	//motorR.setSpeed(0);

	//Set battery ref voltage
	//analogReference(INTERNAL2V56);
	pinMode(LED_PIN,OUTPUT);

}
int speedVsTurn;
int tmpSpeed, tmpTurn;
void loop()
{
	byte msg[3];






	//Control update, 100 Hz loop
	if (millis()-timer_pid >= 1000/CONTROL_FREQ ){
		timer_pid = millis();

      tmpSpeed = vehicle_speed;
      tmpTurn = vehicle_turn/2;

if(tmpSpeed==0 && tmpTurn==0)
{
     digitalWrite(leftSpeed, LOW);
     digitalWrite(leftDir, LOW); 
     digitalWrite(rightSpeed, LOW);
     digitalWrite(rightDir, LOW);   // motor stop

}else
{
if(tmpSpeed>0 && tmpTurn>0)
{
                   digitalWrite(leftDir, HIGH );
                    analogWrite(leftSpeed, ((vehicle_speed+tmpTurn) * 2.5));
                   digitalWrite(rightDir, HIGH );
                    analogWrite(rightSpeed, ((vehicle_speed-tmpTurn) * 2.5));
  
//left fast fowr
//right slow forw
}
if(tmpSpeed<0 && tmpTurn<0)
{
                   digitalWrite(leftDir, LOW );
                    analogWrite(leftSpeed, 255-((vehicle_speed-tmpTurn) * 2.5));
                   digitalWrite(rightDir, LOW );
                    analogWrite(rightSpeed, 255-((vehicle_speed+tmpTurn) * 2.5));

//left fast rev
//right slow rev
}
if(tmpSpeed<0 && tmpTurn>0)
{
                   digitalWrite(leftDir, LOW );
                    analogWrite(leftSpeed, 255-((vehicle_speed+tmpTurn) * 2.5));
                   digitalWrite(rightDir, HIGH );
                    analogWrite(rightSpeed, ((vehicle_speed-tmpTurn) * 2.5));
//left fast fow
//right slow rev

}
if(tmpSpeed>0 && tmpTurn<0)
{
                   digitalWrite(leftDir, HIGH );
                    analogWrite(leftSpeed, ((vehicle_speed-tmpTurn) * 2.5));
                   digitalWrite(rightDir, LOW);
                    analogWrite(rightSpeed, 255-((vehicle_speed+tmpTurn) * 2.5));
//left fast rev
//right slow fow

}
}

    
      
       
             //   if(speedVsTurn!=2)return;
                  
	//	motorL.setSpeed(fmap(vehicle_speed,-100,100,-15,15));
	//	motorR.setSpeed(fmap(vehicle_speed,-100,100,-15,15));
           

		//motorL.runPID();
		//motorR.runPID();
	}

	//Sensor update, 10 Hz loop
	if (millis()-timer_speed >= 1000/SPEED_FREQ && speedVsTurn ==1 ){
	/*	timer_speed = millis();
             //  if(speedVsTurn!=1)return;
		Serial.print(" speed: ");
		Serial.print(vehicle_speed);
		Serial.print(" turn: ");
		Serial.print(vehicle_turn);
                if(vehicle_speed == 0 ){
                   digitalWrite(2, LOW);
                   digitalWrite(3, LOW); 
                   digitalWrite(4, LOW);
                   digitalWrite(5, LOW);
                }
                else if(vehicle_speed>0 ){
                    digitalWrite(2, HIGH );
                    analogWrite(3, vehicle_speed * 2.5);
                    digitalWrite(2, HIGH );
                    analogWrite(3, vehicle_speed * 2.5);
                }
                else if (vehicle_speed<0 ) {
                    digitalWrite(2, LOW );
                    analogWrite(3, 255 - (vehicle_speed * 2.5));
                    digitalWrite(2, LOW );
                    analogWrite(3, 255 - (vehicle_speed * 2.5));
                }
                
		//Serial.print(" l speed: ");
		//Serial.print(motorL.getTargetSpeed());
		//Serial.print(" r speed: ");
		//Serial.print(motorR.getTargetSpeed());

		//Serial.print(" batt: ");
		//Serial.print(map(batt_filtered,360,500,0,101));

		//Serial.print("\n");

	//	motorL.updateSpeed();
	//	motorR.updateSpeed();
		updateBatt();*/
	}

	//ADK Communication
	if (acc.isConnected() && millis()-timer_comm >= 1000/COMM_FREQ) {
		timer_comm = millis();
		//Always Read
		int len = acc.read(msg, sizeof(msg), 1); // read data into msg variable
		if (len > 0) {
			if (msg[0] == SYNC){
				switch (msg[1]){
					case SPEED:
						vehicle_speed = constrain((msg[2] < 128 ? msg[2] : msg[2]-256), -100, 100);
						break;

					case TURN:
						vehicle_turn = constrain((msg[2] < 128 ? msg[2] : msg[2]-256), -100, 100);
						break;

					case DRIVE:
						if (msg[2] == ENABLED){
					//		Motor::on();
							Serial.print("drive enabled\n");
						}else{
					//		Motor::off();
							Serial.print("drive disabled\n");
						}
						break;
				}
			}else{
				Serial.print("Out of SYNC!");
			}
		}
	}

	//Batt update, 0.1 Hz loop
	if (acc.isConnected() && millis()-timer_batt >= 1000/BATT_FREQ) {
		timer_batt = millis();
		//Serial.print("b");
		msg[0] = SYNC;
		msg[1] = BATTERY_LEVEL;
		msg[2] = getBatt();
		acc.write(msg, 3);

	}

	//Failsafe
	if (!acc.isConnected()){
		//If not connected, disable motors
		//Motor::off();
		vehicle_speed = 0;
		vehicle_turn = 0;
	}
}


//Dirty, dirty interrupts!
void ticL(){
//	motorL.incTic();
}
void ticR(){
	//motorR.incTic();
}

void updateBatt(){

	//Low-pass filter on battery level
	batt_filtered = batt_prev + 0.1 * (analogRead(A0) - batt_prev);
	batt_prev = batt_filtered;
	if(batt_filtered > 380){
		digitalWrite(LED_PIN,HIGH);
	}else{
		digitalWrite(LED_PIN,LOW);
	}

}

byte getBatt(){

	//Map to percentage
	return (byte)map(batt_filtered,380,516,0,100);
}


float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
