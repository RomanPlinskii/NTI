/*#include <GOST4401_81.h>
#include <l3g4200d.h>
#include <lis331dlh.h>
#include <lis3mdl.h>
#include <MadgwickAHRS.h>
#include <stmhw.h>
#include <troyka-imu.h>
*/

#include <Wire.h>
#include <TroykaIMU.h>
#include <Servo.h>

#include <NewPing.h>

#define max_distance 300
#define m_speed 5
#define dir 4
#define direct 112

#define BETA 0.22
 
// создаём объект для фильтра Madgwick
Madgwick filter;
 
// создаём объект для работы с акселерометром
Accelerometer accel;
// создаём объект для работы с гироскопом
Gyroscope gyro;
// создаём объект для работы с компасом
Compass compass;
// переменные для данных с гироскопа, акселерометра и компаса
float gx, gy, gz, ax, ay, az, mx, my, mz;
// получаемые углы ориентации (Эйлера)
float yaw, pitch, roll;
float fps = 100;
unsigned long startMillis;
unsigned long deltaMillis;
const double compassCalibrationBias[3] = {
  -1109.851,
  595.231,
  -201.431
};
const double compassCalibrationMatrix[3][3] = {
  {1.323, 0.061, 0.087},
  {0.011, 1.313, -0.027},
  {0.025, -0.092, 1.205}
};

//trig echo
NewPing forward_sonar(48,49,max_distance);
NewPing left_sonar(50,51,max_distance);
NewPing rightForw_sonar(52,53,max_distance);
NewPing rightBack_sonar(46,47,max_distance);
Servo servo;
void setup() {
	pinMode(m_speed,OUTPUT);
	pinMode(dir,OUTPUT);
	pinMode(9,OUTPUT);
	pinMode(8,OUTPUT);
	servo.attach(26);
	Serial.begin(9600);
	pinMode(10, INPUT_PULLUP);
	/*
	for (int i=180;i>50;i--){
	servo.write(i);
	Serial.println(i);
	delay(200);
	}
	*/
	 accel.begin();
  // инициализация гироскопа
  gyro.begin();
  // инициализация компаса
  compass.begin();
  // калибровка компаса
  compass.calibrateMatrix(compassCalibrationMatrix, compassCalibrationBias);
	Serial.println("start");
	//delay(25000);
	
	while(millis()<20000){get_degrees();}
	Serial.println("go");
	turn(direct+30);
	turn(direct);
		
}

void loop() {
	get_degrees();
	if (!digitalRead(10)){
		
   		go(); 
		delay(7000);
		turn_left();
		delay(1000);
		turn_left();
		delay(7000);
		reverse();
		stop();
			
	}
		Serial.print(" forw_sonar: ");
		Serial.println(getSonar(forward_sonar, true));

		//reverse();
	
	/*
	go();
		correct_way();
		turn_left();
		correct_way();
		turn_left();
		correct_way();
		turn_left();
		correct_way();
		stop();
		
		go();
		correct_way();
		turn_left();
		delay(1000);
		turn_left();
		correct_way();
		stop();
		
		go();
		correct_way(13500);
		turn_left();
		correct_way(1000);
		turn_left();
		delay(14000);
		reverse();
		stop();
	*/
	/*
		go();
		correct_way(13500);
		turn_left();
		correct_way(4000);
		turn_left();
		correct_way(14000);
		turn_left();
		correct_way(4500);
		reverse();
		stop();
	
		go();
		delay(7000);
		turn_left();
		delay(1000);
		turn_left();
		correct_way();
		stop();
		
		go();
		delay(3000);
		turn_left();
		delay(1000);
		turn_left();
		delay(3000);
		reverse();
		stop();
	*/
	
	//Serial.println(get_degrees());
	/*
	while(true){
		turn(direct+50);
		delay(100);
		turn(direct-50);
		delay(100);
	}
	*/
}


void go(){
		
		analogWrite(m_speed, 120);
		delay(500);
		analogWrite(m_speed, 90);
		digitalWrite(dir, HIGH);
		
}
void stop(){analogWrite(m_speed, 0);}

void reverse(){
	digitalWrite(dir, LOW);
	analogWrite(m_speed, 100);
	delay(2500);
	analogWrite(m_speed, 0);
}


void turn_left(){
	Serial.println("turn left");
	digitalWrite(8,1);
	float start_deg = get_degrees();
	//Serial.println(start_deg);
	turn(180);
	float deg=0;
	while(deg<=90){
		deg = start_deg-get_degrees();
		if(deg<0)deg=-1*deg;
		if(deg>180)deg = 360-deg;
		deg *=1.14; 
	}
	turn(direct);
	digitalWrite(8,0);
}

void turn(int angle){
	servo.write(angle);
	delay(30);
}
float get_degrees(){
	startMillis = millis();
	accel.readGXYZ(&ax, &ay, &az);
	// считываем данные с гироскопа в радианах в секунду
	gyro.readRadPerSecXYZ(&gx, &gy, &gz);
	// считываем данные с компаса в Гауссах
	compass.readCalibrateGaussXYZ(&mx, &my, &mz);
	// устанавливаем коэффициенты фильтра
	filter.setKoeff(fps, BETA);
	// обновляем входные данные в фильтр
	filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
	// получение углов yaw, pitch и roll из фильтра
	yaw = filter.getYawDeg()+180;
	
	// вычисляем частоту обработки фильтра
	deltaMillis = millis() - startMillis;
	fps = 1000 / deltaMillis;
	return yaw;
}
void correct_way(int t){
	long startTime1 = millis();
	int lastSonar = getSonar(forward_sonar, true),nowSonar=0;
	while(millis()-startTime1<t){//nowSonar>60 ||nowSonar==0){
		nowSonar = getSonar(forward_sonar, false);
	
		float son2 = getSonar(rightBack_sonar, false), son1 = getSonar(rightForw_sonar, false);
		float sr = (son1+son2)/2;
		
		if(sr>20){
			turn(direct-15);
			Serial.println("correct right");
		}else if (sr<10){
			turn(direct+15);
			Serial.println("correct left");
		}else if(abs(son1-son2)<5){
			turn(direct);
			Serial.println("correct direct");
		}else if(son2<son1){
			turn(direct-10);
			Serial.println("correct left");
		}else {
			turn(direct+10);	
			Serial.println("correct right");
		}
		/*while (abs(lastSonar-nowSonar)>20){
			nowSonar = getSonar(forward_sonar, true);
			Serial.print(nowSonar);
			Serial.print(" raz ");
			Serial.println(abs(lastSonar-nowSonar));
		}*/
		delay(300);
		lastSonar=nowSonar;
	}
	digitalWrite(9,1);
	delay(50);
	digitalWrite(9,0);
	Serial.println("block");
}

int getSonar(NewPing sonar,boolean forw ){
int n=10; //от скольки показаний мы берем среднее
int sum =0, son;
for(int i=1;i<=n;i++){ 
	son=sonar.ping_cm();
	while(forw && son==0)son=sonar.ping_cm();
	sum += son;
}
return sum/n;
}
	
//float abs(float a){if(a<0)return -a;
//return a;}