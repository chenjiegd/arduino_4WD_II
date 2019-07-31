/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         arduino_4WD_McNamee-wheel_Bluetooth_control_OLED
* @author       wusicaijuan
* @version      V1.0
* @date         2019.07.01
* @brief        蓝牙控制arduino4WD小车
* @details
* @par History  见如下说明
*
*/
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const int adcal[3] = {35, 35, 35};
int sensor[3];

/*小车初始速度控制*/
int CarSpeedControl = 50;
const int key = 7; //按键key

void setup()
{
	// put your setup code here, to run once:
	//初始化电机驱动IO为输出方式
	pwm.begin();
	pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
	Clear_All_PWM();

	pinMode(A0, INPUT);
	pinMode(A1, INPUT);
	pinMode(A2, INPUT);
	pinMode(key, INPUT); //定义按键输入脚
	keysacn();
}

int pos;
void loop()
{
	// put your main code here, to run repeatedly:
	pos = echoTrace();
  	stateMachine(pos);
}

//PID算法部分
float Kp = 10; // 25
float Ki = 0;   // 0.15
float Kd = 0;   //1200
float error, errorLast, erroInte;
float calcPid(float input)
{
	float errorDiff;
	float output;
	error = error * 0.7 + input * 0.3; // filter
	//error = input;
	errorDiff = error - errorLast;
	erroInte = constrain(erroInte + error, -50, 50);
	output = Kp * error + Ki * erroInte + Kd * errorDiff;
	/*
	Serial.print(error); Serial.print(' ');
	Serial.print(erroInte); Serial.print(' ');
	Serial.print(errorDiff); Serial.print(' ');
	Serial.println(output);
	*/
	errorLast = error;

	return output;
}

int echoTrace()
{
	// sensor[0] = analogRead(A2);
	// sensor[1] = analogRead(A1);
	// sensor[2] = analogRead(A0);
	// int ret = 0;
	// // int a[3];
	// for (int i = 0; i < 3; i++)
	// {
	// 	// sensor[i] = constrain((1025 - analogRead(A2 - i)) / 10 - 4, 0, 20);
	// 	if (sensor[i] > adcal[i])
	// 		ret += (0x1 << i);
	// }
	// return ret;

	int ret = 0;
	int a[3];
	for (int i = 0; i < 3; i++) {
		a[i] = constrain((1025 - analogRead(A0 + i)) / 10 - 4, 0, 20);
		if (a[i] > 2) ret += (0x1 << i);
	}
	return ret;
}

int bias = 0;
int outlineCnt = 0;

void stateMachine(int a)
{
	switch (a)
	{
	case B000:
		outlineCnt++;
		break;
	case B111:
		outlineCnt++;
		break;
	case B001:
		outlineCnt = 0;
		bias = 2;
		break;
	case B011:
		outlineCnt = 0;
		bias = 1;
		break;
	case B010:
		outlineCnt = 0;
		bias = 0;
		break;
	case B110:
		outlineCnt = 0;
		bias = -1;
		break;
	case B100:
		outlineCnt = 0;
		bias = -2;
		break;
	default:
		Serial.println(a, BIN);
		outlineCnt++;
		break;
	}

	if (outlineCnt > 10)
	{
		doDcSpeed(0, 0);
	}
	else
	{
		float ff = 50;
		float ctrl = calcPid(bias);
		doDcSpeed(ff - ctrl, ff + ctrl);
	}
}

void doDcSpeed(int spdL, int spdR)
{
	spdL = map(spdL, 0, 255, 0, 4095);
	spdR = map(spdR, 0, 255, 0, 4095);
	// spdR = -spdR;
	if (spdL < 0)
	{
		// analogWrite(5, 0);
		// analogWrite(6, -spdL);
		pwm.setPWM(13, 0, spdL); //左前
		pwm.setPWM(12, 0, 0);
		pwm.setPWM(15, 0, spdL); //左后
		pwm.setPWM(14, 0, 0);
	}
	else
	{
		// analogWrite(5, spdL);
		// analogWrite(6, 0);
		pwm.setPWM(13, 0, spdL); //左前
		pwm.setPWM(12, 0, 0);
		pwm.setPWM(15, 0, spdL); //左后
		pwm.setPWM(14, 0, 0);
	}

	if (spdR < 0)
	{
		// analogWrite(9, 0);
		// analogWrite(10, -spdR);
		pwm.setPWM(10, 0, spdR); //右前
		pwm.setPWM(11, 0, 0);
		pwm.setPWM(8, 0, spdR); //右后
		pwm.setPWM(9, 0, 0);
	}
	else
	{
		// analogWrite(9, spdR);
		// analogWrite(10, 0);
		pwm.setPWM(10, 0, spdR); //右前
		pwm.setPWM(11, 0, 0);
		pwm.setPWM(8, 0, spdR); //右后
		pwm.setPWM(9, 0, 0);
	}
}

/**
* Function       run
* @author        wusicaijuan
* @date          2019.06.25
* @brief         小车前进
* @param[in]     Speed
* @param[out]    void
* @retval        void
* @par History   无
*/
void run(int Speed)
{
	Speed = map(Speed, 0, 255, 0, 4095);
	pwm.setPWM(10, 0, Speed); //右前
	pwm.setPWM(11, 0, 0);
	pwm.setPWM(8, 0, Speed); //右后
	pwm.setPWM(9, 0, 0);

	pwm.setPWM(13, 0, Speed); //左前
	pwm.setPWM(12, 0, 0);
	pwm.setPWM(15, 0, Speed); //左后
	pwm.setPWM(14, 0, 0);
}

/**
* Function       keysacn
* @author        wusicaijuan
* @date          2019.06.04
* @brief         按键扫描
* @param[in1]    void
* @retval        void
* @par History   无
*/
void keysacn()
{
	int val;
	val = digitalRead(key); //读取数字7口电平值赋给val
	while (val == HIGH)		//当按键没被按下时，一直循环
	{
		val = digitalRead(key); //此句可省略，可让循环跑空
	}
	while (val == LOW) //当按键被按下时
	{
		delay(1);				//延时10ms
		val = digitalRead(key); //读取数字7口电平值赋给val
		while (val == HIGH)		//判断按键是否被松开
		{
			break;
		}
	}
}

/*
* Function       Clear_All_PWM
* @author        wusicaijuan
* @date          2019.07.04
* @brief         关闭所有PWM
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void Clear_All_PWM()
{
	for (int i = 0; i < 16; i++)
	{
		pwm.setPWM(i, 0, 0);
	}
}
