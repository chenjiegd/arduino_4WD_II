#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

float Kp = 80, Ki = 0, Kd = 9000;//
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int sensor[3] = {0, 0, 0};
int initial_motor_speed = 40;

const int key = 7; //按键key

void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);
void keyscan(void);
void Clear_All_PWM(void);

void setup()
{
	Serial.begin(9600);
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

void loop()
{
	read_sensor_values();
	calculate_pid();
	motor_control();
}

void read_sensor_values()
{
	// sensor[0] = digitalRead(A2);
	// sensor[1] = digitalRead(A1);
	// sensor[2] = digitalRead(A0);
	sensor[0] = analogRead(A0);
	sensor[1] = analogRead(A1);
	sensor[2] = analogRead(A2);
	if (sensor[0] > 40)
	{
		sensor[0] = 1;
	}
	else
	{
		sensor[0] = 0;
	}
	if (sensor[1] > 40)
	{
		sensor[1] = 1;
	}
	else
	{
		sensor[1] = 0;
	}
	if (sensor[2] > 40)
	{
		sensor[2] = 1;
	}
	else
	{
		sensor[2] = 0;
	}

	if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1))
	{
		error = 2;
	}
	else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1))
	{
		error = 1;
	}
	else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0))
	{
		error = 0;
	}
	else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0))
	{
		error = -1;
	}
	else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0))
	{
		error = -2;
	}
	else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0))
	{
		Clear_All_PWM();
	}
}

void calculate_pid()
{
	P = error;
	I = I + previous_I;
	D = error - previous_error;

	PID_value = (Kp * P) + (Ki * I) + (Kd * D);
	// Serial.println(PID_value);

	previous_I = I;
	previous_error = error;
}

void motor_control()
{
	// Calculating the effective motor speed:
	int left_motor_speed = initial_motor_speed - PID_value;
	int right_motor_speed = initial_motor_speed + PID_value;

	// The motor speed should not exceed the max PWM value
	left_motor_speed = constrain(left_motor_speed, -255, 255);
	right_motor_speed = constrain(right_motor_speed, -255, 255);

	// analogWrite(9, initial_motor_speed - PID_value);  //Left Motor Speed
	// analogWrite(10, initial_motor_speed + PID_value); //Right Motor Speed
	// run(initial_motor_speed - PID_value, initial_motor_speed + PID_value);
	run(left_motor_speed, right_motor_speed);

	//following lines of code are to make the bot move forward
	/*The pin numbers and high, low values might be different
    depending on your connections */
	// digitalWrite(4, HIGH);
	// digitalWrite(5, LOW);
	// digitalWrite(6, LOW);
	// digitalWrite(7, HIGH);
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
void run(float Speed1, float Speed2)
{
	Speed1 = map(Speed1, -255, 255, -4095, 4095);
	Speed2 = map(Speed2, -255, 255, -4095, 4095);
	if (Speed2 > 0)
	{
		pwm.setPWM(10, 0, Speed2); //右前
		pwm.setPWM(11, 0, 0);
		pwm.setPWM(8, 0, Speed2); //右后
		pwm.setPWM(9, 0, 0);
	}
	else
	{
		pwm.setPWM(10, 0, 0); //右前
		pwm.setPWM(11, 0, -Speed2);
		pwm.setPWM(8, 0, 0); //右后
		pwm.setPWM(9, 0, -Speed2);
	}
	if (Speed1 > 0)
	{
		pwm.setPWM(13, 0, Speed1); //左前
		pwm.setPWM(12, 0, 0);
		pwm.setPWM(15, 0, Speed1); //左后
		pwm.setPWM(14, 0, 0);
	}
	else
	{
		pwm.setPWM(13, 0, 0); //左前
		pwm.setPWM(12, 0, -Speed1);
		pwm.setPWM(15, 0, 0); //左后
		pwm.setPWM(14, 0, -Speed1);
	}

	// pwm.setPWM(10, 0, Speed2); //右前
	// pwm.setPWM(11, 0, 0);
	// pwm.setPWM(8, 0, Speed2); //右后
	// pwm.setPWM(9, 0, 0);

	// pwm.setPWM(13, 0, Speed1); //左前
	// pwm.setPWM(12, 0, 0);
	// pwm.setPWM(15, 0, Speed1); //左后
	// pwm.setPWM(14, 0, 0);
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
