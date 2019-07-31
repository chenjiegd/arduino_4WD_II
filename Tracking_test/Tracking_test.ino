#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

int initial_motor_speed = 100;
int sensor[3];

const int key = 7; //按键key

void keyscan(void);
void Clear_All_PWM(void);

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

void loop()
{
  read_sensor_values();
}

void read_sensor_values()
{
  sensor[0] = analogRead(A0);
  sensor[1] = analogRead(A1);
  sensor[2] = analogRead(A2);
  if(sensor[0]>40){
    sensor[0] = 1;
  }
  else{
    sensor[0] = 0;
  }
  if(sensor[1]>40){
    sensor[1] = 1;
  }
  else{
    sensor[1] = 0;
  }
  if(sensor[2]>40){
    sensor[2] = 1;
  }
  else{
    sensor[2] = 0;
  }
  

  if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1))
  {
    sleft(initial_motor_speed);
  }
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1))
  {
    left(initial_motor_speed);
  }
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0))
  {
    run(initial_motor_speed);
  }
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0))
  {
    right(initial_motor_speed);
  }
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0))
  {
    sright(initial_motor_speed);
  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0))
  {
    Clear_All_PWM();
  }
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1))
  {
    Clear_All_PWM();
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
void run(float Speed)
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
void left(float Speed)
{
  Speed = map(Speed, 0, 255, 0, 4095);
  pwm.setPWM(10, 0, Speed); //右前
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, Speed); //右后
  pwm.setPWM(9, 0, 0);

  pwm.setPWM(13, 0, 0); //左前
  pwm.setPWM(12, 0, 0);
  pwm.setPWM(15, 0, 0); //左后
  pwm.setPWM(14, 0, 0);
}
void right(float Speed)
{
  Speed = map(Speed, 0, 255, 0, 4095);
  pwm.setPWM(10, 0, 0); //右前
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, 0); //右后
  pwm.setPWM(9, 0, 0);

  pwm.setPWM(13, 0, Speed); //左前
  pwm.setPWM(12, 0, 0);
  pwm.setPWM(15, 0, Speed); //左后
  pwm.setPWM(14, 0, 0);
}
void sleft(float Speed)
{
  Speed = map(Speed, 0, 255, 0, 4095);
  pwm.setPWM(10, 0, Speed); //右前
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, Speed); //右后
  pwm.setPWM(9, 0, 0);

  pwm.setPWM(13, 0, 0); //左前
  pwm.setPWM(12, 0, Speed);
  pwm.setPWM(15, 0, 0); //左后
  pwm.setPWM(14, 0, Speed);
}
void sright(float Speed)
{
  Speed = map(Speed, 0, 255, 0, 4095);
  pwm.setPWM(10, 0, 0); //右前
  pwm.setPWM(11, 0, Speed);
  pwm.setPWM(8, 0, 0); //右后
  pwm.setPWM(9, 0, Speed);

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
  while (val == HIGH)   //当按键没被按下时，一直循环
  {
    val = digitalRead(key); //此句可省略，可让循环跑空
  }
  while (val == LOW) //当按键被按下时
  {
    delay(1);       //延时10ms
    val = digitalRead(key); //读取数字7口电平值赋给val
    while (val == HIGH)   //判断按键是否被松开
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
