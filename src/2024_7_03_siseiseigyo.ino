#include <MadgwickAHRSva.h>
#include "esp32-hal-ledc.h"
#include <M5Core2.h>


hw_timer_t * timer = NULL;

Madgwick MadgwickFilter;

int count_e = 0,f_m = 0;
unsigned long time_e = 0,time_pid = 0;

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

float ROLL = 0.0F;
float PITCH = 0.0F;
float YAW = 0.0F;

float dt,I,D,Pm;

float Kp = 0.5;
float Ki = 10000000000.0;
float Kd = 0.007;


float NTM = 49;
float out = 0;
int out2,fg = 0;
float count_e50 = 0;

float beta = 0.8;


void setup() {

  Serial.begin(115200);

  M5.begin();
  M5.IMU.Init();

  pinMode(35,INPUT);//エンコーダ
  pinMode(36,INPUT);//エンコーダ
  pinMode(27,OUTPUT);//ブレーキ
  pinMode(19,OUTPUT);//回転方向
  ledcSetup(1, 20000, 10);//PWM設定20khz　1024
  ledcAttachPin(26, 1);//PWM

  M5.Lcd.setTextColor(RED,DARKGREEN); 
  M5.Lcd.setTextSize(2);

  digitalWrite(27,LOW);//モータ停止
  ledcWrite(1,1024);//モータ停止

  attachInterrupt(35,E_count,RISING);//エンコーダ読み取り割り込み

  MadgwickFilter.begin(100);//マドウィックフィルタの動作速度　100Hz  
  MadgwickFilter.betaset(beta);//マドウィックフィルタのβの値

  delay(1000);

}

void loop() {

  M5.update();
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.IMU.getAccelData(&accX,&accY,&accZ);  
  M5.IMU.getAhrsData(&pitch,&roll,&yaw); 

  //pitch = pitch-0.5;
///////////////lcd表示
  M5.Lcd.setCursor(0, 25);
  M5.Lcd.fillRect(0, 25, 100, 25, BLACK);
  M5.Lcd.setCursor(0, 70);
  M5.Lcd.printf("%d", out2);
  M5.Lcd.setCursor(0, 120);
  M5.Lcd.printf("pitch");
  M5.Lcd.setCursor(0, 142);
  M5.Lcd.printf("%5.2fdeg", pitch);
  M5.Lcd.setCursor(0, 182);
  M5.Lcd.printf("%5.2f  %3d", PITCH, out2);
  
////////////////Teleplot用
  Serial.printf(">PITCH:%f\n",PITCH);
  Serial.printf(">Pm:%f\n",Pm);
  //Serial.printf(">pitchj_k:%f\n",(float)pitchj_k);
  Serial.printf(">I:%f\n",I);
  Serial.printf(">D:%f\n",D);
  Serial.printf(">out2:%d\n",out2);

/////////////////Madgwick filter////////////////////////////////////

  MadgwickFilter.updateIMU(gyroX, gyroY, gyroZ, accX, accY, accZ);
  ROLL  = MadgwickFilter.getRoll();
  PITCH = ((MadgwickFilter.getPitch()-6+1.6)*1.2+4)/100*90;//6~82
  YAW   = MadgwickFilter.getYaw();

  //////////////βの値を変えたい場合(ジャイロと加速度の割合を変える)はMadgwickAHRS.cpp内の29行目のbetaDefの値を変える///////
  //////////////β=0はジャイロセンサの単純積分で求めた姿勢角に対応し、βを大きくするにしたがい加速度による補正量が大きくなります。<---書いてあった////////
  
//////////////PID　など制御則　最終的な出力はoutという変数//////////////////////////

  dt = (millis() - time_pid) * 0.001;
  time_pid = millis();
  Pm = NTM - PITCH;
  I = I+Pm*dt;
  D = -1*gyroZ;

/////////////////////モータ停止状態から倒立点付近でモータ始動
  if((NTM-1)<pitch&&pitch<(NTM+1)){
    f_m = 0;
  }
/////////////////////傾きすぎたとき値リセット
  else if(f_m!=0){
    Pm = 0;
    I = 0;
    D = 0;
  }

  out2 = Kp*(Pm+I/Ki+Kd*D);
  out2 = constrain(out2,-1024,1024);

  if(out2<0){
    digitalWrite(19,HIGH);
    digitalWrite(27,HIGH); 
    out = 1024 - (-1 * out2);
    }
  else if(out2>0){
    digitalWrite(19,LOW);
    digitalWrite(27,HIGH);
    out = 1024 - out2;
    }
  else{
    digitalWrite(27,LOW);
    out = 1024;
    }

/////////////////////////////////////////////////////////////////////////////
////////////////////傾き度合いによるlcdの色の変化
  if((NTM-35) < pitch && pitch <(NTM+35)){
    M5.Lcd.setTextColor(YELLOW,DARKGREEN); 
    if((NTM-3)<pitch&&pitch<(NTM+3))
      M5.Lcd.setTextColor(GREEN,DARKGREEN);
  }
  else{
    M5.Lcd.setTextColor(RED,DARKGREEN); 
    digitalWrite(27,LOW);
    f_m = 1;
  }
//////////////////エンコーダ
  time_e = millis() - time_e;
  count_e50 = (count_e*360/50)/time_e;
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.printf("%d, %d, %.2f",time_e,count_e,count_e50);
  count_e = 0;
}

void E_count(){
  if(digitalRead(36)==LOW)count_e++;
  else count_e--;
}
