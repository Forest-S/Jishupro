#include <TimerOne.h>

#define dRotAPin  2
#define dRotBPin  3
#define valvePin1 7 //下
#define valvePin2 8 //上
#define motPinIN1 5
#define motPinIN2 6
#define pRefPin 0
#define rot_step 100 // rot/step
int cnt = 0;
int R_count   = 0;
int i = 0;
char input[30];

// ロータリーエンコーダーの状態を記憶する
// 割り込み中に変化する変数はvolatileはをつけて宣言する
volatile long m_nValue  = 0;
volatile int oldf = 0;
volatile int diff_rot = 0;

void setup()
{
  // ピンの設定
  pinMode(13,OUTPUT);
  //エンコーダ
  pinMode(dRotAPin, INPUT);
  pinMode(dRotBPin, INPUT);
  // プルアップを有効に
  digitalWrite(dRotAPin, HIGH);
  digitalWrite(dRotBPin, HIGH);
  //割込み設定
  attachInterrupt(0, rotRotEnc, CHANGE);
  attachInterrupt(1, rotRotEnc, CHANGE);

  //バルブ
  pinMode(valvePin1, OUTPUT);
  pinMode(valvePin2, OUTPUT);

  //モーター
  pinMode(motPinIN1, OUTPUT);
  digitalWrite(motPinIN1, LOW);
  pinMode(motPinIN2, OUTPUT);
  digitalWrite(motPinIN2, LOW);

  //タイマー割り込み設定
  Timer1.initialize(10000);
  Timer1.attachInterrupt(timer_10ms);
  //シリアル通信速度を設定
  Serial.begin(9600);
}

void loop()
{
  //シリアル通信の受信
  if (Serial.available()) {
    input[i] = Serial.read();
    //30文字以上or末尾文字で終了
    if (i > 30 || input[i] == '.') {
      input[i] = '\0';
      if(String(input) == "LEDon"){
         digitalWrite(13,HIGH);
      }
      char operand[30],value_s[10];
      sscanf(input,"%[^,],%[^,]",operand, value_s);
      int value = atoi(value_s);
      if(strcmp(operand,"LED") == 0) {digitalWrite(13,value);Serial.println(value);}
      if(strcmp(operand,"valve1") == 0) {
         digitalWrite(valvePin1,value);
      }
      if(strcmp(operand,"valve2") == 0) {
         digitalWrite(valvePin2,value);
      }
      if(strcmp(operand,"mot") == 0) {
         MotorDrive(value);
      }
      if(strcmp(operand,"init") == 0) {
        init_theta(value);
      }

       i = 0;
    }
    else { i++; }
  }
}

void timer_10ms() {
  //回転速度計算
  static long old_rot = 0;
  diff_rot = m_nValue - old_rot;
  old_rot = m_nValue;
  if(cnt == 100) {
   //Serial.println(m_nValue);
   cnt = 0;
  }
cnt++;  
}

void init_theta(int pow) {
  MotorDrive(pow);
  while(analogRead(pRefPin) < 980) {
  }
  MotorDrive(0);
  delay(100);
  if(analogRead(pRefPin) < 980) {
     if(pow>0) {init_theta(-60);}
     else{init_theta(60);}
  }
}

void rotRotEnc(void)
{
  int APin = digitalRead(dRotAPin);
  int BPin = digitalRead(dRotBPin);
  int f = (APin << 1) xor BPin;
  //同じABが何度か出力されるので、その防止
  if (oldf != f) {
    int D = ((oldf << 1) xor f) >> 1;
    if ((D == 1) || (D == 3)) {
      //Serial.println('R');
      m_nValue++;
    } else {
      //Serial.println('L');
      m_nValue--;
    }
    oldf = f;
  }
}

void MotorDrive(int duty) {
  if (duty > 255) duty = 255;
  if (duty < -255) duty = -255;
  if (duty > 0) {
    analogWrite(motPinIN1, duty);
    digitalWrite(motPinIN2, LOW);
  } else if (duty < 0) {
    digitalWrite(motPinIN1, LOW);
    analogWrite(motPinIN2, -duty);
  } else {
    digitalWrite(motPinIN1, LOW);
    digitalWrite(motPinIN2, LOW);
  }
}

