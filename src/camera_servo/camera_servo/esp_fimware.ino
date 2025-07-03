/*
回读所有舵机反馈参数:位置、速度、负载、电压、温度、移动状态、电流；
FeedBack函数回读舵机参数于缓冲区，Readxxx(-1)函数返回缓冲区中相应的舵机状态；
函数Readxxx(ID)，ID=-1返回FeedBack缓冲区参数；ID>=0，通过读指令直接返回指定ID舵机状态,
无需调用FeedBack函数。
Read back all feedback parameters: position, speed, load, voltage, temperature, movement status;
The FeedBack function reads back the servo parameters in the buffer, and the Readxxx (-1) function returns the corresponding servo state in the buffer;
Function Readxxx (ID), ID=1 returns the FeedBack buffer parameter; ID > 0, and directly returns the specified ID rudder state by reading the instruction.
There is no need to call the FeedBack function.
*/

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19
#include <SCServo.h>

SMS_STS sms_sts;

void setup(){
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  Serial.begin(115200);
  sms_sts.pSerial = &Serial1;
  delay(1000);
}

void loop(){
  if (Serial.available() > 0) {
    String input = Serial.readString();

    int spaceIndex = input.indexOf(' '); // Find the position of space

    if (spaceIndex != -1) {
      String part1 = input.substring(0, spaceIndex);              // "12.34"
      String part2 = input.substring(spaceIndex + 1);             // "56.78"

      int value1 = part1.toInt();  // 12.34
      int value2 = part2.toInt();  // 56.78
      sms_sts.WritePosEx(1, value1, 1500, 50);//servo(ID1) speed=3400，acc=50，move to position=4095.
      sms_sts.WritePosEx(2, value2, 1500, 50);//servo(ID1) speed=3400，acc=50，move to position=4095.
      delay(2000);

    }
  }
  int Pos;
  int Pos2;
  if(sms_sts.FeedBack(1)!=-1){
    Pos = sms_sts.ReadPos(1);
    Pos2 = sms_sts.ReadPos(2);
    Serial.print(Pos);
    Serial.print(' ');
    Serial.print(Pos2);
    Serial.print('\n');
    delay(100);
  }else{
    Serial.println("FeedBack err");
    delay(500);
  }
}
