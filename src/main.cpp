/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/
 * Arduino Mecanum Omni Direction Wheel Robot Car Lesson5 Wifi Control
 * Tutorial URL http://osoyoo.com/?p=30176
 * CopyRight www.osoyoo.com
 *
 */
#include <Arduino.h>

#include "WiFiEsp.h"
#include "secrets.h"

#define SPEED 63  // PWM 0-255
#define TURN_SPEED 67
#define SHIFT_SPEED 130

#define TURN_TIME 500
#define MOVE_TIME 500

#define speedPinR 9  //  RIGHT WHEEL PWM pin D45 connect front MODEL-X ENA
#define RightMotorDirPin1 \
  22  // Front Right Motor direction pin 1 to Front MODEL-X IN1  (K1)
#define RightMotorDirPin2 \
  24  // Front Right Motor direction pin 2 to Front MODEL-X IN2   (K1)
#define LeftMotorDirPin1 \
  26  // Left front Motor direction pin 1 to Front MODEL-X IN3 (  K3)
#define LeftMotorDirPin2 \
  28  // Left front Motor direction pin 2 to Front MODEL-X IN4 (  K3)
#define speedPinL 10  // Left WHEEL PWM pin D7 connect front MODEL-X ENB

#define speedPinRB 11  //  RIGHT WHEEL PWM pin connect Back MODEL-X ENA
#define RightMotorDirPin1B \
  5  // Rear Right Motor direction pin 1 to Back MODEL-X IN1 (  K1)
#define RightMotorDirPin2B \
  6  // Rear Right Motor direction pin 2 to Back MODEL-X IN2 (  K1)
#define LeftMotorDirPin1B \
  7  // Rear left Motor direction pin 1 to Back MODEL-X IN3  K3
#define LeftMotorDirPin2B \
  8  // Rear left Motor direction pin 2 to Back MODEL-X IN4  k3
#define speedPinLB 12  //   LEFT WHEEL  PWM pin D8 connect Rear MODEL-X ENB
/*motor control*/

int motorPwd = 63;

int diamondHigh = 90;

int diamondLow = diamondHigh * 3 / 4;

void FR_fwd(int speed)  // front-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  analogWrite(speedPinR, speed);
}
void FR_bck(int speed)  // front-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  analogWrite(speedPinR, speed);
}
void FL_fwd(int speed)  // front-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
  analogWrite(speedPinL, speed);
}
void FL_bck(int speed)  // front-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
  analogWrite(speedPinL, speed);
}

void RR_fwd(int speed)  // rear-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1B, HIGH);
  digitalWrite(RightMotorDirPin2B, LOW);
  analogWrite(speedPinRB, speed);
}
void RR_bck(int speed)  // rear-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B, HIGH);
  analogWrite(speedPinRB, speed);
}
void RL_fwd(int speed)  // rear-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1B, HIGH);
  digitalWrite(LeftMotorDirPin2B, LOW);
  analogWrite(speedPinLB, speed);
}
void RL_bck(int speed)  // rear-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B, HIGH);
  analogWrite(speedPinLB, speed);
}
void right_shift(int speed_fl_fwd, int speed_rl_bck, int speed_rr_fwd,
                 int speed_fr_bck) {
  FL_fwd(speed_fl_fwd);
  RL_bck(speed_rl_bck);
  FR_bck(speed_fr_bck);
  RR_fwd(speed_rr_fwd);
  ;
}
void left_shift(int speed_fl_bck, int speed_rl_fwd, int speed_rr_bck,
                int speed_fr_fwd) {
  FL_bck(speed_fl_bck);
  RL_fwd(speed_rl_fwd);
  FR_fwd(speed_fr_fwd);
  RR_bck(speed_rr_bck);
}
void go_advance(int speed) {
  RL_fwd(speed);
  RR_fwd(speed);
  FR_fwd(speed);
  FL_fwd(speed);
}
void go_back(int speed) {
  RL_bck(speed);
  RR_bck(speed);
  FR_bck(speed);
  FL_bck(speed);
}
void left_turn(int speed) {
  RL_bck(0);
  RR_fwd(speed);
  FR_fwd(speed);
  FL_bck(0);
}
void right_turn(int speed) {
  RL_fwd(speed);
  RR_bck(0);
  FR_bck(0);
  FL_fwd(speed);
}
void left_back(int speed) {
  RL_fwd(0);
  RR_bck(speed);
  FR_bck(speed);
  FL_fwd(0);
}
void right_back(int speed) {
  RL_bck(speed);
  RR_fwd(0);
  FR_fwd(0);
  FL_bck(speed);
}

void clockwise(int speed) {
  RL_fwd(speed);
  RR_bck(speed);
  FR_bck(speed);
  FL_fwd(speed);
}
void countclockwise(int speed) {
  RL_bck(speed);
  RR_fwd(speed);
  FR_fwd(speed);
  FL_bck(speed);
}

void stop_Stop()  // Stop
{
  analogWrite(speedPinLB, 0);
  analogWrite(speedPinRB, 0);
  analogWrite(speedPinL, 0);
  analogWrite(speedPinR, 0);
}

void printWifiStatus() {
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in the browser
  Serial.println();
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
  Serial.println();
}

// Pins initialize
void init_GPIO() {
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);

  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  pinMode(RightMotorDirPin1B, OUTPUT);
  pinMode(RightMotorDirPin2B, OUTPUT);
  pinMode(speedPinLB, OUTPUT);

  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT);
  pinMode(speedPinRB, OUTPUT);

  stop_Stop();
}

int readSpeed(WiFiEspClient client, RingBuffer buf) {
  int out = 0;
  while (client.connected()) {  // loop while the client's connected
    Serial.println("Client Connected");
    if (client.available()) {  // if there's bytes to read from the client,
      Serial.println("Client Available");
      char c = client.read();  // read a byte, then
      buf.push(c);             // push it to the ring buffer
      int chr = (int)c - 48;
      Serial.println(chr);
      if (chr < 0 || chr > 9) {  // break on invalid number
        client.println("HTTP/1.1 200 OK");
        client.println();
        return out;
      }
      out = out * 10 + chr;
    }
  }
  return out;
}

int status = WL_IDLE_STATUS;
WiFiEspServer server(80);
// use a ring buffer to increase speed and reduce memory allocation
RingBuffer buf(8);
void setup() {
  init_GPIO();
  Serial.begin(9600);  // initialize serial for debugging
  Serial1.begin(115200);
  Serial1.write("AT+UART_DEF=9600,8,1,0,0\r\n");
  delay(200);
  Serial1.write("AT+RST\r\n");
  delay(200);
  Serial1.begin(9600);  // initialize serial for ESP module
  WiFi.init(&Serial1);  // initialize ESP module

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true)
      ;
  }

  // attempt to connect to WiFi network
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  Serial.println("You're connected to the network");
  printWifiStatus();

  // start the web server on port 80
  server.begin();
}

void loop() {
  WiFiEspClient client = server.available();  // listen for incoming clients

  if (client) {                    // if you get a client,
    Serial.println("New client");  // print a message out the serial port
    buf.init();                    // initialize the circular buffer
    while (client.connected()) {   // loop while the client's connected
      if (client.available()) {    // if there's bytes to read from the client,
        char c = client.read();    // read a byte, then
        Serial.write(c);
        buf.push(c);  // push it to the ring buffer

        if (buf.endsWith("\r\n\r\n")) {
          client.println("HTTP/1.1 200 OK");
          client.println();
          break;
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (buf.endsWith("?a=S")) {
          Serial.println("Reading Speed");
          motorPwd = readSpeed(client, buf);
          break;
        } else if (buf.endsWith("?a=A")) {
          // Serial.println("go Ahead");
          go_advance(motorPwd);
        } else if (buf.endsWith("?a=B")) {
          go_back(motorPwd);
        } else if (buf.endsWith("?a=L")) {
          left_turn(motorPwd);
        } else if (buf.endsWith("?a=R")) {
          right_turn(motorPwd);
        } else if (buf.endsWith("?a=E")) {
          stop_Stop();
        } else if (buf.endsWith("?a=O")) {
          left_shift(diamondHigh, diamondLow, diamondLow, diamondHigh);
        } else if (buf.endsWith("?a=F")) {
          right_shift(diamondHigh, diamondLow, diamondLow, diamondHigh);
        } else if (buf.endsWith("?a=1")) {
          left_shift(0, diamondLow, 0, diamondLow);  // left ahead
        } else if (buf.endsWith("?a=4")) {
          left_shift(diamondLow, 0, diamondLow, 0);  // left back
        } else if (buf.endsWith("?a=3")) {
          right_shift(diamondLow, 0, diamondLow, 0);  // right ahead
        } else if (buf.endsWith("?a=6")) {
          right_shift(0, diamondLow, 0, diamondLow);  // right back
        }
      }
    }

    // close the connection
    client.stop();
    Serial.println("Client disconnected");
  }
}
