// Import library PS2
// Thư viện tay PS2 cho chế độ điều khiển bằng tay

#include <PS2X_lib.h>
// Define pin connect PS2 to arduino Mega
#define PS2_DAT        52
#define PS2_CMD        51
#define PS2_SEL        53
#define PS2_CLK        50
#define rumble true
#define pressures true
boolean run_derection = true;

// Create object PS2X
//Tạo đối tượng mới PS2X , khởi tạo chế độ kết nối.
PS2X ps2x;
int error = 0;
byte type = 0;
byte vibrate = 0;
int pwmRvalue = 255;
int pwmLvalue = 255;

//Define config motor
const int RightFrontMotor_PWM  = 2;   // ENA RF
const int LeftFrontMotor_PWM   = 3;   // ENA RF
const int RightRearMotor_PWM   = 4;   // ENA RR
const int LeftRearMotor_PWM    = 5;   // ENA LR
const int HandTopMotor_PWM     = 6;   // ENA RR
const int HandLeftMotor_PWM    = 7;   // ENA LR

//Front motors
const int RightFrontMotor_IN1  = 22;  // IN1 RF
const int RightFrontMotor_IN2  = 23;  // IN2 RF
const int LeftFrontMotor_IN1   = 24;  // IN1 LF
const int LeftFrontMotor_IN2   = 25;  // IN2 LF

//Rear motors
const int RightRearMotor_IN1   = 26;  // IN1 RR
const int RightRearMotor_IN2   = 27;  // IN2 RR
const int LeftRearMotor_IN1    = 28;  // IN1 LR
const int LeftRearMotor_IN2    = 29;  // IN2 LR

//Hands motor
const int HandTopMotor_IN1     = 30;  // IN1 HandMid
const int HandYTopMotor_IN2     = 31;  // IN2 HandMid
const int HandLeftMotor_IN1    = 32;  // IN1 HandLeft
const int HandLeftMotor_IN2    = 33;  // IN2 HandLeft

// Sensor
const int sensorTopRightEnd = 35; // 
const int sensorTopLeftEnd = 36;
const int sensorTopMidStart = 37;
const int sensorLeftStart = 38;
const int sensorLeftEnd = 39;

// bool sensor
bool isTopHit; // trang thai co the danh
bool isLeftHit;

const int speedHit = 25;
// Function setup for arduino
// Hàm cài đặt tín hiệu đầu vào đầu ra cho arduino Mega
void setup() {
  //
  Serial.begin(9600);
  //
  pinMode(RightFrontMotor_IN1, OUTPUT);
  pinMode(RightFrontMotor_IN2, OUTPUT);
  //
  pinMode(LeftFrontMotor_IN1, OUTPUT);
  pinMode(LeftFrontMotor_IN2, OUTPUT);
  //
  pinMode(RightRearMotor_IN1, OUTPUT);
  pinMode(RightRearMotor_IN2, OUTPUT);
  //
  pinMode(LeftRearMotor_IN1, OUTPUT);
  pinMode(LeftRearMotor_IN2, OUTPUT);
  //
  pinMode(HandMidMotor_IN1, OUTPUT);
  pinMode(HandMidMotor_IN2, OUTPUT);
  //
  pinMode(HandLeftMotor_IN1, OUTPUT);
  pinMode(HandLeftMotor_IN2, OUTPUT);
  
  // Reset các chân tín hiệu được sử dụng về mức LOW
  resetBeforeStart();
  // Cài đặt chế độ điều khiển bằng tay qua PS2
  setupPS2();
}

void setupPS2() {
  delay(300);
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  if (error == 0) {
    //Found Controller, configured successful
  }
  else if (error == 1) {
    //No controller found, check wiring .
  }
  else if (error == 2) {
    //Controller found but not accepting commands.
  }
  else if (error == 3) {
    //Controller refusing to enter Pressures mode.
  }
  type = ps2x.readType();
  switch (type) {
    case 0:
      //Unknown Controller type found.
      break;
    case 1:
      //DualShock Controller found.
      break;
    case 2:
      //GuitarHero Controller found.
      break;
    case 3:
      //Wireless Sony DualShock Controller found.
      break;
  }
}

void motorControl(String motorStr, int mdirection, int mspeed) {
  int IN1;
  int IN2;
  int motorPWM;

  if (motorStr == "rf") {       //right front
    IN1 = RightFrontMotor_IN1;
    IN2 = RightFrontMotor_IN2;
    motorPWM = RightFrontMotor_PWM;
  }
  else if (motorStr == "lf") { //left front
    IN1 = LeftFrontMotor_IN1;
    IN2 = LeftFrontMotor_IN2;
    motorPWM = LeftFrontMotor_PWM;
  }
  else if (motorStr == "rr") {
    IN1 = RightRearMotor_IN1;
    IN2 = RightRearMotor_IN2;
    motorPWM = RightRearMotor_PWM;
  }
  else if (motorStr == "lr") {
    IN1 = LeftRearMotor_IN1;
    IN2 = LeftRearMotor_IN2;
    motorPWM = LeftRearMotor_PWM;
  }
  else if (motorStr == "top") {
    IN1 = HandTopMotor_IN1;
    IN2 = HandTopMotor_IN2;
    motorPWM = HandTopMotor_PWM;
  }
  else if (motorStr == "left") {
    IN1 = HandLeftMotor_IN1;
    IN2 = HandLeftMotor_IN2;
    motorPWM = HandLeftMotor_PWM;
  }
  
  if (mdirection == 1) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else if (mdirection == -1) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  analogWrite(motorPWM, mspeed);
}

// Hàm di chuyển robot lên phía trước
void goForward(int mspeed) {
  motorControl("rf", 1, mspeed);
  motorControl("lf", 1, mspeed);
  motorControl("rr", 1, mspeed);
  motorControl("lr", 1, mspeed);
  run_derection = true;
  Serial.println("goForward");
}


// Hàm di chuyển robot lùi về phía sau
void goBackward(int mspeed) {
  motorControl("rf", -1, mspeed);
  motorControl("lf", -1, mspeed);
  motorControl("rr", -1, mspeed);
  motorControl("lr", -1, mspeed);
  run_derection = true;
  Serial.println("goBackwad");
}

// Hàm di chuyển robot sang phải.
void moveRight(int mspeed) {
  motorControl("rf", -1, mspeed);
  motorControl("lf", 1, mspeed);
  motorControl("rr", 1, mspeed);
  motorControl("lr", -1, mspeed);
  run_derection = true;
  Serial.println("moveRight");
}

// Hàm di chuyển robot sang trái.
void moveLeft(int mspeed) {
  motorControl("rf", 1, mspeed);
  motorControl("lf", -1, mspeed);
  motorControl("rr", -1, mspeed);
  motorControl("lr", 1, mspeed);
  run_derection = true;
  Serial.println("moveLeft");
}


// Hàm di chuyển robot sang phải lên phía trước.
void moveRightForward(int mspeed) {
  motorControl("rf", 1, 0);
  motorControl("lf", 1, mspeed);
  motorControl("rr", 1, mspeed);
  motorControl("lr", 1, 0);
  run_derection = true;
  Serial.println("moveRightForward");
}


// Hàm di chuyển robot sang phải lùi xuống phía sau.
void moveRightBackward(int mspeed) {
  motorControl("rf", -1, mspeed);
  motorControl("lf", 1, 0);
  motorControl("rr", 1, 0);
  motorControl("lr", -1, mspeed);
  run_derection = true;
  Serial.println("moveRightBackward");
}


// Hàm di chuyển robot sang trái lên phía trước.
void moveLeftForward(int mspeed) {
  motorControl("rf", 1, mspeed);
  motorControl("lf", 1, 0);
  motorControl("rr", 1, 0);
  motorControl("lr", 1, mspeed);
  run_derection = true;
  Serial.println("moveLeftForward");
}


// Hàm di chuyển robot sang trái lùi xuống phía sau.
void moveLeftBackward(int mspeed) {
  motorControl("rf", 1, 0);
  motorControl("lf", -1, mspeed);
  motorControl("rr", -1, mspeed);
  motorControl("lr", 1, 0);
  run_derection = true;
  Serial.println("moveLeftBackward");
}


// Hàm di chuyển robot rẽ sang phải.
void turnRight(int mspeed) {
  motorControl("rf", -1, mspeed);
  motorControl("lf", 1, mspeed);
  motorControl("rr", -1, mspeed);
  motorControl("lr", 1, mspeed);
  run_derection = true;
  Serial.println("turnRight");
}


// Hàm di chuyển robot rẽ sang trái.
void turnLeft(int mspeed) {
  motorControl("rf", 1, mspeed);
  motorControl("lf", -1, mspeed);
  motorControl("rr", 1, mspeed);
  motorControl("lr", -1, mspeed);
  run_derection = true;
  Serial.println("turnLeft");
}


// Hàm dừng robot.
void stopRobot(int delay_ms) {
  analogWrite(RightFrontMotor_PWM, 0);
  analogWrite(LeftFrontMotor_PWM, 0);
  analogWrite(RightRearMotor_PWM, 0);
  analogWrite(LeftRearMotor_PWM, 0);
  delay(delay_ms);
  run_derection = false;
  Serial.println("stopRobot");
}


// Hàm dừng cứng robot.
void hardStop() {
  analogWrite(RightFrontMotor_PWM, 0);
  analogWrite(LeftFrontMotor_PWM, 0);
  analogWrite(RightRearMotor_PWM, 0);
  analogWrite(LeftRearMotor_PWM, 0);
  run_derection = false;
  Serial.println("hardStop");
}


// Hàm set xung
void SetPWM (const long pwm_num, byte pwm_channel) {
  if (pwm_channel == 1) { // DRIVE MOTOR
    analogWrite(RightFrontMotor_PWM, pwm_num);
    pwmRvalue = pwm_num;
  }
  else if (pwm_channel == 2) { // STEERING MOTOR
    analogWrite(LeftFrontMotor_PWM, pwm_num);
    pwmLvalue = pwm_num;
  }
}

boolean isSensor(int sensorPin) {
  return digitalRead(sensorPin);
}

void HitBadminton(String nameMotor, int dir, int motorSpeed, int sensorStartPin, int sensorEndPin) {
  if (isSensor(sensorStartPin)){
    motorControl(nameMotor, dir, motorSpeed);
  }
  
}
void loop() {
  //  Check found PS2 Connect
  //  Kiểm tra kết nối với tay PS2
  // khi gặp lỗi kết nối thoát khỏi chương trình điều khiển
  if (error == 1) //skip loop if no controller found
    return;
  // Thực hiện đọc tín hiệu từ tay PS2 truyền tới
  if (type == 2) {
    // Nếu kiểu tay điều khiển là : Guitar Hero Controller
    ps2x.read_gamepad();          //read controller
    if (ps2x.ButtonPressed(GREEN_FRET)) ;  // Green Fret Pressed
    if (ps2x.ButtonPressed(RED_FRET))   ;  // Red Fret Pressed
    if (ps2x.ButtonPressed(YELLOW_FRET)) ; // Yellow Fret Pressed
    if (ps2x.ButtonPressed(BLUE_FRET))  ;  // Blue Fret Pressed
    if (ps2x.ButtonPressed(ORANGE_FRET)) ; // Orange Fret Pressed
    if (ps2x.ButtonPressed(STAR_POWER))  ; // Star Power Command
    if (ps2x.Button(UP_STRUM))         ;   // Up Strum
    if (ps2x.Button(DOWN_STRUM))      ;    // DOWN Strum
    if (ps2x.Button(PSB_START))      ;     // Start is being held
    if (ps2x.Button(PSB_SELECT))    ;      // Select is being held
    if (ps2x.Button(ORANGE_FRET))  ;       // Wammy Bar Position: ps2x.Analog(WHAMMY_BAR)
  }
  else {
    // Nếu kiểu tay điều khiển là : DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    if (ps2x.Button(PSB_START))  {      //will be TRUE as long as button is pressed
      Serial.println("Start is being held");
    }
    if (ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");

    if (ps2x.Button(PSB_PAD_UP)) {     //will be TRUE as long as button is pressed
      goForward(100);
      // PSB_PAD_UP
    }
    if (ps2x.Button(PSB_PAD_DOWN)) {
      goBackward(100);
      // PSB_PAD_DOWN
    }
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      moveRight(100);
      // PSB_PAD_RIGHT
    }
    if (ps2x.Button(PSB_PAD_LEFT)) {
      moveLeft(100);
      // PSB_PAD_LEFT
    }


    vibrate = ps2x.Analog(PSAB_CROSS);
    if (ps2x.NewButtonState()) {
      if (ps2x.Button(PSB_L3))                           // N/A
        Serial.println("L3 pressed");
      if (ps2x.Button(PSB_R3))                           // N/A
        Serial.println("R3 pressed");
      if (ps2x.Button(PSB_L2))                           // N/A
        Serial.println("L2 pressed");
      if (ps2x.Button(PSB_R2))                           // N/A
        Serial.println("R2 pressed");
      if (ps2x.Button(PSB_TRIANGLE))                     // N/A
        Serial.println("Triangle pressed");
    }

    if (ps2x.ButtonPressed(PSB_CIRCLE) && !isTopHit) {               // Tay vợt chính giữa vụt bên phải.
      Serial.println("Circle just pressed");
      isTopHit = true;
      motor("top", 1, speedHit);
      
      while(isTopHit)){
        if(isSensor(sensorTopRightEnd)){
          isTopHit = false;
        }
      }
      motor("top", 1, 0);
      delay(500);
      motor("top", -1, speedHit/2);
      delay(500);
      motor("top", 1, 0);
//      while(!isTopHit && !isSensor(sensorTopRightStart)){
//        if(isSensor(sensorTopRightStart)){
//          motor("top", 1, 0);
//        }
//      }
    }
    if (ps2x.ButtonPressed(PSB_TRIANGLE))             // Tay vợt chính giữa vụt bên trái.
      Serial.println("Circle just pressed");
    if (ps2x.NewButtonState(PSB_CROSS))                 // N/A
      Serial.println("X just changed");
    if (ps2x.ButtonReleased(PSB_SQUARE))                // Tay vợt bên trái đánh cầu
      Serial.println("Square just released");

    if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
      int lx = ps2x.Analog(PSS_LX);
      int ly = ps2x.Analog(PSS_LY);
      int rx = ps2x.Analog(PSS_RX);
      int ry = ps2x.Analog(PSS_RY);
      if (ly < 80) {
        if (rx < 80) {
          moveLeftForward(100);                       // Di chuyển sang trái lên phía trước.
        }
        else if (rx > 200 ) {
          moveRightForward(100);                      // Di chuyển sang phải lên phía trước.
        }
        else {
          goForward(100);                             // Di chuyển lên phía trước.
        }
      }
      else if (ly > 200) {
        if (rx < 80) {
          moveLeftBackward(100);                      //  Di chuyển sang trái lùi xuống dưới.
        }
        else if (rx > 200 ) {
          moveRightBackward(100);                     //  Di chuyển sang phải lùi xuống dưới.
        }
        else {
          goBackward(100);                            // Di chuyển lùi xuống dưới.
        }
      } else {
        if (rx < 80) {
          moveLeft(100);                              // Di chuyển sang trái.
        }
        else if (rx > 200 ) {
          moveRight(100);                             // Di chuyển sang phải.
        }
        else {
          stopRobot(100);                             // Dừng di chuyển.
        }
      }
    }
  }
  if (!ps2x.Button(PSB_PAD_UP) && !ps2x.Button(PSB_PAD_DOWN) && !ps2x.Button(PSB_PAD_LEFT) && !ps2x.Button(PSB_PAD_RIGHT) && !ps2x.Button(PSB_L1) && !ps2x.Button(PSB_R1) && run_derection) {
    stopRobot(50);
  }
  delay(50);
}

// Hàm reset gí trị đầu vào khi bắt đầu .
void resetBeforeStart() {
  digitalWrite(RightFrontMotor_IN1, LOW);
  digitalWrite(RightFrontMotor_IN2, LOW);
  //
  digitalWrite(LeftFrontMotor_IN1, LOW);
  digitalWrite(LeftFrontMotor_IN2, LOW);
  //
  digitalWrite(RightRearMotor_IN1, LOW);
  digitalWrite(RightRearMotor_IN2, LOW);
  //
  digitalWrite(LeftRearMotor_IN1, LOW);
  digitalWrite(LeftRearMotor_IN2, LOW);
  //
  digitalWrite(HandMidMotor_IN1, LOW);
  digitalWrite(HandMidMotor_IN2, LOW);
  //
  digitalWrite(HandLeftMotor_IN1, LOW);
  digitalWrite(HandLeftMotor_IN2, LOW);
  //
  digitalWrite(RightFrontMotor_PWM, LOW);
  digitalWrite(LeftFrontMotor_PWM, LOW);
  digitalWrite(RightRearMotor_PWM, LOW);
  digitalWrite(LeftRearMotor_PWM, LOW);
  digitalWrite(HandMidMotor_PWM, LOW);
  digitalWrite(HandLeftMotor_PWM, LOW);
}


// Hàm đánh vợt................
