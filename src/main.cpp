#include <MeAuriga.h>

#define USE_DEBUG
#define USE_encodernew
#define USE_encoderonboard
//#define USE_audioplayer
//#define USE_linefollower
//#define USE_linefollower
#define USE_ledmatrix
//#define USE_lightsensor
//#define USE_temperature
//#define USE_gyro
//#define USE_buzzer
#define USE_rgbledring
//#define USE_SoundSensor

// create an object named encodernew
  // 1. void MeEncoderNew::begin(void);
  // 2. void MeEncoderNew::setAddr(uint8_t i2cAddr,uint8_t slot);
  // 3. void MeEncoderNew::move(long angle, float speed, float lock_state);
  // 4. void MeEncoderNew::moveTo(long angle, float speed,float lock_state);
  // 5. void MeEncoderNew::runSpeed(int speed);
  // 6. void MeEncoderNew::runTurns(long turns, float speed,float lock_state);
  // 7. void MeEncoderNew::reset(void);
  // 8. void MeEncoderNew::setSpeedPID(float p,float i,float d);
  // 9. void MeEncoderNew::setPosPID(float p,float i,float d);
  // 10. void MeEncoderNew::setMode(uint8_t mode);
  // 11. void MeEncoderNew::setPWM(int pwm);
  // 12. void MeEncoderNew::setCurrentPosition(long pulse_counter)
  // 13. long MeEncoderNew::getCurrentPosition();
  // 14. void MeEncoderNew::getSpeedPID(float * p,float * i,float * d);
  // 15. void MeEncoderNew::getPosPID(float * p,float * i,float * d);
  // 16. float MeEncoderNew::getCurrentSpeed(void);
  // 17. void MeEncoderNew::sendCmd(void);
  // 18. float MeEncoderNew::getRatio(void);
  // 19. void MeEncoderNew::setRatio(float r);
  // 20. int MeEncoderNew::getPulse(void);
  // 21. void MeEncoderNew::setPulse(int p);
  // 22. void MeEncoderNew::setDevid(int devid);
  // 23. void MeEncoderNew::runSpeedAndTime(float speed, float time, float lock_state);
  // 24. boolean MeEncoderNew::isTarPosReached(void);
#ifdef USE_encodernew
  #include <Wire.h>
  MeEncoderNew motor3(PORT_9, SLOT1);
  //MeEncoderNew motor2(PORT_9, SLOT2);
#endif

// create an object named encoderonboard
  // 1. void MeEncoderOnBoard::reset(uint8_t slot);
  // 2. uint8_t MeEncoderOnBoard::getSlotNum(void);
  // 3. uint8_t MeEncoderOnBoard::getIntNum(void);
  // 4. uint8_t MeEncoderOnBoard::getPortA(void);
  // 5. uint8_t MeEncoderOnBoard::getPortB(void);
  // 6. long MeEncoderOnBoard::getPulsePos(void);
  // 7. void MeEncoderOnBoard::setPulsePos(long pulse_pos);
  // 8. void MeEncoderOnBoard::pulsePosPlus(void);
  // 9. void MeEncoderOnBoard::pulsePosMinus(void);
  // 10. void MeEncoderOnBoard::setCurrentSpeed(float speed);
  // 11. float MeEncoderOnBoard::getCurrentSpeed(void);
  // 12. int16_t MeEncoderOnBoard::getCurPwm(void);
  // 13. void MeEncoderOnBoard::setTarPWM(int16_t pwm_value);
  // 14. void MeEncoderOnBoard::setMotorPwm(int16_t pwm);
  // 15. void MeEncoderOnBoard::updateSpeed(void);
  // 16. void MeEncoderOnBoard::updateCurPos(void);
  // 17. long MeEncoderOnBoard::getCurPos(void)
  // 18. void MeEncoderOnBoard::runSpeed(float speed);
  // 19. void MeEncoderOnBoard::setSpeed(float speed);
  // 20. void MeEncoderOnBoard::move(long position,float speed,int16_t extId,cb callback);
  // 21. void MeEncoderOnBoard::moveTo(long position,float speed,int16_t extId,cb callback);
  // 22. long MeEncoderOnBoard::distanceToGo(void);
  // 23. void MeEncoderOnBoard::setSpeedPid(float p,float i,float d);
  // 24. void MeEncoderOnBoard::setPosPid(float p,float i,float d);
  // 25. void MeEncoderOnBoard::setPulse(int16_t pulseValue);
  // 26. void MeEncoderOnBoard::setRatio(int16_t RatioValue);
  // 27. void MeEncoderOnBoard::setMotionMode(int16_t motionMode);
  // 28. int16_t MeEncoderOnBoard::pidPositionToPwm(void);
  // 29. int16_t MeEncoderOnBoard::speedWithoutPos(void);
  // 30. void MeEncoderOnBoard::encoderMove(void);
  // 31. void MeEncoderOnBoard::pwmMove(void);
  // 32. boolean MeEncoderOnBoard::isTarPosReached(void);
  // 33. void MeEncoderOnBoard::loop(void);
#ifdef USE_encoderonboard
  MeEncoderOnBoard motor1(SLOT2);
  MeEncoderOnBoard motor2(SLOT1);
  void isr_process_encoder1(void) // count the ticks - i.e. how far the motor has moved
  {
    if(digitalRead(motor1.getPortB()) == 0)
    {
      motor1.pulsePosMinus();
    }
    else
    {
      motor1.pulsePosPlus();;
    }
  }
  void isr_process_encoder2(void) // count the ticks - i.e. how far the motor has moved
  {
    if(digitalRead(motor2.getPortB()) == 0)
    {
      motor2.pulsePosMinus();
    }
    else
    {
      motor2.pulsePosPlus();
    }
  }
#endif

// create an object named audioplayer
  // void MeAudioPlayer::PlayerInit(void)
  // void MeAudioPlayer::playMusicFileIndex(uint16_t music_index);
  // void MeAudioPlayer::playMusicFileName(String str);  
  // void MeAudioPlayer::pauseMusic(void);
  // void MeAudioPlayer::stopMusic(void);
  // void MeAudioPlayer::playNextMusic(void);
  // void MeAudioPlayer::playPrevMusic(void);
  // void MeAudioPlayer::setMusicVolume(uint8_t vol);
  // void MeAudioPlayer::setMusicPlayMode(uint8_t mode);
  // void MeAudioPlayer::startRecordingFileName(String str);
  // void MeAudioPlayer::stopRecording(void);
  // void MeAudioPlayer::volumeUp(void);
  // void MeAudioPlayer::volumeDown(void);
  // void MeAudioPlayer::deleteFileName(String str);
  // void MeAudioPlayer::deleteAllMusicFile(void);
#ifdef USE_audioplayer
  #include <Wire.h>
  #include <MeAudioPlayer.h>
  MeAudioPlayer audioplayer(PORT_7);
  int S_Walle = 1;
  int S_Wow = 2;
  int S_Eve = 3;
  int S_EngineFail = 4;
  int S_Blast1 = 5;
  int S_StartUp = 6;
  int S_Robot1 = 7;
  int S_Robot2 = 8;
  int S_Robot3 = 9;
  int S_Robot4 = 10;
  int S_Robot5 = 11;
  int S_Blast2 = 12;
  int S_Blast3 = 13;
  int S_Tie = 14;
  int S_Explosion = 15;
  int S_Flyby = 16;
  int S_count = 16;
#endif

// create an object named linefollower 
//MeLineFollower linefollower(PORT_7);
  // 2. uint8_t MeLineFollower::readSensors(void)
  // 3. bool MeLineFollower::readSensor1(void)
  // 4. bool MeLineFollower::readSensor1(void)

// create an object named ultrasensor
//MeUltrasonicSensor ultraSensor(PORT_9);
  // 2. double MeUltrasonicSensor::distanceCm(uint16_t MAXcm)

// create an object named ledmatrix
  // 1.    void MeLEDMatrix::clearScreen();
  // 2.    void MeLEDMatrix::setBrightness(uint8_t Bright);
  // 3.    void MeLEDMatrix::setColorIndex(bool Color_Number);
  // 4.    void MeLEDMatrix::drawBitmap(int8_t x, int8_t y, uint8_t Bitmap_Width, uint8_t *Bitmap);
  // 5.    void MeLEDMatrix::drawStr(int16_t X_position, int8_t Y_position, const char *str);
  // 6.    void MeLEDMatrix::showClock(uint8_t hour, uint8_t minute, bool point_flag);
  // 7.    void MeLEDMatrix::showNum(float value,uint8_t digits);
  // 8.    void MeLEDMatrix::reset(uint8_t port);
#ifdef USE_ledmatrix
  MeLEDMatrix ledmatrix(PORT_10);
#endif

// create an object named lightsensor
//MeLightSensor lightsensor;
  // 2. int16_t MeLightSensor::read()
  // 3. void    MeLightSensor::lightOn()
  // 4. void    MeLightSensor::lightOff()

// create an object named temperature
//MeTemperature temperature;
  // 4. float MeTemperature::temperature(void)

// create an object named gyro
//MeGyro gyro;
  // 2. void MeGyro::begin(void)
  // 3. void MeGyro::update(void)
  // 4. void MeGyro::fast_update(void)
  // 6. double MeGyro::getAngleX(void)
  // 7. double MeGyro::getAngleY(void)
  // 8. double MeGyro::getAngleZ(void)
  // 9. double MeGyro::getGyroX(void)
  // 10. double MeGyro::getGyroY(void)

// create an object named buzzer
//MeBuzzer buzzer;
  // 2. void MeBuzzer::tone(int pin, uint16_t frequency, uint32_t duration);
  // 3. void MeBuzzer::tone(uint16_t frequency, uint32_t duration)
  // 4. void MeBuzzer::noTone(int pin);
  // 5. void MeBuzzer::noTone();

// create an object named rgbled
// on-board LED ring, at PORT0 (onboard), with 12 LEDs
  // 1. void MeRGBLed::reset(uint8_t port)
  // 2. void MeRGBLed::reset(uint8_t port,uint8_t slot)
  // 3. void MeRGBLed::setpin(uint8_t port)
  // 4. uint8_t MeRGBLed::getNumber()
  // 5. cRGB MeRGBLed::getColorAt(uint8_t index)
  // 6. void MeRGBLed::fillPixelsBak(uint8_t red, uint8_t green, uint8_t blue)
  // 7. bool MeRGBLed::setColorAt(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
  // 8. bool MeRGBLed::setColor(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
  // 9. bool MeRGBLed::setColor(uint8_t red, uint8_t green, uint8_t blue)
  // 10. bool MeRGBLed::setColor(uint8_t index, long value)
  // 11. void MeRGBLed::show()
#ifdef USE_rgbledring
  #define RINGLEDNUM  12
  #define RINGALLLEDS 12
  MeRGBLed rgbledring( 0, RINGLEDNUM );
  int LedMap[13] = {RINGALLLEDS,5,6,7,8,9,10,11,0,1,2,3,4};
  cRGB amber;      //255,194,000
  cRGB orange;     //255,165,000
  cRGB vermillion; //227,066,052
  cRGB red;        //50,000,000
  cRGB magenta;    //255,000,255
  cRGB purple;     //128,000,128
  cRGB indigo;     //075,000,130
  cRGB blue;       //000,000,50
  cRGB aquamarine; //127,255,212
  cRGB green;      //000,50,000
  cRGB chartreuse; //127,255,000
  cRGB yellow;     //255,255,000
  cRGB black;      //000,000,000
  cRGB white;      //255,255,255
  cRGB LedCol[13] = {black,red,red,red,red,red,red,red,red,red,red,red,red};  
#endif

// create an object named SoundSensor
  // void MeSoundSensor::setpin(uint8_t SoundSensorPin)
  // uint8_t MeSoundSensor::strength()
#ifdef USE_SoundSensor
  MeSoundSensor SoundSensor(PORT_14);
#endif

#ifdef USE_DEBUG
  #include <SoftwareSerial.h>
#endif

//Program variables
int M1_Pos[] = {0,0}; // Array to store Current[0] and Previous[1] positions of Motor 1
int M2_Pos[] = {0,0}; // Array to store Current[0] and Previous[1] positions of Motor 1
int M3_Pos[] = {0,0}; // Array to store Current[0] and Previous[1] positions of Motor 1
int LockMain_active = 1;
int LockMain_value[] = {100,0,0,0,0,0,0,0,0,0,0,0,0}; // current value for 12 different lock positions
int LockMain_unlock[] = {100,0,0,0,0,0,0,0,0,0,0,0,0}; // unlock value for 12 different lock positions
int mode = 2; // 1 = intro, 2 = game, 3 = outro

//Program Methods
// ConvertPos - Converts wheel positions to an int value, can be used in different cases.
int ConvertPos(int difPos, int RetMin, int RetMax, int Old)
{
  int RetNew = Old;
  if(difPos > 0)
  {
    if(abs(difPos) > 10)
    {
      ++RetNew;
      if(RetNew > RetMax){RetNew = RetMin;}
    }
  }
  else if(difPos < 0)
  {
    if(abs(difPos) > 10)
    {
      --RetNew;
      if(RetNew < RetMin){RetNew = RetMax;}    
    }
  }
  return RetNew;
}
// ConvertPosBis - Converts wheel positions to an int value, can be used in different cases.
int ConvertPosBis(int Pos, int Degrees, int Old){
  int RetNew = Old;
  int difPos = Pos * -1;
  if(difPos >= 0)
  {
    while (difPos > 360)
    {
      difPos = difPos - 360;
    }
    RetNew = difPos / Degrees;
    RetNew = RetNew + 1;
  }
  else if(difPos < 0)
  {
    while (difPos < -360)
    {
      difPos = difPos + 360;
    }
    RetNew = difPos / Degrees;
    RetNew = 12 + RetNew;
  }
  return RetNew;
}
// GetMotor1Pos - Get the motor 1 position as int
int GetMotor1Pos(int lockMain_old)
{
  motor1.loop();
  M1_Pos[1] = M1_Pos[0];
  M1_Pos[0] = motor1.getCurPos();
  //return ConvertPos(M1_Pos[1] - M1_Pos[0], 1, 12,  lockMain_old);
  return ConvertPosBis(M1_Pos[0],30,1);
}
// GetMotor2Pos - Get the motor 2 position as int
int GetMotor2Pos(int lock1_old)
{
  motor2.loop();
  M2_Pos[1] = M2_Pos[0];
  M2_Pos[0] = motor2.getCurPos();
  //return ConvertPos(M2_Pos[1] - M2_Pos[0], 0, 9,  lock1_old);
  return ConvertPosBis(M2_Pos[0],36,1);
}
// GetMotor3Pos - Get the motor 3 position as int
int GetMotor3Pos(int lock2_old)
{
  //motor3.loop();
  M3_Pos[1] = M3_Pos[0];
  M3_Pos[0] = motor3.getCurrentPosition();
  //return ConvertPos(M3_Pos[1] - M3_Pos[0], 0, 9,  lock2_old);
  return ConvertPosBis(M3_Pos[0],36,1);
}
//
void IsLocked(int LockMain_active) {
      LockMain_value[LockMain_active] = GetMotor2Pos(LockMain_value[LockMain_active]);
      //ledmatrix.showNum(LockMain_value[LockMain_active]);
      if (LockMain_value[LockMain_active] == LockMain_unlock[LockMain_active])
      {
        LedCol[LockMain_active] = green;
      } else {
        LedCol[LockMain_active] = red;
      }
      #ifdef USE_DEBUG
        Serial.println(LockMain_value[LockMain_active]);
      #endif
}
// SetLedColor - Set all leds to a single color
void SetLedColor(cRGB color)
{
  rgbledring.setColor( color.r, color.g, color.b );
  rgbledring.show();
}
// SetLedColorAt - Set led at position to a single color
void SetLedColorAt(int index, cRGB color)
{
  rgbledring.setColorAt( index, color.r, color.g, color.b );
  rgbledring.show();
}
// SetAllLedColor - Set all leds to the color from the LedCol color map
void SetAllLedColor()
{
  for (int i = 1; i <= 12; i++) {
    if(i == LockMain_active){
      SetLedColorAt(LedMap[i], amber);
      if(LockMain_value[i] == LockMain_unlock[i]){
        SetLedColorAt(LedMap[i], LedCol[i]);
      }else{
        SetLedColorAt(LedMap[i], amber);
      }
    }else{
      SetLedColorAt(LedMap[i], LedCol[i]);
    }
  }
}
void setup()
{
  #ifdef USE_DEBUG
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Serial Debug Activated!!");
  #endif
  #ifdef USE_encodernew
    motor3.begin();
    motor3.reset();
  #endif
  #ifdef USE_encoderonboard
    // these interrupts are necesssary to
    // enable the motor to know where it is
    attachInterrupt(motor1.getIntNum(), isr_process_encoder1, RISING); 
    attachInterrupt(motor2.getIntNum(), isr_process_encoder2, RISING);
    //Set PWM 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);
    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);
    motor1.setPulse(9);
    motor2.setPulse(9);
    motor1.setRatio(39.267);
    motor2.setRatio(39.267);
    motor1.setPosPid(1.8,0,1.2);
    motor2.setPosPid(1.8,0,1.2);
    motor1.setSpeedPid(0.18,0,0);
    motor2.setSpeedPid(0.18,0,0);
  #endif
  #ifdef USE_rgbledring
    //Set color values
    amber.r = 50; amber.g = 25; amber.b = 000;
    black.r = 000; black.g = 000; black.b = 000;
    red.r = 50; red.g = 000; red.b = 000;
    green.r = 000; green.g = 50; green.b = 000;
    rgbledring.setpin( 44 );
    for (int i = 1; i <= 12; i++) {
      LedCol[i] = red;
    }
    //Turn off all Leds
    SetLedColor( black );
    //Set all leds to starting colors
    SetAllLedColor();   
  #endif
  #ifdef USE_ledmatrix
    ledmatrix.setBrightness(1);
    ledmatrix.setColorIndex(1);
  #endif
  randomSeed(analogRead(0));
  for (int i = 1; i <= 12; i++) {
    LockMain_unlock[i]= random(10);
  }
}

void loop()
{
  #ifdef USE_DEBUG
    Serial.println("---> Top of the loop <---");
  #endif
  switch (mode) {
    case 1:
      #ifdef USE_DEBUG
        Serial.println("---> Mode: Intro <---");
      #endif
      ledmatrix.drawStr(0,7,"test");
      break;
    case 2:
      #ifdef USE_DEBUG
        Serial.println("---> Mode: Game <---");
      #endif
        //Determine Current lock position
        LockMain_active = GetMotor1Pos(LockMain_active);
        ledmatrix.showNum(LockMain_active);
        //determine the color for the current lock
        IsLocked(LockMain_active);
        //set all leds to their color map 
        SetAllLedColor();
      break;
    case 3:
      #ifdef USE_DEBUG
        Serial.println("---> Mode: Outro <---");
      #endif
      break;
  }
  delay(200);
}