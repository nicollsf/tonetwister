// Fred Nicolls, Sept 2025

#include <MobaTools.h>

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial btSerial;

MoToStepper stepperW(2048, STEPDIR);  
MoToStepper stepperF(2048, STEPDIR);  
int sposW, sposF;

// Motor A
const int ENA=23;
const int DIRA=5;
const int STEPA=18;
const int M0A=19;
const int M1A=21;
const int M2A=17;  // BRIAN'S IS STILL WIRED ON 3 (COLLIDES WITH SERIAL0)

// Motor B
const int ENB=22;
const int DIRB=13;
const int STEPB=12;
const int M0B=14;
const int M1B=27;
const int M2B=26;

// Output pins
const int RPINS[] = { ENA, DIRA, STEPA, M0A, M1A, M2A, ENB, DIRB, STEPB, M0B, M1B, M2B }; 
const int rpins_length = sizeof(RPINS)/sizeof(RPINS[0]);
inline void rpins_reset(void) { for( int i=0; i<rpins_length; i++ ) digitalWrite(RPINS[i], 0); }

void setup_pins(void) 
{ 
  for( int i=0; i<rpins_length; i++ ) pinMode(RPINS[i], OUTPUT);  
  rpins_reset(); 
}
void loop_pins(void) {};


// ----------------------------------------------------------------------
//   Serial command input
// ----------------------------------------------------------------------

// Character-based code
void loop_btserialcmd(void)
{
  char inchar;
  char mess1[32], endchar1 = 'K';
  char mess2[32], endchar2 = 10;

  while( btSerial.available()>0 ) {
    //Serial.println("In loop_btserialcmd: btSerial.available=" + String(btSerial.available()));
    inchar = btSerial.read();
    //Serial.println("In loop_btserialcmd: btSerial.read() returned " + String(inchar));
    //Serial.print(String(inchar));

    if( inchar == -1 ) {
      btLog("btSerial.read() returned zero bytes");
      break;
    }

    // Greedy chomp AT message
    if( inchar == '+' ) {
      Serial.println("Chomping AT message: +");
      int bytesread = btSerial.readBytesUntil(endchar1, mess1, 30);
      mess1[bytesread] = endchar1;  mess1[bytesread + 1] = '\0';
      Serial.print(mess1);
      bytesread = btSerial.readBytesUntil(endchar2, mess2, 30);
      mess2[bytesread] = endchar2;  mess2[bytesread + 1] = '\0';
      Serial.print(mess2);  Serial.println("[Chomped]");
      break;
    }

    // Else execute command
    //Serial.println("In loop_btserialcmd: calling serialcmd(" + String(inchar) + ")");
    serialcmd(inchar);
  }
}

// Commands
void serialcmd(char cmd)
{
  char inchar;
  char mess[32];
  Serial.print("Entered serialcmd with cmd=");  Serial.println(cmd);
  const int dstep = 250;

  // Kill
  if( cmd == 'N' ) {
    btLog("KILL");
    stepperW.stop();
    stepperF.stop();
    setup_pins();
  
    report_status();
  }

  // Relays (manual)
  if( cmd>='f' && cmd<='q' ) {
    int rno = int(cmd) - int('f');  
    digitalWrite(RPINS[rno], !digitalRead(RPINS[rno]));
    btLog("Toggling pin " + String(cmd));
  }

  if( cmd=='r') {
    //btLog("Stepping motor W from " + String(sposA) + " to " + String(sposA+dstep));   
    stepperW.rotate(-1);
  }
  if( cmd=='s' ) {
    //btLog("Stepping motor W from " + String(sposA) + " to " + String(sposA-dstep));
    stepperW.rotate(1);
  }
  if( cmd=='R' || cmd=='S' ) {
    stepperW.rotate(0);
  }

  if( cmd=='t') {
    //btLog("Stepping motor B from " + String(sposB) + " to " + String(sposB+dstep));   
    stepperF.rotate(-1);
  }
  if( cmd=='u' ) {
    //btLog("Stepping motor B from " + String(sposB) + " to " + String(sposB-dstep));
    stepperF.rotate(1);
  }
  if( cmd=='T' || cmd=='U' ) {
    stepperF.rotate(0);
  }


  if( cmd=='z') {
    btLog("Setting current positions as home");
    stepperW.setZero();
    stepperF.setZero();
  }

  report_status();
}


// ----------------------------------------------------------------------
//   GUI reporting
// ----------------------------------------------------------------------

// Flashing light to indicate bluetooth connection
unsigned long connind_lastupdate = 0;
unsigned long connind_period = 750;
int connind_state = 0;
void report_connblink()
{
  String rcmd = "*jR0G0B0*";
  String gcmd = "*jR255G255B0*";

  if( millis()-connind_lastupdate>=connind_period ) {
    connind_lastupdate = millis();
    connind_state = !connind_state;
    if( connind_state ) btSerial.println(rcmd);
    else btSerial.println(gcmd);
  }
}

void btLog(String mess)
{
  static String lastmess;
  if( mess==lastmess ) return;

  Serial.println("btLog: " + mess);
  btSerial.print("*L" + mess + "\n" + "*");
  lastmess = mess;
}

// Report relay pin outputs
unsigned long rpins_lastreport = 0;
unsigned long rpins_period = 500;
void report_rpins()
{
  String mstr;

  String cstr = "";
  for( int i=0; i<rpins_length; i++ ) {
    if( digitalRead(RPINS[i])==0 ) cstr += "0";
    else cstr += "1";
  }

  mstr = "*i";
  mstr += cstr.substring(0,rpins_length/2);
  mstr += "*";
  btSerial.println(mstr);

  mstr = "*I";
  mstr += cstr.substring(rpins_length/2,rpins_length);
  mstr += "*";
  btSerial.println(mstr);
  
  rpins_lastreport = millis();
}

// Report stepper positions
unsigned long stepperpos_lastreport = 0;
unsigned long stepperpos_period = 500;
void report_stepperpos()
{
  String mstr;
  btLog("Reporting stepperpos");
 
  mstr = "*v";
  mstr += String(stepperW.currentPosition());
  mstr += "*";
  btSerial.println(mstr);

  mstr = "*V";
  mstr += String(stepperF.currentPosition());
  mstr += "*";
  btSerial.println(mstr);

  stepperpos_lastreport = millis();
}

// Report all
void report_status(void)
{
  if( millis()-rpins_lastreport>=rpins_period ) report_rpins();
  if( millis()-stepperpos_lastreport>=stepperpos_period ) report_stepperpos();
}


// ----------------------------------------------------------------------
//   Main setup and loop
// ----------------------------------------------------------------------

void setup() {
  int st;

  Serial.begin(115200);
  btSerial.begin("tonetwister");  // bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  setup_pins();

  st = stepperW.attach(STEPA, DIRA);
  stepperW.attachEnable(ENA, 0, LOW);
  stepperW.autoEnable(1);
  stepperW.setSpeed(150);  // 10 rpm
  stepperW.setRampLen(5);//2048/2);
  //stepperW.rotate(0);

  st = stepperF.attach(STEPB, DIRB);
  stepperF.attachEnable(ENB, 0, LOW);
  stepperF.autoEnable(1);
  stepperF.setSpeed(150);  // 10 rpm
  stepperW.setRampLen(5);
  //stepperF.rotate(1);

  //stepperW.connectToPins(STEPA, DIRA);
  //stepperW.setEnablePin(ENA);
  //stepperF.connectToPins(STEPB, DIRB);
  //stepperF.setEnablePin(ENB);

  //stepperW.setSpeedInRevolutionsPerSecond(1);
  //stepperF.setSpeedInRevolutionsPerSecond(1);

  //stepperW.setAccelerationInRevolutionsPerSecondPerSecond(0.5);
  //stepperF.setAccelerationInRevolutionsPerSecondPerSecond(0.5);

  //stepperW.setCurrentPositionAsHomeAndStop();
  //stepperF.setCurrentPositionAsHomeAndStop();

  //stepperW.startAsService(0);
  //stepperF.startAsService(0);
}

void loop() {
  report_connblink();

  //sposA = stepperW.getCurrentPositionInSteps();
  //sposB = stepperF.getCurrentPositionInSteps();

  // if (Serial.available()) {
  //   btSerial.write(Serial.read());
  // }
  // if (btSerial.available()) {
  //   Serial.write(btSerial.read());
  // }

  // Process command input from bluetooth serial
  // Uncomment below for board with BT serial attached

  loop_btserialcmd();

  loop_pins();
  //myStepper.step(stepsPerRotation);  

  // Timed report
  report_status();

  delay(20);
}
