// Fred Nicolls, Sept 2025

#include <MobaTools.h>

#include "BluetoothSerial.h"

// #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
// #error Bluetooth is not Enabled! Please run `make menuconfig` to and Enable it
// #endif

BluetoothSerial btSerial;

MoToStepper stepperW(2048, STEPDIR);  
MoToStepper stepperF(2048, STEPDIR);  

int rwt = 500;
int wsps = 1024;
int wspwt = 2048;
int fspwt = 100;
int fs_l = 0;
int fs_u = 300;
int jwsps = 300;
int jfsps = 300;

int ismoving_flag = 0;
int iswinding_flag = 0;
int feed_direction = 1;

// #include <Preferences.h>
// Preferences preferences;

//int sposW, sposF;

// Motor Winder
const int ENW=23;
const int DIRW=5;
const int STEPW=18;
const int M0W=19;
const int M1W=21;
const int M2W=17;  // BRIAN'S IS STILL WIRED ON 3 (COLLIDES WITH SERIAL0)

// Motor Feed
const int ENF=22;
const int DIRF=13;
const int STEPF=12;
const int M0F=14;
const int M1F=27;
const int M2F=26;

// Output pins
const int RPINS[] = { ENW, DIRW, STEPW, M0W, M1W, M2W, ENF, DIRF, STEPF, M0F, M1F, M2F }; 
const int rpins_length = sizeof(RPINS)/sizeof(RPINS[0]);
inline void rpins_reset(void) { for( int i=0; i<rpins_length; i++ ) digitalWrite(RPINS[i], 0); }

void setup_pins(void) 
{ 
  for( int i=0; i<rpins_length; i++ ) pinMode(RPINS[i], OUTPUT);  
  rpins_reset(); 
  //digitalWrite(ENW, 1);  digitalWrite(ENF, 1);
}
void loop_pins(void) {};


void setmicrostepping(int mstepexp, int M0, int M1, int M2)
{
  int Mv[3];
  Mv[0] = M0;  Mv[1] = M1;  Mv[2] = M2;

  for( int i=0; i<3; i++ ) {
    if( (mstepexp>>i) & 1) digitalWrite(Mv[i],LOW);
    else digitalWrite(Mv[i],HIGH);
  }
}


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

float getval_btserial(void)
{
  char inchar;
  char mess[32];

  int bytesread = btSerial.readBytesUntil('*', mess, 30);
  mess[bytesread + 1] = '\0';
  double num_double = std::stod(mess);
  //Serial.println("num_double = " + String(num_double));

  return(num_double);
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
    ismoving_flag = 0;
    iswinding_flag = 0;

    stepperW.rotate(0);
    stepperF.rotate(0);
  
    report_status();
  }

  // Advance microstepping
  if( cmd=='a' || cmd=='A' ) {
    int Mv[3];
    if( cmd=='a' ) { Mv[0] = M0W;  Mv[1] = M1W;  Mv[2] = M2W; }
    else { Mv[0] = M0F;  Mv[1] = M1F;  Mv[2] = M2F; }

    int mstepexp = 4*digitalRead(Mv[2]) + 2*digitalRead(Mv[1]) + digitalRead(Mv[0]);
    mstepexp += 1;
    if( mstepexp>5 ) mstepexp = 0;
    for( int i=0; i<3; i++ ) {
      if( (mstepexp>>i) & 0x1) digitalWrite(Mv[i],HIGH);
      else digitalWrite(Mv[i],LOW);
    }
  }

  // Relays (manual)
  if( cmd>='f' && cmd<='q' ) {
    int rno = int(cmd) - int('f');  
    digitalWrite(RPINS[rno], !digitalRead(RPINS[rno]));
    btLog("Toggling pin " + String(cmd));
  }

  // Jog W
  if( cmd=='r' || cmd=='s' || cmd=='R' || cmd=='S' ) {
    stepperW.setSpeedSteps(jwsps*10);
    stepperW.setRampLen(100);
  }
  if( cmd=='r') stepperW.rotate(-1);
  if( cmd=='s' ) stepperW.rotate(1);
  if( cmd=='R' || cmd=='S' ) stepperW.rotate(0);

  // Jog F
  if( cmd=='t' || cmd=='u' || cmd=='T' || cmd=='U' ) {
    stepperF.setSpeedSteps(jfsps*10);
    stepperF.setRampLen(1);
  }
  if( cmd=='t') stepperF.rotate(-1);
  if( cmd=='u' ) stepperF.rotate(1);
  if( cmd=='T' || cmd=='U' ) stepperF.rotate(0);

  if( cmd=='B' ) rwt = (int)round(getval_btserial());
  if( cmd=='c' ) wsps = (int)round(getval_btserial());
  if( cmd=='C' ) wspwt = (int)round(getval_btserial());
  if( cmd=='d' ) fspwt = (int)round(getval_btserial());
  if( cmd=='e' ) {
    fs_l = (int)round(getval_btserial());
    if( fs_l>fs_u ) fs_u = fs_l + 1;
  }
  if( cmd=='E' ) {
    fs_u = (int)round(getval_btserial());
    if( fs_u<fs_l ) fs_l = fs_u - 1;
  }
  if( cmd=='K' ) jwsps = (int)round(getval_btserial());
  if( cmd=='J' ) jfsps = (int)round(getval_btserial());

  if( cmd=='F' ) {
    fs_l = stepperF.currentPosition();
    if( fs_l>fs_u ) fs_u = fs_l + 1;
  }
  if( cmd=='G' ) {
    fs_u = stepperF.currentPosition();
    if( fs_u<fs_l ) fs_l = fs_u - 1;
  }
  if( cmd=='P' ) {
    stepperF.setSpeedSteps(jfsps*10);
    stepperF.writeSteps(fs_l);
    ismoving_flag = 1;
  }
  if( cmd=='Q') {
    stepperF.setSpeedSteps(jfsps*10);
    stepperF.writeSteps(fs_u);
    ismoving_flag = 1;
  }

  if( cmd=='z') {
    btLog("Setting current positions as home");
    stepperW.setZero();
    stepperF.setZero();
  }

  // Start winding
  if( cmd=='W' ) {

    // Abort if current feed position not within limits
    if( stepperF.currentPosition()<fs_l || stepperF.currentPosition()>fs_u) {
      btLog("Feed stepper must be inside limits");
      String mstr = "*S*";
      btSerial.println(mstr);
      return;
    }

    // Set motor speeds for winding
    stepperW.setSpeedSteps(wsps*10);
    stepperW.setRampLen(100);
    float wtps = round((float)wsps/wspwt);
    float fsps = round((float)fspwt*wtps);
    //Serial.println("wtps=" + String(wtps));
    //Serial.println("fsps=" + String(fsps));
    stepperF.setSpeedSteps(fsps*10);
    stepperF.setRampLen(1);

    // Start winding
    stepperW.setZero();
    stepperW.writeSteps(rwt*wspwt);
    //btLog("Calling stepperW.writeSteps(" + String(rwt*wspwt) + ")");
    stepperF.rotate(feed_direction);
    //btLog("Calling stepperF.rotate((" + String(feed_direction) + ")");
    iswinding_flag = 1;
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
unsigned long stepperpos_period = 250;
void report_stepperpos()
{
  String mstr;
  //btLog("Reporting stepperpos");
 
  mstr = "*v";
  if( iswinding_flag ) mstr += String(stepperW.currentPosition());
  else mstr += String(0);
  mstr += "*";
  btSerial.println(mstr);

  mstr = "*V";
  mstr += String(stepperF.currentPosition());
  mstr += "*";
  btSerial.println(mstr);

  mstr = "*b";
  if( iswinding_flag ) mstr += String((double)stepperW.currentPosition()/wspwt,2);
  else mstr += String((double)0.0, 2);
  mstr += "*";
  btSerial.println(mstr);

  stepperpos_lastreport = millis();
}

// Report stepper settings
unsigned long stepperset_lastreport = 0;
unsigned long stepperset_period = 500;
void report_stepperset()
{
  String mstr;

  int mstepexpw = 4*digitalRead(M2W) + 2*digitalRead(M1W) + digitalRead(M0W);
  mstr = "*a";  mstr += String((int)round(pow(2,mstepexpw)));  mstr += "*";  btSerial.println(mstr);
  int mstepexpf = 4*digitalRead(M2F) + 2*digitalRead(M1F) + digitalRead(M0F);
  mstr = "*A";  mstr += String((int)round(pow(2,mstepexpf)));  mstr += "*";  btSerial.println(mstr);
  mstr = "*B";  mstr += String(rwt);  mstr += "*";  btSerial.println(mstr);
  mstr = "*c";  mstr += String(wsps);  mstr += "*";  btSerial.println(mstr);
  mstr = "*C";  mstr += String(wspwt);  mstr += "*";  btSerial.println(mstr);
  mstr = "*d";  mstr += String(fspwt);  mstr += "*";  btSerial.println(mstr);
  mstr = "*e";  mstr += String(fs_l);  mstr += "*";  btSerial.println(mstr);
  mstr = "*E";  mstr += String(fs_u);  mstr += "*";  btSerial.println(mstr);
  mstr = "*K";  mstr += String(jwsps);  mstr += "*";  btSerial.println(mstr);
  mstr = "*J";  mstr += String(jfsps);  mstr += "*";  btSerial.println(mstr);

  stepperset_lastreport = millis();
}


// Report all
void report_status(void)
{
  if( millis()-rpins_lastreport>=rpins_period ) report_rpins();
  if( millis()-stepperpos_lastreport>=stepperpos_period ) report_stepperpos();
  if( millis()-stepperset_lastreport>=stepperset_period ) report_stepperset();
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

  iswinding_flag = 0;

  st = stepperW.attach(STEPW, DIRW);
  stepperW.attachEnable(ENW, 0, LOW);
  stepperW.autoEnable(1);
  stepperW.setSpeed(150);
  stepperW.setRampLen(5);
  //stepperW.setZero();  stepperW.rotate(0);

  st = stepperF.attach(STEPF, DIRF);
  stepperF.attachEnable(ENF, 0, LOW);
  stepperF.autoEnable(1);
  stepperF.setSpeed(150);
  stepperF.setRampLen(5);
  //stepperF.setZero();  stepperF.rotate(0);
}

void loop() {
  report_connblink();

  loop_btserialcmd();
  loop_pins();

  // Handle winding feed and finish
  if( iswinding_flag ) {

    if( feed_direction==1 && stepperF.currentPosition()>fs_u ) {
      feed_direction = -1;
      stepperF.rotate(feed_direction);
    }

    if( feed_direction==-1 && stepperF.currentPosition()<fs_l ) {
      feed_direction = 1;
      stepperF.rotate(feed_direction);
    }

    if( (double)stepperW.currentPosition()/wspwt>=rwt ) {
      //stepperW.rotate(0);  
      stepperF.rotate(0);
      iswinding_flag = 0;
    }

  }

  report_status();  // timed report

  delay(20);
}
