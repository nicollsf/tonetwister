// Fred Nicolls, Sept 2025

#include "BluetoothSerial.h"
BluetoothSerial btSerial;

#include <Preferences.h>
Preferences prefs;

#include <MobaTools.h>

MoToStepper stepperW(2048, STEPDIR);  
MoToStepper stepperF(2048, STEPDIR);  

// Winding parameters
int rwt = 500;
int wsps = 300;
int wspwt = 200;
int wsacc = 100;
float fspwt = 100.0;
int fs_l = 0;
int fs_u = 300;
int jwsps = 300;
int jfsps = 300;
int mstepexpw = 0;
int mstepexpf = 0;

// State flags
int ismoving_flag = 0;
int iswinding_flag = 0;
int feed_direction = 1;

// Motor Winder
const int ENW = 23;
const int DIRW = 5;
const int STEPW = 18;
const int M0W = 19;
const int M1W = 21;
const int M2W = 17;

// Motor Feed
const int ENF = 22;
const int DIRF = 13;
const int STEPF = 12;
const int M0F = 14;
const int M1F = 27;
const int M2F = 26;

// Feed waveform parameters
float pt_per, pt_slope, pt_dc, pt_wmin;

void setmicrostepping(int mstepexp, int M0, int M1, int M2)
{
  int Mv[3];
  Mv[0] = M0;  Mv[1] = M1;  Mv[2] = M2;

  for( int i=0; i<3; i++ ) {
    if( (mstepexp>>i) & 1) digitalWrite(Mv[i], HIGH);
    else digitalWrite(Mv[i], LOW);
  }
}

// Output pins
const int RPINS[] = { ENW, DIRW, STEPW, M0W, M1W, M2W, ENF, DIRF, STEPF, M0F, M1F, M2F }; 
const int rpins_length = sizeof(RPINS)/sizeof(RPINS[0]);
inline void rpins_reset(void) { for( int i=0; i<rpins_length; i++ ) digitalWrite(RPINS[i], 0); }

void setup_pins(void) 
{ 
  for( int i=0; i<rpins_length; i++ ) pinMode(RPINS[i], OUTPUT);  
  rpins_reset(); 
  setmicrostepping(mstepexpw, M0W, M1W, M2W);
  setmicrostepping(mstepexpf, M0F, M1F, M2F);
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


// Given a point w in the domain this function calculates the value for a periodic triangular 
// waveform function.  The waveform is determined by the following parameters:
// pt_per - period of triangular waveform function
// pt_slope - magnitude of slope of linear segments
// pt_dc - average value (dc offset) of periodic triangular waveform
// pt_wmin - a point w in the domain where the function attains a minimum value
float getval_ptriang(float w, float pt_per, float pt_slope, float pt_dc, float pt_wmin)
{
  // Get interpolation point for function over the range -pt_per/2 to pt_per/2
  float wr = std::remainder(w - pt_wmin, pt_per);
  if( wr>pt_per/2 ) wr -= pt_per;

  float minv = pt_dc - pt_slope*pt_per/4;  // minimum value of triangular waveform
  float val = minv + pt_slope*fabs(wr);

  //Serial.println(String(val) + " = getval_ptriang(" +String(w) + "," + String(pt_per) + "," + String(pt_slope) + "," + String(pt_dc) + "," + String(pt_wmin) + ")");
  return(val);
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
    //setup_pins();
    ismoving_flag = 0;
    iswinding_flag = 0;

    //stepperW.rotate(0);
    //stepperF.rotate(0);
  
    report_status();
  }

  // Advance microstepping
  if( cmd=='a' ) {
    mstepexpw += 1;
    if( mstepexpw>5 ) mstepexpw = 0;
    setmicrostepping(mstepexpw, M0W, M1W, M2W);
    prefs.putInt("mstepexpw", mstepexpw);
  }

  if( cmd=='A' ) {
    mstepexpf += 1;
    if( mstepexpf>5 ) mstepexpf = 0;
    setmicrostepping(mstepexpf, M0F, M1F, M2F);
    prefs.putInt("mstepexpw", mstepexpw);
  }

  // Relays (manual)
  if( cmd>='f' && cmd<='q' ) {  // cmd=='f', cmd=='g', cmd=='h', cmd=='i', cmd=='j', cmd=='k', cmd=='l', cmd=='m', cmd=='n', cmd=='o', cmd=='p', cmd=='q' 
    int rno = int(cmd) - int('f');  
    digitalWrite(RPINS[rno], !digitalRead(RPINS[rno]));
    btLog("Toggling pin " + String(cmd));
  }

  // Jog W
  if( cmd=='r' || cmd=='s' || cmd=='R' || cmd=='S' ) {
    stepperW.setSpeedSteps(jwsps*10);
    stepperW.setRampLen(wsacc);
  }
  if( cmd=='r') stepperW.rotate(-1);
  if( cmd=='s' ) stepperW.rotate(1);
  if( cmd=='R' || cmd=='S' ) stepperW.rotate(0);

  // Jog F
  if( cmd=='t' || cmd=='u' || cmd=='T' || cmd=='U' ) {
    stepperF.setSpeedSteps(jfsps*10);
    stepperF.setRampLen(0);
  }
  if( cmd=='t') stepperF.rotate(-1);
  if( cmd=='u' ) stepperF.rotate(1);
  if( cmd=='T' || cmd=='U' ) stepperF.rotate(0);

  if( cmd=='B' ) { rwt = (int)round(getval_btserial());  prefs.putInt("rwt", rwt); }
  if( cmd=='c' ) { wsps = (int)round(getval_btserial());  prefs.putInt("wsps", wsps); }
  if( cmd=='C' ) { wspwt = (int)round(getval_btserial());  prefs.putInt("wspwt", wspwt); }
  if( cmd=='d' ) { fspwt = (float)getval_btserial();  prefs.putFloat("fspwt", fspwt); }
  if( cmd=='k' ) { wsacc = (int)round(getval_btserial());  prefs.putInt("wsacc", wsacc); }
  if( cmd=='e' ) {
    fs_l = (int)round(getval_btserial());
    if( fs_l>fs_u ) fs_u = fs_l + 1;
    prefs.putInt("fs_l", fs_l);  prefs.putInt("fs_u", fs_u);
  }
  if( cmd=='E' ) {
    fs_u = (int)round(getval_btserial());
    if( fs_u<fs_l ) fs_l = fs_u - 1;
    prefs.putInt("fs_l", fs_l);  prefs.putInt("fs_u", fs_u);
  }
  if( cmd=='K' ) { jwsps = (int)round(getval_btserial()); prefs.putInt("jwsps", jwsps); }
  if( cmd=='J' ) { jfsps = (int)round(getval_btserial()); prefs.putInt("jfsps", jfsps); }

  if( cmd=='F' ) {
    fs_l = stepperF.currentPosition();
    if( fs_l>fs_u ) fs_u = fs_l + 1;
    prefs.putInt("fs_l", fs_l);  prefs.putInt("fs_u", fs_u);
  }
  if( cmd=='G' ) {
    fs_u = stepperF.currentPosition();
    if( fs_u<fs_l ) fs_l = fs_u - 1;
    prefs.putInt("fs_l", fs_l);  prefs.putInt("fs_u", fs_u);
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

  // Modify fspwt to make integral number of feed cycles (start and end positions same)
  if( cmd=='D' ) {
    float fspfc = 2*(fs_u - fs_l + 1);  // feed steps per feed cycle
    float rfc_f = rwt*(float)fspwt/fspfc;  // total feed cycles (float)
    int rfc = (int)round(rfc_f);  // total feed cycles (integer)
    float fspwt_f = (float)rfc/rwt*fspfc;
    fspwt = fspwt_f;
    prefs.putFloat("fspwt", fspwt);
  }

  if( cmd=='z') {
    btLog("Setting current positions as home");
    stepperW.setZero();
    stepperF.setZero();
  }

  // Start winding
  if( cmd=='W' ) {

    // Abort if current feed position not within limits
    if( stepperF.currentPosition()<fs_l || stepperF.currentPosition()>fs_u ) {
      btLog("Feed stepper must be inside limits");
      String mstr = "*S*";
      btSerial.println(mstr);
      return;
    }

    // Parameters for feed step calculation
    float fspfc = 2*(fs_u - fs_l + 1);  // feed steps per feed cycle
    float wspfc = wspwt/fspwt*fspfc;  // winding steps per feed cycle
    float fspws = fspwt/wspwt;  // feed steps per winding step

    // Parameters for feed waveform
    pt_per = wspfc;
    pt_slope = fspws;
    pt_dc = ((float)fs_u + fs_l)/2.0;
    pt_wmin = -0.5;  // FIX!!!!  Assumes starting from fs_l

    // Set motor speeds for winding
    stepperW.setSpeedSteps(wsps*10);
    stepperW.setRampLen(wsacc);
    float wtps = round((float)wsps/wspwt);
    float fsps = round((float)fspwt*wtps);
    stepperF.setSpeedSteps(fsps*10);
    stepperF.setRampLen(0);

    // Start winding
    stepperW.setZero();
    stepperW.writeSteps(rwt*wspwt);
    stepperF.rotate(feed_direction);

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

  mstr = "*a";  mstr += String((int)round(pow(2,mstepexpw)));  mstr += "*";  btSerial.println(mstr);
  mstr = "*A";  mstr += String((int)round(pow(2,mstepexpf)));  mstr += "*";  btSerial.println(mstr);
  mstr = "*B";  mstr += String(rwt);  mstr += "*";  btSerial.println(mstr);
  mstr = "*c";  mstr += String(wsps);  mstr += "*";  btSerial.println(mstr);
  mstr = "*C";  mstr += String(wspwt);  mstr += "*";  btSerial.println(mstr);
  mstr = "*d";  mstr += String(fspwt);  mstr += "*";  btSerial.println(mstr);
  mstr = "*e";  mstr += String(fs_l);  mstr += "*";  btSerial.println(mstr);
  mstr = "*E";  mstr += String(fs_u);  mstr += "*";  btSerial.println(mstr);
  mstr = "*K";  mstr += String(jwsps);  mstr += "*";  btSerial.println(mstr);
  mstr = "*J";  mstr += String(jfsps);  mstr += "*";  btSerial.println(mstr);
  mstr = "*k";  mstr += String(wsacc);  mstr += "*";  btSerial.println(mstr);

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

  // Setup and load preferences
  prefs.begin("tonetwister-app");
  rwt = prefs.getInt("rwt", rwt);
  wsps = prefs.getInt("wsps", wsps);
  wspwt = prefs.getInt("wspwt", wspwt);
  wsacc = prefs.getInt("wsacc", wsacc);
  fspwt = prefs.getFloat("fspwt", fspwt);
  fs_l = prefs.getInt("fs_l", fs_l);
  fs_u = prefs.getInt("fs_u", fs_u);
  jwsps = prefs.getInt("jwsps", jwsps);
  jfsps = prefs.getInt("jfsps", jfsps);
  mstepexpw = prefs.getInt("mstepexpw", mstepexpw);
  mstepexpf = prefs.getInt("mstepexpf", mstepexpf);

  setup_pins();
  iswinding_flag = 0;

  st = stepperW.attach(STEPW, DIRW);
  stepperW.attachEnable(ENW, 0, LOW);
  stepperW.autoEnable(1);
  stepperW.setSpeedSteps(jwsps*10);
  stepperW.setRampLen(wsacc);
  //stepperW.setZero();  stepperW.rotate(0);

  st = stepperF.attach(STEPF, DIRF);
  stepperF.attachEnable(ENF, 0, LOW);
  stepperF.autoEnable(1);
  stepperF.setSpeedSteps(jfsps*10);
  stepperF.setRampLen(0);
  //stepperF.setZero();  stepperF.rotate(0);

}

void loop() {
  report_connblink();

  loop_btserialcmd();
  loop_pins();

  // Handle feed position for winding count
  if( iswinding_flag ) {

    if( !stepperW.moving() ) {
      stepperW.stop();
      stepperF.stop();
      iswinding_flag = 0;
    }

    if( 1 ) {
      if( feed_direction==1 && stepperF.currentPosition()>fs_u ) {
        feed_direction = -1;
        stepperF.rotate(feed_direction);
      }

      if( feed_direction==-1 && stepperF.currentPosition()<fs_l ) {
        feed_direction = 1;
        stepperF.rotate(feed_direction);
      }
    } else {
      // Set feed to required position
      int ws = stepperW.currentPosition();
      int riv = (int)round(getval_ptriang(ws, pt_per, pt_slope, pt_dc, pt_wmin));
      Serial.println("current,target,stepsrem=" + String(stepperF.currentPosition()) + "," + String(riv) + "," + String(stepperF.stepsToDo()));
      if( stepperF.currentPosition()!=riv ) {
        //Serial.println("Feed stepper to current,target=" + String(stepperF.currentPosition()) + "," + String(riv));
        Serial.println("Calling stepperF.moveTo(" + String(riv) + ")");
        stepperF.moveTo(riv);
      }
    }

  }

  report_status();  // timed report

  delay(100);
}
