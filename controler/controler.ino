  // *********************************************
// Authors: João Lopes; Gabriel Antero; Maria Brites 
//                - Group 13 -
// IST - SCDTR
// *********************************************
#include <math.h>
#include <EEPROM.h>
#include <mcp2515.h>
#include "nodeClass.h"
#include "PiC.h"
//***********************************************
//*****EEPROM VARIABLES *************************
int id = 0;
int address = 0;
/***********************************************
 * Input/Output pins
 */
/***********************************************/
const int analogInPin = A0;
const int analogOutPin = 9;
//***********************************************
float m;
float b;
float illuminance = 0;
float exIlluminance = 0;
float lowerBoundU = 50;
float lowerBoundO = 100;
int occupancy = 0;
float finalIluminance = lowerBoundU; //Referencia
int input = 0;

//  Timer Variables
unsigned long previousTime = 0; // milliseconds
unsigned long T = 10; // milliseconds
unsigned long resetTime = 0; // milliseconds
//*********************************************
// Interrupt flag
int flag = 0;
//*********************************************
// Controller constants and variables
float Kp=1;
float Ki=5;
PiC PIcontroller(Kp,Ki,T);
float u = 0;
float gain;
float ff;
float voltage;
//********************************************
const byte maskpresc= B11111000; // mask bits that are not prescale
int prescale = 1; //fastest possible
//**************************************************
//*****************CAN VARIABLES *******************
unsigned long c; 
unsigned long ident; 
//**************************************************
//**************CONSENSUS VARIABLES ****************
int nodes[10];
float k[10];
int idx = 0;

int check[3]={0,0,0};// if check[i-1]=1, then the results computed by node i in the current consensus iteration have been received
int sr[3]={0,0,0};// if sr[i-1]=2, then node i has received the results of the current consensus iteration sent by of this node and sent an acknowledge
float rho;
uint32_t mask1=0x000000FF;
uint32_t mask2=0x0000FF00;
uint32_t mask3=0x00FF0000;
uint32_t maskiter=0xFF000000;
uint32_t maskid=0x00000003;
uint32_t maskr=0x0000000C;
uint32_t maskidc=0x000007F0;
uint32_t message=0x00000000;
uint32_t data;
int niter=0;
uint32_t riter;
uint32_t sident=0; //write id in the first 4 bits of the id
uint32_t dreceive=0;
uint32_t rident=0;
uint32_t rid=0;
uint32_t sid=0;
  float d[3] = {0.0,0.0,0.0};
  float y[3] ={0.0,0.0,0.0};
  float d_av[3] = {0.0,0.0,0.0};
  float cst[3] = {0.0,0.0,0.0};
  NodeClass node(id,d,d_av,y,k,cst,exIlluminance,finalIluminance);
enum States {CONSENSUS, SEND, WAIT, END};
States state;
unsigned long wait_chrono;
unsigned long consensus_chrono;
bool consensusFlag=false;
bool hubFlag=false;

//**************Performance metrics****************
float E = 0;
unsigned long Nsamples = 0;
unsigned long Nsamplesflick = 0;
int flickErrorCounter = 0;
float V = 0;
float pl = 0;
float ppl = 0;
float F = 0;


//-------------------------------------------------------------------------------
//------------------- READ/WRITE CAN-BUS ----------------------------------------
//-------------------------------------------------------------------------------

MCP2515 mcp2515(10); //SS pin 10

uint32_t mask = 0x0000000C;
uint32_t filt0 = 0x00000000; //accepts msg ID 1 on RXB0


union my_can_msg { uint32_t value; uint8_t bytes[4]; };

MCP2515::ERROR write(unsigned long id, uint32_t val) {
  struct can_frame frame;
  frame.can_id = id;
  frame.can_dlc = 4;
  my_can_msg msg;
  msg.value = val;
  for(int i = 0; i < 4; i++)
    frame.data[i] = msg.bytes[i];
  return mcp2515.sendMessage(&frame);
}

MCP2515::ERROR read(unsigned long &c,unsigned long &ident) {
  struct can_frame frame;
  my_can_msg msg;
  MCP2515::ERROR err = mcp2515.readMessage(&frame);
  if(err == MCP2515::ERROR_OK) {
    for(int i = 0; i < 4; i++)
      msg.bytes[i] = frame.data[i];
    c = msg.value;
    ident = frame.can_id;
  }
  return err;
}

//-------------------------------------------------------------------------------
//----------------------------- READ LDR ----------------------------------------
//-------------------------------------------------------------------------------

float readLUX(int sensorValue){
    float R1 = 10000;
    float Vi = 5;
    float Vo = sensorValue * (5.0 / 1023.0);
    float LDR = (R1*(Vi-Vo))/Vo;
    return pow(10,(log10(LDR)-b)/m);
}

//-------------------------------------------------------------------------------
//---------------------- Compute Energy Consumption -----------------------------
//-------------------------------------------------------------------------------

void acEnergy(float PWM){
    float P=1;
    PWM=PWM/255;
    E=E+P*PWM*T/1000;
}

//-------------------------------------------------------------------------------
//----------------------------- Visibility Error --------------------------------
//-------------------------------------------------------------------------------

void visError(float L, float l){
  if(L-l>0){
    V=V+L-l;
  }
}

//-------------------------------------------------------------------------------
//------------------------------- Flicker Error ---------------------------------
//-------------------------------------------------------------------------------

void flicError(float l, float _pl, float _ppl){
  float f;
  if((l-_pl)*(_pl-_ppl)<0){
    f=((abs(l-_pl)+abs(_pl-_ppl))*1000)/(2*T);
  }else{
    f=0;
  }
  F=F+f;
}

//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
//------------------------- CALIBRACAO ------------------------------------------
//-------------------------------------------------------------------------------
float calibra(){
  float LUXsum=0;
  const int windowSize=10;
  for (int i = 0 ; i < windowSize; i += 1) {
    voltage = analogRead(analogInPin);
    LUXsum=LUXsum+readLUX(voltage);
  }
  float LUXmax = LUXsum/windowSize;
  return (LUXmax-exIlluminance)/100; // Devolve o ganho
}

unsigned long InitTime;
unsigned long CurrTime;



void calibracao(int idx, float* k, int id){
  int i = 1;
  while(i<=idx ){
    
    if(i == id){
      
      analogWrite(analogOutPin,255);
      delay(1000);
      k[i-1] = calibra();
      InitTime = millis();
      CurrTime = millis();
      while(CurrTime-InitTime<2000){
        write(2,id); // Calibrem-se estou aceso
        delay(100);
        CurrTime = millis();
      }
      analogWrite(analogOutPin,0);
      InitTime = millis();
      CurrTime = millis();
      while(CurrTime-InitTime<2000){
        write(3,id); // Avanca
        delay(100);
        CurrTime = millis();
      }
      i += 1;
    }
    else{
      InitTime = millis();
      CurrTime = millis();
      while(CurrTime-InitTime<2000){
        read(c, ident);
        if(ident == 2){
          k[c-1] = calibra();
        }
        CurrTime = millis();
      }

      while(ident!=3){
        read(c, ident);
      }
      i += 1;
    } 
  }
}


//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

//------------------------------------ SETUP ------------------------------------

void setup() {

  
  //-----------------------------------READ EEPROM----------------------------------
  id = EEPROM.read(address);
  address += sizeof(int);
  EEPROM.get(address,m);
  address += sizeof(float);
  EEPROM.get(address,b);
  
  MCP2515::ERROR err;
  Serial.begin(2000000);

  TCCR1B = (TCCR1B & maskpresc) | prescale;

  Serial.println("------------------------SETUP-------------------------");
  filt0 = (unsigned uint32_t) id << 2;
  Serial.println(filt0);
  Serial.println(m);
  Serial.println(b);
  SPI.begin();
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);

  mcp2515.setFilterMask(MCP2515::MASK0, 0, mask);
  mcp2515.setFilterMask(MCP2515::MASK1, 0, mask);
  mcp2515.setFilter(MCP2515::RXF0, 0, filt0); mcp2515.setFilter(MCP2515::RXF1, 0, 0x00000000);
  mcp2515.setFilter(MCP2515::RXF2, 0, filt0); mcp2515.setFilter(MCP2515::RXF3, 0, 0x00000000);
  
  // -------------------
  mcp2515.setNormalMode();
  //--------------------------------------------------------------------------------
  nodes[idx] = id;
  idx +=1;
  InitTime = millis();
  CurrTime = millis();
  while(CurrTime-InitTime<4000){ //4 segundos para se identificarem todos
    write(1, id);
    delay(100);
    if(read(c, ident) == MCP2515::ERROR_OK){
      if(ident == 1){ // Se for =1 é uma mensgem de identificacao 
        for(int i=0; i<10; i++){//Já existe;
          if(nodes[i]==c){
            break;
          }
          else if( i == 9){
            nodes[idx] = c;
            idx += 1;
          }
        }
      }
    }
    CurrTime = millis();
  }

  // ----------------------------FUNCIONA ATE AQUI ----------------------------------
  int i = 1 ;
  
  analogWrite(analogOutPin, 0);//turn off the led
  voltage = analogRead(analogInPin);
  exIlluminance = readLUX(voltage);
  calibracao(idx, k, id);
  // put your setup code here, to run once:
  gain = k[id-1];
  Serial.println(id);
  Serial.println(idx);
  for(int i =0 ; i<idx; i++){
    Serial.println(k[i]);
  }

  delay((3-id)*1000);

  float n,m;
  float c[3] = {0.0,0.0,0.0};
  c[id-1]=1.0;
  rho = 0.07;
  node.ID=id;
  node.c=c;
  node.k=k;
  node.o=exIlluminance;
  node.L=finalIluminance;
  n = node.norm(k);
  m = node.M();

  consensusFlag = true;
  consensus_chrono=millis();
  state=CONSENSUS;
  ff=(finalIluminance/gain)*255/100; // FEEDFORWARD
}

void runConsensus(){
  if((millis()-consensus_chrono)>3500){
    state=END;
  }
  switch(state){
    case SEND:
      {
        wait_chrono=millis();
        for(int i=0;i<idx;i++){
          if(nodes[i]!=id && sr[nodes[i]-1]!=2){
            sid=nodes[i];
            if(check[nodes[i]-1]==0){
              sident=0x00000010|(sid << 2)|id;
            }else{
              sident=0x00000020|(sid << 2)|id;
            }
            write(sident, message);
          }
        }
        state=WAIT;
      }
      break;
    case WAIT:
      {
        if(millis()-wait_chrono>5){
          int j=0;
          int l=0;
          for(int i=0;i<idx;i++){
            if(sr[nodes[i]-1]!=2&&nodes[i]!=id){
              state=SEND;
            }
            j=j+sr[i];
            l=l+check[i];
          }
          if(j==4){state=CONSENSUS;}
          if(l==2&&riter>niter){state=CONSENSUS;}
        }else{
          int j=0;
          int l=0;
          for(int i=0;i<idx;i++){
            j=j+sr[i];
            l=l+check[i];
          }
          if(j==4){state=CONSENSUS;}
          if(l==2&&riter>niter){state=CONSENSUS;}
        }
      }
      break;
    case CONSENSUS:
      {
        message=0x00000000;
        niter++;
        if(niter==50){
          sident=0x00000030;
          write(sident, message);
          state=END;
        }else{
          
          for(int i = 0; i < idx; i++){
            node.y[i]=node.y[i]+rho*(node.d[i]-node.d_av[i]);
          }
  
            float n = node.norm(k);
            float m = node.M();
            
          float *result;
          unsigned long tic=micros();
          result=node.consensus_iterate(node,rho);
          Serial.println(micros()-tic);
          for(int i = 0; i < idx; i++){
            if(result[i]<0){
              result[i]=0;
            }
            else if(result[i]>100){
              result[i]=100;
            }
            node.d[i]=result[i];
            node.d_av[i]=result[i]/idx;
  
            result[i]=(result[i]/100)*255;
            data=(uint32_t)result[i]; 
            data = data << i*8;
            message=message|data;
          }
          data=niter;
          data = data << 3*8;
          message=message|data;
          
          for(int i=0;i<idx;i++){
            check[i]=0;sr[i]=0;
          }
          //Serial.println(niter);
          state=SEND;
        }
      }
      break;
    case END:
      {
        Serial.print("---------------------------------------------****END CONSENSUS***>");
        Serial.println(millis()-consensus_chrono);
        Serial.println(niter);
        consensusFlag=false;
        Serial.println(node.d[0]);
        Serial.println(node.d[1]);
        Serial.println(node.d[2]);
        ff = (node.d[id-1]/100)*255;
        PIcontroller.reset();
        flickErrorCounter=0;
        niter=0;
      }
      break;
  }

  /***********Interpret Consensus Messages***************/
  if((rident&maskidc)==0x00000010){
    sid=(unsigned uint32_t)rident&maskid;
      riter = (unsigned uint32_t) dreceive&maskiter;
      riter = (unsigned uint32_t) riter >> 8*3;
      //Serial.print("iter");
      //Serial.print("\t");
      //Serial.println(riter);
    if(check[sid-1]==0&&riter==niter){
      uint32_t dr[3]={0,0,0};
      dr[0] = (unsigned uint32_t) dreceive&mask1;
      dr[1] = (unsigned uint32_t) dreceive&mask2;
      dr[2] = (unsigned uint32_t) dreceive&mask3;
      check[sid-1]=1;
      for(int i = 0; i < idx; i++){
        dr[i]=(unsigned uint32_t)dr[i]>>8*i;
        float aux = (((float)dr[i]/255)*100);
        node.d_av[i]=node.d_av[i]+aux/idx;
      }
    }
    sident=0x00000020|(sid << 2)|id;
    write(sident, message);
  }else if((rident&maskidc)==0x00000020){//acknowledge message
    sid=(unsigned uint32_t)rident&maskid;
      riter = (unsigned uint32_t) dreceive&maskiter;
      riter = (unsigned uint32_t) riter >> 8*3;
      //Serial.print("iter");
      //Serial.print("\t");
      //Serial.println(riter);
    if(riter==niter){
      sr[sid-1]=2;
      if(check[sid-1]==0){
        uint32_t dr[3]={0,0,0};
        dr[0] = (unsigned uint32_t) dreceive&mask1;
        dr[1] = (unsigned uint32_t) dreceive&mask2;
        dr[2] = (unsigned uint32_t) dreceive&mask3;
        check[sid-1]=1;
        for(int i = 0; i < idx; i++){
          dr[i]=(unsigned uint32_t)dr[i]>>8*i;
          float aux = (((float)dr[i]/255)*100);
          node.d_av[i]=node.d_av[i]+aux/idx;
        }
      }
    }
  }else if((rident&maskidc)==0x00000030){
    state=END;
  }
  
}

void loop() {

  // Le nova referencia  da serial input
  if (Serial.available() > 0) {
    hubFlag=true;
    // read the incoming byte:
    uint32_t input = Serial.parseInt();
    //if((input&maskcom)==0x00001000){
    Serial.println(input); 
    if(input>0){
      uint32_t rid = (unsigned uint32_t) input&maskr;
      rid = (unsigned uint32_t) rid >> 2;
      uint32_t data;
      Serial.println(rid);
      if((input&maskidc)==0){
        if(rid==id){
          Serial.print("--------------------------------------------->");
          finalIluminance=lowerBoundO;
          ff=(finalIluminance/gain)*255/100;
          consensusFlag = true;
          consensus_chrono=millis();
          state=CONSENSUS;
          node.L=finalIluminance;
              for (int i=0;i<idx;i++){
                if(i!=id-1){
                 ff=ff-(k[i]*node.d[i]*255)/100; 
                }
                node.d[i]=0;node.y[i]=0;node.d_av[i]=0;check[i]=0;sr[i]=0;
              }
          input = 0x00000300;
          write(input,0x00000000);
          Serial.println(input);
          flickErrorCounter=0;
        }else{
          write(0x00000300,rid);
          Serial.println(input);
          flickErrorCounter=0;
        }
      }else if((input&maskidc)==0x0000040){
        if(rid==id){
          Serial.println(illuminance);
        }else{
          input = (uint32_t)input|id;
          write(input,0);
        }
      }else if((input&maskidc)==0x0000050){
        if(rid==id){
          Serial.println(u);
        }else{
          input = (uint32_t)input|id;
          write(input,0);
        }    
      }else if((input&maskidc)==0x0000060){
        if(rid==id){
          Serial.println(exIlluminance);
        }else{
          input = (uint32_t)input|id;
          write(input,0);
        }   
      }else if((input&maskidc)==0x0000070){
        if(rid==id){
          Serial.println(finalIluminance);
        }else{
          input = (uint32_t)input|id;
          write(input,0);
        }   
      }else if((input&maskidc)==0x0000080){
        if(rid==id){
          Serial.println(1);
        }else{
          input = (uint32_t)input|id;
          write(input,0);
        }   
      }else if((input&maskidc)==0x0000090){
        if(rid==id){
          Serial.println(E);
        }else{
          input = (uint32_t)input|id;
          write(input,0);
        }   
      }else if((input&maskidc)==0x00000A0){
        if(rid==id){
          Serial.println(V);
        }else{
          input = (uint32_t)input|id;
          write(input,0);
        }   
      }else if((input&maskidc)==0x00000B0){
        if(rid==id){
          Serial.println(F);
        }else{
          input = (uint32_t)input|id;
          write(input,0);
        }   
      }
    }
  }
    
  MCP2515::ERROR e; 
  if(consensusFlag){
    runConsensus();
  }

  e=read(c, ident);
  if(e==MCP2515::ERROR_OK){
    if(ident&maskidc)==0x00000010|ident&maskidc)==0x00000020|ident&maskidc)==0x00000030){
      dreceive=c;
      rident=ident;
      riter = (unsigned uint32_t) dreceive&maskiter;
      riter = (unsigned uint32_t) riter >> 8*3;      
      if((!consensusFlag)&&riter==1){
          consensusFlag = true;
          consensus_chrono=millis();
          state=CONSENSUS;
          node.L=finalIluminance;
          for (int i=0;i<idx;i++){
            node.d[i]=0;node.y[i]=0;node.d_av[i]=0;check[i]=0;sr[i]=0;
          }
      }
    }else if((ident&maskidc)==0x00000300){
        if(hubFlag){
          Serial.println(c);
        }else if(c==id){
          finalIluminance=lowerBoundO;
          ff=(finalIluminance/gain)*255/100;
          flickErrorCounter=0;
          consensusFlag = true;
          consensus_chrono=millis();
          state=CONSENSUS;
          node.L=finalIluminance;
          for (int i=0;i<idx;i++){
            if(i!=id-1){
             ff=ff-(k[i]*node.d[i]*255)/100; 
            }
            node.d[i]=0;node.y[i]=0;node.d_av[i]=0;check[i]=0;sr[i]=0;
          }
        }else{
          consensusFlag = true;
          consensus_chrono=millis();
          state=CONSENSUS;
          node.L=finalIluminance;
          for (int i=0;i<idx;i++){
            node.d[i]=0;node.y[i]=0;node.d_av[i]=0;check[i]=0;sr[i]=0;
          }
        }
    }else if((ident&maskidc)==0x00000040){
        if(hubFlag){
          Serial.println(c);
        }else{
          sid = (unsigned uint32_t)ident&maskid;
          sid = (unsigned uint32_t)sid<<2;
          uint32_t data=(unsigned uint32_t)ident&0xFFFFFFF0;
          data = data|sid;
          data = data|id;
          write(data,illuminance);
        }
    }else if((ident&maskidc)==0x00000050){
        if(hubFlag){
          Serial.println(c);
        }else{
          sid = (unsigned uint32_t)ident&maskid;
          sid = (unsigned uint32_t)sid<<2;
          uint32_t data=(unsigned uint32_t)ident&0xFFFFFFF0;
          data = data|sid;
          data = data|id;
          write(data,u);
        }
    }else if((ident&maskidc)==0x00000060){
        if(hubFlag){
          Serial.println(c);
        }else{
          sid = (unsigned uint32_t)ident&maskid;
          sid = (unsigned uint32_t)sid<<2;
          uint32_t data=(unsigned uint32_t)ident&0xFFFFFFF0;
          data = data|sid;
          data = data|id;
          write(data,exlIlluminance);
        }
    }else if((ident&maskidc)==0x00000070){
        if(hubFlag){
          Serial.println(c);
        }else{
          sid = (unsigned uint32_t)ident&maskid;
          sid = (unsigned uint32_t)sid<<2;
          uint32_t data=(unsigned uint32_t)ident&0xFFFFFFF0;
          data = data|sid;
          data = data|id;
          write(data,finalIluminance);
        }
    }else if((ident&maskidc)==0x00000080){
        if(hubFlag){
          Serial.println(c);
        }else{
          sid = (unsigned uint32_t)ident&maskid;
          sid = (unsigned uint32_t)sid<<2;
          uint32_t data=(unsigned uint32_t)ident&0xFFFFFFF0;
          data = data|sid;
          data = data|id;
          write(data,1);
        }
    }else if((ident&maskidc)==0x00000090){
        if(hubFlag){
          Serial.println(c);
        }else{
          sid = (unsigned uint32_t)ident&maskid;
          sid = (unsigned uint32_t)sid<<2;
          uint32_t data=(unsigned uint32_t)ident&0xFFFFFFF0;
          data = data|sid;
          data = data|id;
          write(data,E);
        }
    }else if((ident&maskidc)==0x000000A0){
        if(hubFlag){
          Serial.println(c);
        }else{
          sid = (unsigned uint32_t)ident&maskid;
          sid = (unsigned uint32_t)sid<<2;
          uint32_t data=(unsigned uint32_t)ident&0xFFFFFFF0;
          data = data|sid;
          data = data|id;
          write(data,V);
        }
    }else if((ident&maskidc)==0x000000B0){
        if(hubFlag){
          Serial.println(c);
        }else{
          sid = (unsigned uint32_t)ident&maskid;
          sid = (unsigned uint32_t)sid<<2;
          uint32_t data=(unsigned uint32_t)ident&0xFFFFFFF0;
          data = data|sid;
          data = data|id;
          write(data,F);
        }
    }
  }
  unsigned long currentTime = millis();
  
  if(currentTime - previousTime >= T){
    
    ppl=pl;
    pl=illuminance;
    
    // READ A/D -----------------------------------------------------
    float LUXsum=0;
    for(int i=0; i<5; i++){
      voltage = analogRead(analogInPin);
      LUXsum=LUXsum+readLUX(voltage);
    }
    illuminance = LUXsum/5;
    // --------------------------------------------------------------

    float fb = PIcontroller.calc(finalIluminance,illuminance); // FEEDBACK  
    
    u = ff + fb; // CONTROL = FEEDFORWARD+FEEDBACK

    // ------------------Write D/A-------------------------
    if(u<=255 && u>=0){
      analogWrite(analogOutPin, round(u));
    }
    else if(u<0){
      u = 0;
      analogWrite(analogOutPin, round(u)); //Saturacao negativa
    }
    else{
      u = 255;
      analogWrite(analogOutPin, round(u)); //Saturacao positiva
    }
    
    // ---------------------------------------------------

    PIcontroller.updat(finalIluminance,illuminance,ff);


    Serial.print(u);
    Serial.print("\t");
    Serial.print(exIlluminance);
    Serial.print("\t");
    Serial.print(illuminance);
    Serial.print("\t");
    Serial.print(finalIluminance);
    Serial.print("\t");
    Serial.println(millis()-currentTime);
    previousTime = currentTime;
  }
}
