/*
 * GPS도 없고 통신장치도 Lipo-rider도 없는 버젼
 * Light Sensor Connection:
     VCC-5v
     GND-GND
     SCL-SCL(analog pin 5)
     SDA-SDA(analog pin 4)
     ADD-NC or GND
 *
 */
 
//상수
#define DEG_TO_RAD 0.01745329
#define PI 3.141592654
#define TWOPI 6.28318531
//서보
#include <Servo.h>
//광센서
#include <Wire.h> //BH1750 IIC Mode 
#include <math.h> 

//광센서
//
int BH1750address = 0x23; //setting i2c address
byte buff[2];
uint16_t lightsensor; //(lux)
float lightenergy; //(W)

const float lmpW=93; //태양빛은 93(lm/W)의 효율을 보인다.
const float panelarea = 0.013; //(m^2)
const float lightsensorcap = 65535.0; // 광센서의 측정한계(lux)
int overcaperror;

//enableinterrupt 는 softwareserial과 충동, altsoftserial은 servo와 충돌

//태양 계산
int hour=0, localhour;
int minute=0, second=0, month=6, day=21;
int year=2018;
float latitude = 37;
float longitude = 127;
float zone0;
int zone;
float Lon, Lat;
float T,JD_frac,L0,M,e,C,L_true,f,R,GrHrAngle,Obl,RA,Decl,HrAngle,elev,azimuth;
long JD_whole,JDx;

//서보모터
float xyangle0, zangle0;
int xyangle, zangle;

const int xyservoPin = 10;
const int zservoPin = 11;

//전력계산
const int Vmeterin[6] = {0,1,2,3,4,5};  //연결단자 번호
int Vmeter[6];  //측정값
float V[6];  //전압값
float Vs, Is, Ps, Vb, Ib, Pb;
const float R1 = 10000.00;
const float R2 = 1000.00;
const float R3 = 220.00;

//시간 값 입력에 필요한 함수
int timeinput;
int textcount;

float panelef;

int inputerror=0;

Servo xyservo;
Servo zservo;


void setup() {
  Wire.begin();
  Serial.begin(9600);
  xyservo.attach(xyservoPin);
  zservo.attach(zservoPin);
 // xyservo.write(0);
  zservo.write(0);//서보모터 위치 초기화
  Serial.println("태양광패널 제어장치");
  Serial.println("2018년 6월 제작");
  Serial.println("경희대학교 물리학과 이성이, 오현식");
  Serial.print("설정된위치정보 : 경도");
  Serial.print(longitude);
  Serial.print(" 위도");
  Serial.println(latitude);
  Serial.print("설정된일자정보 : ");
  Serial.print(year);
  Serial.print("년 ");
  Serial.print(month);
  Serial.print("월 ");
  Serial.print(day);
  Serial.print("일 ");
  Serial.println("시간정보를 아래와 같은 양식으로 입력해 주십시오");
  Serial.println("hhmm");
  Serial.println("ex) 1201  (12시 1분)");
  Serial.println("");
}

void loop() {

  if(Serial.available()){//시리얼에 입력이 있으면 작동하는 조건문

    timereader(); //시리얼에 입력된 시간, 위치 정보를 받아옴 + 입력에 오류가 있다고 판단되면 에러 문구 출력
    Serial.end();
    Serial.begin(9600); // 시리얼 버퍼 초기화
    
    if(inputerror==0){                //시간 입력에 오류가 없을 경우
      do{                            //다음 시간 입력이 있을 때까지 아래 행동 반복
      sunposition();                  //azimuth, elev 계산
      servocontrol();                 //서보모터 컨트롤
      for(int k=0; k<2; k++){        // (1분)/(출력간격) = 12
        Power();                     //전력측정
        light();                     //태양광 측정 및 에너지 전환효율 계산
        finalprint();
        delay(4800);                  //여기 있는 시간 + 200 (ms) = 출력간격
      }
  
      if(minute =! 59){              //입력 시간에 1분추가
        minute +=1;
      }else{
        minute =0;
        if(hour =! 23){
          hour +=1;
        }else{
          hour=0;
        }
      }
      }while(Serial.available()==0);
    }
  }
}


//태양 위치 계산
void sunposition(){
  zone0 = (longitude*12)/180;
  zone = ((int)zone0) +1;
  hour = localhour - zone ;
  Lon=longitude*DEG_TO_RAD, Lat=latitude*DEG_TO_RAD;
  JD_whole=JulianDate(year,month,day);
  JD_frac=(hour+minute/60.+second/3600.)/24.-.5;
  T=JD_whole-2451545; T=(T+JD_frac)/36525.;
  L0=DEG_TO_RAD*fmod(280.46645+36000.76983*T,360);
  M=DEG_TO_RAD*fmod(357.5291+35999.0503*T,360);
  e=0.016708617-0.000042037*T;
  C=DEG_TO_RAD*((1.9146-0.004847*T)*sin(M)+(0.019993-0.000101*T)*sin(2*M)+0.00029*sin(3*M));
  f=M+C;
  Obl=DEG_TO_RAD*(23+26/60.+21.448/3600.-46.815/3600*T);
  JDx=JD_whole-2451545;
  GrHrAngle=280.46061837+(360*JDx)%360+.98564736629*JDx+360.98564736629*JD_frac;
  GrHrAngle=fmod(GrHrAngle,360.);
  L_true=fmod(C+L0,TWOPI);
  R=1.000001018*(1-e*e)/(1+e*cos(f));
  RA=atan2(sin(L_true)*cos(Obl),cos(L_true));
  Decl=asin(sin(Obl)*sin(L_true));
  HrAngle=DEG_TO_RAD*GrHrAngle+Lon-RA;
  elev=asin(sin(Lat)*sin(Decl)+cos(Lat)*(cos(Decl)*cos(HrAngle)));
  // Azimuth measured eastward from north.
  azimuth=PI+atan2(sin(HrAngle),cos(HrAngle)*sin(Lat)-tan(Decl)*cos(Lat));

  //hour 는 그리니치 시간(gps)
  //현재 현지 시간은 hour+zone
}

long JulianDate(int year, int month, int day) {
long JD_whole;
int A,B;
if (month<=2) {
year--; month+=12;
}
A=year/100; B=2-A+A/4;
JD_whole=(long)(365.25*(year+4716))+(int)(30.6001*(month+1))+day+B-1524;
return JD_whole;
}

//전력측정
void Power(){
  //전압계
  for(int i=0; i<2; i++){
    Vmeter[i] = analogRead(Vmeterin[i]);  //0번 전압계를 읽는다
    V[i] = ((Vmeter[i] * 5.00)/1024.00)*(R1+R2)/R2; //그라운드와의 전위차계산
    if(V[i]<0.04){
      V[i]=0.00;//노이즈 제거
    }
  }
  
  //전위차
  Vs = V[1] - V[0];

  //전류
  Is = (V[1] - V[0]) / R3 ;

  //일률
  Ps = Is * Vs;
}

// 태양광 측정
void light(){
  int j;
  lightsensor=0;
  BH1750_Init(BH1750address);
  delay(200);
 
  if(2==BH1750_Read(BH1750address))
  {
    lightsensor=((buff[0]<<8)|buff[1])/1.2;
    lightenergy= (lightsensor * panelarea) / lmpW;
    if(lightsensor > (lightsensorcap - 100)){
      lightsensor = lightsensorcap;
      overcaperror = 1;
    }else{
      overcaperror = 0;
    }
    panelef = (Ps / lightenergy) * 100;//(%)
  }
}

int BH1750_Read(int address) //
{
  int j=0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while(Wire.available()) //
  {
    buff[j] = Wire.read();  // receive one byte
    j++;
  }
  Wire.endTransmission();  
  return j;
} 
void BH1750_Init(int address) 
{
  Wire.beginTransmission(address);
  Wire.write(0x10);//1lx reolution 120ms
  Wire.endTransmission();
}

void hourreader()
{
  localhour = Serial.parseInt();
  year = 2018;
  month = 6;
  day = 21;
  minute = 0;
  second = 0;
}

void timereader(){
  timeinput = Serial.parseInt();
    localhour = (int) (timeinput/100);
    minute = timeinput - (localhour * 100);
    second = 0;
    inputerror = 0;
    if(timeinput>2400){
      inputerror = 1;
    }
}

void servocontrol(){
      xyangle0 = (((azimuth/DEG_TO_RAD) - 90)/2);
      xyangle = (int) ((8*xyangle0)/9);
      xyangle +=10;
      if(xyangle < 0){
        xyangle = 0;
      }
      if(xyangle > 170){
        xyangle = 170;
      }
      //90~270
      zangle0 = elev/DEG_TO_RAD;
      zangle = (int) (zangle0 * 0.9);
      if(zangle < 0){
        zangle =0;
      }
      
      //0~90
  
      //최종적인 서보모터 회전
     // xyservo.write(xyangle);
      zservo.write(zangle);
}

void finalprint(){
  Serial.print("time input : ");
  Serial.println(timeinput);
  Serial.print("Power from solar panel : "); //태양전지판의 전력 출력
  Serial.print(Ps,5);
  Serial.print("W = ");
  Serial.print(Vs,5);
  Serial.print("V * ");
  Serial.print(Is,5);
  Serial.println("Amp");
  Serial.println("");
  if(overcaperror==0){
    if(lightsensor==0){//광센서 연결에 문제가 있다면
      Serial.println("Pls Check the lightsensor connection");
    }else{//
      Serial.print("Power generation efficiency : ");
      Serial.print(panelef);
      Serial.println(" %");
    }
  }else{//태양빛이 너무 세서 광센서의 측정한계를 넘을 경우
    Serial.println("THE SUN IS TOO HOT!");
  }
  Serial.print("Sun position : "); //태양의 방위, 고도 출력
  Serial.print(azimuth/DEG_TO_RAD,2);
  Serial.print(" , ");
  Serial.println(elev/DEG_TO_RAD,2);
  Serial.println("");  
}

