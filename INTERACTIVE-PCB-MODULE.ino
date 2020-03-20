//************************************************************************************************************//
  
//************************************ THE 8x8 INTERACTIVE RGB LED MODULE ************************************//

//************************************************************************************************************//
#include <SPI.h>
#include "font8x8.h"

#define blank_pin   2     // Defines actual BIT of PortD for blank/OE - is Arduino NANO pin 2
#define latch_pin   3     // Defines actual BIT of PortD for latch - is Arduino NANO pin 3
#define clock_pin   13    // used by SPI, must be 13 SCK 13 on Arduino NANO
#define data_pin    11    // used by SPI, must be pin MOSI 11 on Arduino NANO

#define RowA_Pin    4     // 74HC138 - A
#define RowB_Pin    5     // 74HC138 - B
#define RowC_Pin    6     // 74HC138 - C

//*************************************************PHOTO-TRANSISTOR******************************************//
#define IR_Enable             7 
#define IR_Select_C           8   
#define IR_Select_B           9   
#define IR_Select_A           10 

#define BAUDRATE        9600              // SERIAL BAURATE
#define CALIB           1                 // CALIB = 1 for Calibration
#define IR_PRINT        0                 // Print working IR phototransistor on Serial Port      

#define ProximityNoise  100
#define ProximityLimit  400
#define FadeTime        30                // RGB LED fading time in microsecond
#define BAUDRATE        9600              // SERIAL BAURATE

volatile int IR_read_data[16];            // Working IR value Array - Normal Operation
volatile int IR_calib_low[16];            // Calibration LOW Array - Cover all phototransistors in 5 seconds
volatile int IR_calib_high[16];           // Calibration HIGH Array - Put all phtotransistors free in 5 seconds
volatile int IR_average[16];              // Everage value
volatile byte IR_counter = 0;             // Tracking IR number
            
unsigned long samplingtime = 0;
unsigned long samplingfadetime = 0;

wchar_t Calib_HIGH[]=L"    * HIGH - Don't Touch  *    "; 
wchar_t Calib_LOW[]=L"    * LOW - Cover Phototransistors  *    "; 
wchar_t Calib_FINISH[]=L"    * COMPLETED  *    ";
  
//***************************************************BAM Variables******************************************//
byte red[4][8];
byte green[4][8];
byte blue[4][8];

int level=0; // Keeps track of which level we are shifting data to
int row=0;
int BAM_Bit, BAM_Counter=0; // Bit Angle Modulation variables to keep track of things

#define myPI      3.14159265358979323846
#define myDPI     1.2732395
#define myDPI2    0.40528473
#define dist(a, b, c, d) sqrt(double((a - c) * (a - c) + (b - d) * (b - d)))

//***********************************************Defining the Matrix***************************************//

#define BAM_RESOLUTION 4    // EG 4 bit colour = 15 variation of R, G & B (16^3 = 4096 colours)
const  byte Size_X = 8;    // Number of Column X axis
const  byte Size_Y = 8;     // Number of Row Y axis

#define COLOUR_WHEEL_LENGTH 256

uint8_t colourR[COLOUR_WHEEL_LENGTH];
uint8_t colourG[COLOUR_WHEEL_LENGTH];
uint8_t colourB[COLOUR_WHEEL_LENGTH];
int16_t ColPos = 0;
uint16_t colourPos;
uint8_t R, G, B;
byte myred, mygreen, myblue;

/**   An RGB color template */
struct Color
{
  unsigned char red, green, blue;

  Color(int r, int g, int b) : red(r), green(g), blue(b) {}
  Color() : red(0), green(0), blue(0) {}
};

const Color redcolor        = Color(0x0F, 0x00, 0x00);
const Color orangecolor     = Color(0x0F, 0x0F, 0x00);
const Color yellowcolor     = Color(0x0F, 0x09, 0x00);
const Color greencolor      = Color(0x00, 0x0F, 0x00);
const Color bluecolor       = Color(0x00, 0x00, 0x0F);
const Color clearcolor       = Color(0x00, 0x00, 0x00);
const Color whitecolor       = Color(0x0F, 0x0F, 0x0F);

#define RED     0x0F,0x00,0x00
#define ORANGE  0x0F,0x04,0x00
#define YELLOW  0x0F,0x09,0x00
#define GREEN   0x00,0x0F,0x00
#define TEAL    0x00,0x0F,0x04
#define BLUE    0x00,0x00,0x0F
#define PURPLE  0x0F,0x00,0x0F
#define WHITE   0x0F,0x0F,0x0F
#define CLEAR   0x00,0x00,0x00


//************************************************PHOTOTRANSISTOR & LED ZONE*****************************************************//
byte Zone00[4][2] = {{0, 1}, {1, 1}, {0, 0}, {1, 0}};
byte Zone01[4][2] = {{2, 1}, {3, 1}, {2, 0}, {3, 0}};
byte Zone02[4][2] = {{4, 1}, {5, 1}, {4, 0}, {5, 0}};
byte Zone03[4][2] = {{6, 1}, {7, 1}, {6, 0}, {7, 0}};
byte Zone04[4][2] = {{0, 3}, {1, 3}, {0, 2}, {1, 2}};
byte Zone05[4][2] = {{2, 3}, {3, 3}, {2, 2}, {3, 2}};
byte Zone06[4][2] = {{4, 3}, {5, 3}, {4, 2}, {5, 2}};
byte Zone07[4][2] = {{6, 3}, {7, 3}, {6, 2}, {7, 2}};
byte Zone08[4][2] = {{0, 5}, {1, 5}, {0, 4}, {1, 4}};
byte Zone09[4][2] = {{2, 5}, {3, 5}, {2, 4}, {3, 4}};
byte Zone10[4][2] = {{4, 5}, {5, 5}, {4, 4}, {5, 4}};
byte Zone11[4][2] = {{6, 5}, {7, 5}, {6, 4}, {7, 4}};
byte Zone12[4][2] = {{0, 7}, {1, 7}, {0, 6}, {1, 6}};
byte Zone13[4][2] = {{2, 7}, {3, 7}, {2, 6}, {3, 6}};
byte Zone14[4][2] = {{4, 7}, {5, 7}, {4, 6}, {5, 6}};
byte Zone15[4][2] = {{6, 7}, {7, 7}, {6, 6}, {7, 6}};

void setup()
{
SPI.setBitOrder(MSBFIRST);
SPI.setDataMode(SPI_MODE0);
SPI.setClockDivider(SPI_CLOCK_DIV2);

noInterrupts();

TCCR1A = B00000000;
TCCR1B = B00001011;
TIMSK1 = B00000010;
OCR1A = 10;

pinMode(latch_pin, OUTPUT);
//pinMode(blank_pin, OUTPUT);
pinMode(data_pin, OUTPUT);
pinMode(clock_pin, OUTPUT);

pinMode(RowA_Pin, OUTPUT);
pinMode(RowB_Pin, OUTPUT);
pinMode(RowC_Pin, OUTPUT);

pinMode(IR_Enable, OUTPUT);
digitalWrite(IR_Enable, LOW);

pinMode(IR_Select_C, OUTPUT);
pinMode(IR_Select_B, OUTPUT);
pinMode(IR_Select_A, OUTPUT);

Serial.begin(BAUDRATE);
SPI.begin();
interrupts();

fill_colour_wheel();
clearfast();

if (CALIB)
  {
    hScroll(0, greencolor, redcolor, Calib_HIGH, 100, 1, 1); // hScroll(uint8_t y, Color For_color, Color Bk_color, wchar_t *mystring, uint8_t font, uint8_t delaytime, uint8_t times, uint8_t dir)
    clearfast();
    fillTable(WHITE);
    delay(3000);
    Calib_High();
    delay(1000);
    hScroll(0, greencolor, redcolor, Calib_FINISH, 100, 1, 1);
    clearfast();
    hScroll(0, greencolor, orangecolor, Calib_LOW, 100, 1, 1);
    clearfast();
    delay(3000);
    Calib_Low();
    delay(1000);
    hScroll(0, greencolor, orangecolor, Calib_FINISH, 100, 1, 1);
    clearfast();
    for (byte i=0; i<16; i++)
      {
        IR_average[i] = (((IR_calib_low[i] + IR_calib_high[i])/2) - ProximityNoise);
      }
    for (byte i=0; i<16; i++)
      {
        Serial.print("IR_average[");
        Serial.print(i);
        Serial.print("] = ");
        Serial.print(IR_average[i]);
        Serial.print(" ;");
        Serial.println("");
      }
    }
else
  {
    for (byte i=0; i<16; i++)
      {
        IR_average[i] = ProximityLimit;
      }
  }
}

void loop()
{ 
  
  //Interactive(whitecolor, redcolor);
  Interactive_Colorwheel();
}

void LED(int X, int Y, int R, int G, int B)
{
  X = constrain(X, 0, 7); 
  Y = constrain(Y, 0, 7);
  
  R = constrain(R, 0, 15);
  G = constrain(G, 0, 15); 
  B = constrain(B, 0, 15);

  for (byte BAM = 0; BAM < BAM_RESOLUTION; BAM++) 
  {
    bitWrite(red[BAM][X], Y, bitRead(R, BAM));

    bitWrite(green[BAM][X], Y, bitRead(G, BAM));

    bitWrite(blue[BAM][X], Y, bitRead(B, BAM));
  }
}

void rowScan(byte row)
{
  
  if (row & 0x01) PORTD |= 1<<RowA_Pin;
    else PORTD &= ~(1<<RowA_Pin);
  
  if (row & 0x02) PORTD |= 1<<RowB_Pin;
    else PORTD &= ~(1<<RowB_Pin);

  if (row & 0x04) PORTD |= 1<<RowC_Pin;
    else PORTD &= ~(1<<RowC_Pin);
}

ISR(TIMER1_COMPA_vect){
    
PORTD |= ((1<<blank_pin));
if(BAM_Counter==8)
BAM_Bit++;
else
if(BAM_Counter==24)
BAM_Bit++;
else
if(BAM_Counter==56)
BAM_Bit++;

BAM_Counter++;

switch (BAM_Bit)
{
    case 0:     
      //Blue        
        SPI.transfer(blue[0][level]);      
      //Green        
        SPI.transfer(green[0][level]);     
      //Red     
        SPI.transfer(red[0][level]);
      break;
    case 1:
      //Blue       
        SPI.transfer(blue[1][level]);
      //Green
        SPI.transfer(green[1][level]);
      //Red
        SPI.transfer(red[1][level]);       
      break;
    case 2:     
      //Blue      
        SPI.transfer(blue[2][level]);       
      //Green
        SPI.transfer(green[2][level]);      
      //Red
        SPI.transfer(red[2][level]);
      break;
    case 3:
      
      //Blue     
        SPI.transfer(blue[3][level]);     
      //Green
        SPI.transfer(green[3][level]);      
      //Red
        SPI.transfer(red[3][level]);
        
  if(BAM_Counter==120){
  BAM_Counter=0;
  BAM_Bit=0;
  }
  break;
}

rowScan(level);
PORTD |= 1<<latch_pin;
PORTD &= ~(1<<latch_pin);
delayMicroseconds(2); 
PORTD &= ~(1<<blank_pin);
level++;
if(level==8)
level=0;
//pinMode(blank_pin, OUTPUT);
DDRD |= ((1<<blank_pin));
}

void clearfast()
{
    memset(red, 0, sizeof(red[0][0]) * 4 * 8);
    memset(green, 0, sizeof(green[0][0]) * 4 * 8);
    memset(blue, 0, sizeof(blue[0][0]) * 4 * 8);
}

void fillTable(byte R, byte G, byte B)
{
    for (byte x=0; x<8; x++)
    {
      for (byte y=0; y<8; y++)
      {
        LED(x, y, R, G, B);
      }
    }
}


//*******************************************************COLORWHEEL*****************************************************//

//FAST SINE APPROX
float mySin(float x){
  float sinr = 0;
  uint8_t g = 0;

  while(x > myPI){
    x -= 2*myPI; 
    g = 1;
  }

  while(!g&(x < -myPI)){
    x += 2*myPI;
  }

  sinr = myDPI*x - myDPI2*x*myAbs(x);
  sinr = 0.225*(sinr*myAbs(sinr)-sinr)+sinr;

  return sinr;
}

//FAST COSINE APPROX
float myCos(float x){
  return mySin(x+myPI/2);
}

float myTan(float x){
  return mySin(x)/myCos(x);
}

//SQUARE ROOT APPROX
float mySqrt(float in){
  int16_t d = 0;
  int16_t in_ = in;
  float result = 2;
  
  for(d = 0; in_ > 0; in_ >>= 1){
    d++;
  }
  
  for(int16_t i = 0; i < d/2; i++){
    result = result*2;
  }
  
  for(int16_t i = 0; i < 3; i++){
    result = 0.5*(in/result + result);
  }
  
  return result;
}

//MAP NUMBERS TO NEW RANGE
float myMap(float in, float inMin, float inMax, float outMin, float outMax){
  float out;
  out = (in-inMin)/(inMax-inMin)*(outMax-outMin) + outMin;
  return out;
}

//ROUND A NUMBER
int16_t myRound(float in){
  int8_t s = in/myAbs(in);
  return (int16_t)(s*(myAbs(in) + 0.5));
}

//ABSOLUTE VALUE
float myAbs(float in){
  return (in)>0?(in):-(in);
} 

void fill_colour_wheel(void) 
{
  float red, green, blue;
  float c, s;
  int32_t phase = 0;
  int16_t I = 0;

  while (phase < COLOUR_WHEEL_LENGTH) 
  {
    s = (1 << BAM_RESOLUTION)*mySin(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));
    c = (1 << BAM_RESOLUTION)*myCos(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));

    red = (I == 0 ? 1 : 0)*s + (I == 1 ? 1 : 0)*c;
    green = (I == 1 ? 1 : 0)*s + (I == 2 ? 1 : 0)*c;
    blue = (I == 2 ? 1 : 0)*s + (I == 0 ? 1 : 0)*c;

    colourR[phase] = red;
    colourG[phase] = green;
    colourB[phase] = blue;

    if (++phase >= (1 + I)*COLOUR_WHEEL_LENGTH / 3) 
      I++;
  }
}

void get_colour(int16_t p, uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (p >= COLOUR_WHEEL_LENGTH)
    p -= COLOUR_WHEEL_LENGTH;

  *R = colourR[p];
  *G = colourG[p];
  *B = colourB[p];
}

void get_next_colour(uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (++ColPos >= COLOUR_WHEEL_LENGTH)
    ColPos -= COLOUR_WHEEL_LENGTH;

  *R = colourR[ColPos];
  *G = colourG[ColPos];
  *B = colourB[ColPos];
}

void increment_colour_pos(uint8_t i)
{
  colourPos += i;
  while (colourPos >= COLOUR_WHEEL_LENGTH)
  {
    colourPos -= COLOUR_WHEEL_LENGTH;
  }
}

Color getcolorPixel(int X, int Y) {
  // Check parameters
  Color pixelColor;
  X = constrain(X, 0, 7);
  Y = constrain(Y, 0, 7);
  
  // RED

  bitWrite(pixelColor.red, 0, bitRead(red[0][X], Y));
  bitWrite(pixelColor.red, 1, bitRead(red[1][X], Y));
  bitWrite(pixelColor.red, 2, bitRead(red[2][X], Y));
  bitWrite(pixelColor.red, 3, bitRead(red[3][X], Y));

  // GREEN

  bitWrite(pixelColor.green, 0, bitRead(green[0][X], Y));
  bitWrite(pixelColor.green, 1, bitRead(green[1][X], Y));
  bitWrite(pixelColor.green, 2, bitRead(green[2][X], Y));
  bitWrite(pixelColor.green, 3, bitRead(green[3][X], Y));

  // BLUE

  bitWrite(pixelColor.blue, 0, bitRead(blue[0][X], Y));
  bitWrite(pixelColor.blue, 1, bitRead(blue[1][X], Y));
  bitWrite(pixelColor.blue, 2, bitRead(blue[2][X], Y));
  bitWrite(pixelColor.blue, 3, bitRead(blue[3][X], Y));

  return pixelColor;
}

void SetLightZone(byte Zone, byte R, byte G, byte B)
{
switch (Zone)
  {
    case 0:   DrawLight(Zone00, R, G, B); break;
    case 1:   DrawLight(Zone01, R, G, B); break;
    case 2:   DrawLight(Zone02, R, G, B); break;
    case 3:   DrawLight(Zone03, R, G, B); break;
    case 4:   DrawLight(Zone04, R, G, B); break;
    case 5:   DrawLight(Zone05, R, G, B); break;
    case 6:   DrawLight(Zone06, R, G, B); break;
    case 7:   DrawLight(Zone07, R, G, B); break;
    case 8:   DrawLight(Zone08, R, G, B); break;
    case 9:   DrawLight(Zone09, R, G, B); break;
    case 10:  DrawLight(Zone10, R, G, B); break;
    case 11:  DrawLight(Zone11, R, G, B); break;
    case 12:  DrawLight(Zone12, R, G, B); break;
    case 13:  DrawLight(Zone13, R, G, B); break;
    case 14:  DrawLight(Zone14, R, G, B); break;
    case 15:  DrawLight(Zone15, R, G, B); break;
  }
}

void DrawLight(byte zone_dots[4][2], byte R, byte G, byte B)
{
  for (int i = 0; i < 4; i++)
  {
    LED(zone_dots[i][1], zone_dots[i][0], R, G, B);
  }
}



void SetLightColorwheel(byte zone_dots[4][2])
{  
  for (int i = 0; i < 4; i++)
  {
    get_colour(colourPos + zone_dots[i][1]* 16 + zone_dots[i][0]* 16, &R, &G, &B);
    LED(zone_dots[i][1], zone_dots[i][0], R, G, B);
  }
}

void SetLightZoneColorwheel(byte Zone)
{
switch (Zone)
  {
    case 0:   SetLightColorwheel(Zone00); break;
    case 1:   SetLightColorwheel(Zone01); break;
    case 2:   SetLightColorwheel(Zone02); break;
    case 3:   SetLightColorwheel(Zone03); break;
    case 4:   SetLightColorwheel(Zone04); break;
    case 5:   SetLightColorwheel(Zone05); break;
    case 6:   SetLightColorwheel(Zone06); break;
    case 7:   SetLightColorwheel(Zone07); break;
    case 8:   SetLightColorwheel(Zone08); break;
    case 9:   SetLightColorwheel(Zone09); break;
    case 10:  SetLightColorwheel(Zone10); break;
    case 11:  SetLightColorwheel(Zone11); break;
    case 12:  SetLightColorwheel(Zone12); break;
    case 13:  SetLightColorwheel(Zone13); break;
    case 14:  SetLightColorwheel(Zone14); break;
    case 15:  SetLightColorwheel(Zone15); break;
  }
}

void FadeLight(byte zone_dots[4][2])
{
  Color pixelColor;
  if ( (unsigned long) (micros() - samplingfadetime) > FadeTime)
    { 
    for (int i = 0; i < 4; i++)
    {
    pixelColor = getcolorPixel(zone_dots[i][1], zone_dots[i][0]);     
    if(pixelColor.red > 0)
      pixelColor.red--;
    if(pixelColor.green > 0)
      pixelColor.green--;
    if(pixelColor.blue > 0)
      pixelColor.blue--;
      LED(zone_dots[i][1], zone_dots[i][0], pixelColor.red, pixelColor.green, pixelColor.blue);      
    }
    samplingfadetime = micros();      
  }  
}

void FadeLightZone(byte Zone)
{
switch (Zone)
  {
    case 0:   FadeLight(Zone00); break;
    case 1:   FadeLight(Zone01); break;
    case 2:   FadeLight(Zone02); break;
    case 3:   FadeLight(Zone03); break;
    case 4:   FadeLight(Zone04); break;
    case 5:   FadeLight(Zone05); break;
    case 6:   FadeLight(Zone06); break;
    case 7:   FadeLight(Zone07); break;
    case 8:   FadeLight(Zone08); break;
    case 9:   FadeLight(Zone09); break;
    case 10:  FadeLight(Zone10); break;
    case 11:  FadeLight(Zone11); break;
    case 12:  FadeLight(Zone12); break;
    case 13:  FadeLight(Zone13); break;
    case 14:  FadeLight(Zone14); break;
    case 15:  FadeLight(Zone15); break;
  }
}
void Read_phototransistor()
{
  //if ( (unsigned long) (micros() - samplingtime) > 350  )
  //{
    for (IR_counter=0; IR_counter< 8; IR_counter++)
      {  
        digitalWrite(IR_Select_C, bitRead(IR_counter, 2));
        digitalWrite(IR_Select_B, bitRead(IR_counter, 1));
        digitalWrite(IR_Select_A, bitRead(IR_counter, 0));
        IR_read_data[0 + IR_counter] = analogRead(A0);
        IR_read_data[8 + IR_counter] = analogRead(A1); 
      } 
    //samplingtime = micros();
  //}
}
void Interactive(Color setcolor, Color bgcolor)
{
    Read_phototransistor();
    for (byte i=0; i<16; i++){
    if (IR_read_data[i] >= IR_average[i]) 
    {  
      SetLightZone(i, setcolor.red, setcolor.green, setcolor.blue);
    }
    else
    {    
      SetLightZone(i, bgcolor.red, bgcolor.green, bgcolor.blue);
    }
  }
}

void Interactive_Colorwheel()
{
    Read_phototransistor();
    for (byte i=0; i<16; i++){
    if (IR_read_data[i] >= IR_average[i]) 
    {  
      SetLightZoneColorwheel(i);
    }
    else
    {    
      FadeLightZone(i);
    }
  }
}

void Calib_High()
{
  for (IR_counter=0; IR_counter< 8; IR_counter++)
  {  
  digitalWrite(IR_Select_C, bitRead(IR_counter, 2));
  digitalWrite(IR_Select_B, bitRead(IR_counter, 1));
  digitalWrite(IR_Select_A, bitRead(IR_counter, 0));
  delay(20);
  // Read phototransistors and store data into calibration HIGH array
  IR_calib_high[0 + IR_counter] = analogRead(A0);
  delay(20);
  IR_calib_high[8 + IR_counter] = analogRead(A1);
  delay(20); 
  }
  // Print on Serial Port
  for (byte i=0; i<16; i++)
  {
    Serial.print("IR_calib_high[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.print(IR_calib_high[i]);
    Serial.print(" ;");
    Serial.println("");
  }
}

void Calib_Low()
{
  for (IR_counter=0; IR_counter< 8; IR_counter++)
  {  
  digitalWrite(IR_Select_C, bitRead(IR_counter, 2));
  digitalWrite(IR_Select_B, bitRead(IR_counter, 1));
  digitalWrite(IR_Select_A, bitRead(IR_counter, 0));
  delay(20);
  // Read phototransistors and store data into calibration LOW array
  IR_calib_low[0 + IR_counter] = analogRead(A0);
  delay(20);
  IR_calib_low[8 + IR_counter] = analogRead(A1);
  delay(20);  
  }
  // Print on Serial Port
  for (byte i=0; i<16; i++)
  {
    Serial.print("IR_calib_low[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.print(IR_calib_low[i]);
    Serial.print(" ;");
    Serial.println("");
  }
}

byte getPixelChar(uint8_t x, uint8_t y, wchar_t ch)

{
    if (x > 7) return 0; // 4 = font Width -1
    return bitRead(pgm_read_byte(&font8x8[ch-32][7-y]),7-x); // 4 = Font witdh -1  
}
byte getPixelHString(uint16_t x, uint16_t y, wchar_t *p)

{
    p=p+x/7;
    return getPixelChar(x%7,y,*p);  
}

unsigned int lenString(wchar_t *p)
{
  unsigned int retVal=0;
  while(*p!='\0')
  { 
   retVal++;
   p++;
  }
  return retVal;
}


void hScroll(uint8_t y, Color For_color, Color Bk_color, wchar_t *mystring, uint8_t delaytime, uint8_t times, uint8_t dir)
{

  int offset;
  Color color;
    while (times)
      {
      for ((dir) ? offset=0 : offset=((lenString(mystring)-8)*7-1); (dir) ? offset <((lenString(mystring)-8)*7-1): offset >0; (dir) ? offset++ : offset--)
        {
        for (byte xx=0; xx<8; xx++)
          {
            for (byte yy=0; yy<8; yy++)
              {
                if (getPixelHString(xx+offset,yy,mystring)) 
                  {
                  color = For_color;
                  }
                else 
                {
                  color = Bk_color;
                }
                  LED(xx,(yy+y),color.red, color.green, color.blue);
              }          
            }
      delay(delaytime);  
        }
      times--;
      }
 }
