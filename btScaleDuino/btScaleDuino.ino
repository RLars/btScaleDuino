#include <HX711.h>
#include <EEPROM.h>
#include <Filters.h>
#include <SoftwareSerial.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <SPI.h>

SoftwareSerial BTserial(2, 3);

#define DEBUG 1
#define VERSION_STRING "0.01"
#define FLIP_DISPLAY 0
// Color definitions
#define  BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

// You can use any (4 or) 5 pins 
#define sclk 13
#define mosi 11
#define cs   10
#define rst  9
#define dc   8

Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, rst);

// Scale Settings
const int SCALE_DOUT_PIN = A1;
const int SCALE_SCK_PIN = A0;

const unsigned long updateRateMs = 200;
const unsigned long updateRateMsDisp = 500;
const unsigned int maxNumSamples = 50;

float meanSum;
unsigned int meanNumSamples;
unsigned long lastTimeUpdate;
unsigned long lastTimeUpdateDisp;

// Low pass filter
// filters out changes faster that 5 Hz.
float filterFrequency = 0.5f;

// create a one pole (RC) lowpass filter
FilterOnePole lowpassFilter( LOWPASS, filterFrequency );   

// Calibration
typedef struct {
  float value;
  uint32_t valid;
} calibData;

const uint32_t validMagicVal = 0xBADC0DE;

const int eeAddressCal = 0;
calibData cal;

const float calibDefault = -294.0f;

// BTserial comm
const char EOS_CHAR = '_';
const char CMD_TARE_CHAR = 't';
const char CMD_STORE_CHAR = 's';
const char CMD_CALIB_CHAR = 'c';
const char CMD_INFO_CHAR = 'i';
const char CMD_VALUE_CHAR = 'v';

// Scale object
HX711 scale(SCALE_DOUT_PIN, SCALE_SCK_PIN);

// Function prototypes
void setCalib(float val);
void loadCalib(void);
void storeCalib(void);

float calcMean(void);
void addMeanSample(float val);

void receiveCommands(void);

void setupBluetoothModule(void);
void setupDisplay(void);

void printBluetoothLogo(bool connected);
void printStatus(float weight, bool connected);

// Functions
void setup() {
  Serial.begin(9600);
  Serial.println("btScaleDuino v" VERSION_STRING);

  setupDisplay();

  display.fillScreen(GREEN);
  display.print("Initializing...");
  
  setupBluetoothModule();

  loadCalib();

  scale.tare();

  meanSum = 0.0f;
  meanNumSamples = 0;
  lastTimeUpdate = millis();

  printInfo();

  display.fillScreen(BLACK);
}

float lastValidMeasurement = 0.0f;

void loop() {
  unsigned long currentMs = millis();
  if ( ( currentMs - lastTimeUpdate ) > updateRateMs )
  {
    //float meanVal = calcMean();
    float meanVal = lowpassFilter.output();
    BTserial.print(CMD_VALUE_CHAR);
    BTserial.print(String(meanVal, 2));
    BTserial.print(EOS_CHAR);

    lastValidMeasurement = meanVal;
    lastTimeUpdate = currentMs;
  }

  if ( ( currentMs - lastTimeUpdateDisp ) > updateRateMsDisp )
  {
    printStatus(lastValidMeasurement, true);
    lastTimeUpdateDisp = currentMs;
  }

  //addMeanSample(scale.get_units(1));
  lowpassFilter.input(scale.get_units(1));

  receiveCommands();
}

void setupDisplay(void) {
  display.begin();
#if FLIP_DISPLAY
  display.setRotation(2);
#endif
  display.fillScreen(BLACK);
}

void printBluetoothLogo(bool connected)
{
  uint16_t col;
  if(connected)
  {
    col = GREEN;
  }
  else
  {
    col = RED;
  }
  display.drawLine(9, 1, 9, 18, col);
  display.drawLine(5, 5, 14, 13, col);
  display.drawLine(5, 14, 14, 5, col);
  display.drawLine(9, 1, 14, 5, col);
  display.drawLine(9, 18, 14, 13, col);
}

void printStatus(float weight, bool connected)
{
  uint16_t preComma, postComma;
  preComma = (uint16_t) weight;
  postComma = (uint16_t)(((uint32_t) (weight*100.0f))%100);
  
  //display.fillScreen(BLACK);
  display.setTextColor(WHITE, BLACK);
  char weightStr[10];
  display.setCursor(0, 30);
  display.setTextSize(2);
  /* TODO: Print leading space in case of positive number. */
  sprintf(weightStr, "%4d.%02dg", preComma, postComma);
  display.print(weightStr);

  printBluetoothLogo(connected);
}

void setupBluetoothModule(void) {
  BTserial.begin(9600);
  //BTserial.println("AT+BAUD8");
  //BTserial.begin(115200);
  BTserial.println("AT+NAME=BtScale=");
  String reply = BTserial.readString();
  // delay(1500);
  // BTserial.println("AT+PIN0000");
  // reply = BTserial.readString();
  // delay(1500);
}

typedef enum {
  REC_STATE_IDLE,
  REC_STATE_COLLECTING,
} receiveStates;

receiveStates state;
char buf[10];
unsigned char bufIdx;

void receiveCommands(void) {
  // check for new character on console
  if (BTserial.available() > 0) {
    // read the incoming byte:
    unsigned char incomingByte = BTserial.read();

    if (REC_STATE_IDLE == state) {
#if DEBUG
      BTserial.print("CMD received: ");
      BTserial.println(incomingByte, DEC);
#endif
      switch (incomingByte) {
        case CMD_TARE_CHAR:
          // BTserial.println("Executing tare command");
          scale.tare();
          break;
        case CMD_STORE_CHAR:
          // BTserial.println("Executing store command");
          storeCalib();
          break;
        case CMD_CALIB_CHAR:
          bufIdx = 0;
          buf[bufIdx++] = incomingByte;
          state = REC_STATE_COLLECTING;
          break;
        case CMD_INFO_CHAR:
          printInfo();
          break;
      }
    }
    else if (REC_STATE_COLLECTING == state) {
      if (incomingByte == EOS_CHAR)
      {
        if (buf[0] == CMD_CALIB_CHAR)
        {
          buf[bufIdx] = '\0';
          String myString = String(&buf[1]);
          setCalib(myString.toFloat());
          printInfo();
        }
        state = REC_STATE_IDLE;
      }
      else
      {
        buf[bufIdx++] = incomingByte;
#if DEBUG
        BTserial.print("Collecting...Next Byte: ");
        BTserial.println(incomingByte, DEC);
#endif
      }
    }
    else {
      /* Do nothing here. */
    }
  }
}

void printInfo(void) {
  BTserial.print(CMD_INFO_CHAR);
  BTserial.print(String(cal.value, 2));
  BTserial.print(EOS_CHAR);
}

float calcMean()
{
  float meanVal = 0;

  if (meanNumSamples > 0)
  {
    meanVal = meanSum / (float)meanNumSamples;
#if DEBUG
    BTserial.println(String(meanSum, 2));
    BTserial.println(meanNumSamples);
#endif
  }

  meanSum = 0.0f;
  meanNumSamples = 0;

  return meanVal;
}

void addMeanSample(float val)
{
  if (meanNumSamples < maxNumSamples)
  {
    meanSum += val;
    meanNumSamples++;

#if DEBUG
    BTserial.print(".");
    BTserial.println(String(val, 2));
#endif
  }
}

void setCalib(float val) {
   BTserial.print("Setting calibration value to: ");
   BTserial.println(String(val, 2));
  
  cal.value = val;
  cal.valid = validMagicVal;

  scale.set_scale(cal.value);
}

void loadCalib() {
#if DEBUG
  BTserial.println("Loading calibration value...");
#endif
  EEPROM.get(eeAddressCal, cal);
  if ( !( cal.valid == validMagicVal ) )
  {
#if DEBUG
    BTserial.print("No calibration found. Using default: ");
#endif
    cal.value = calibDefault;
    cal.valid = validMagicVal;
  }
  else
  {
#if DEBUG
    BTserial.print("Calibration value found: ");
#endif
  }

#if DEBUG
  BTserial.println(String(cal.value, 2));
#endif
  scale.set_scale(cal.value);
}

void storeCalib() {
  EEPROM.put(eeAddressCal, cal);
}

