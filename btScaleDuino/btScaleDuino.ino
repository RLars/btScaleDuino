#include <HX711.h>
#include <EEPROM.h>
#include <Filters.h>

#define DEBUG 1

// Scale Settings
const int SCALE_DOUT_PIN = A1;
const int SCALE_SCK_PIN = A0;

const unsigned long updateRateMs = 200;
const unsigned int maxNumSamples = 50;

float meanSum;
unsigned int meanNumSamples;
unsigned long lastTimeUpdate;

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

// Serial comm
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

// Functions
void setup() {
  setupBluetoothModule();

  loadCalib();

  scale.tare();

  meanSum = 0.0f;
  meanNumSamples = 0;
  lastTimeUpdate = millis();

  printInfo();
}

void loop() {
  unsigned long currentMs = millis();
  if ( ( currentMs - lastTimeUpdate ) > updateRateMs )
  {
    //float meanVal = calcMean();
    float meanVal = lowpassFilter.output();
    Serial.print(CMD_VALUE_CHAR);
    Serial.print(String(meanVal, 2));
    Serial.print(EOS_CHAR);
    lastTimeUpdate = currentMs;
  }

  //addMeanSample(scale.get_units(1));
  lowpassFilter.input(scale.get_units(1));

  receiveCommands();
}


void setupBluetoothModule(void) {
  Serial.begin(9600);
  //Serial.println("AT+BAUD8");
  //Serial.begin(115200);
  Serial.println("AT+NAME=BtScale=");
  String reply = Serial.readString();
  // delay(1500);
  // Serial.println("AT+PIN0000");
  // reply = Serial.readString();
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
  if (Serial.available() > 0) {
    // read the incoming byte:
    unsigned char incomingByte = Serial.read();

    if (REC_STATE_IDLE == state) {
#if DEBUG
      Serial.print("CMD received: ");
      Serial.println(incomingByte, DEC);
#endif
      switch (incomingByte) {
        case CMD_TARE_CHAR:
          // Serial.println("Executing tare command");
          scale.tare();
          break;
        case CMD_STORE_CHAR:
          // Serial.println("Executing store command");
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
        Serial.print("Collecting...Next Byte: ");
        Serial.println(incomingByte, DEC);
#endif
      }
    }
    else {
      /* Do nothing here. */
    }
  }
}

void printInfo(void) {
  Serial.print(CMD_INFO_CHAR);
  Serial.print(String(cal.value, 2));
  Serial.print(EOS_CHAR);
}

float calcMean()
{
  float meanVal = 0;

  if (meanNumSamples > 0)
  {
    meanVal = meanSum / (float)meanNumSamples;
#if DEBUG
    Serial.println(String(meanSum, 2));
    Serial.println(meanNumSamples);
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
    Serial.print(".");
    Serial.println(String(val, 2));
#endif
  }
}

void setCalib(float val) {
   Serial.print("Setting calibration value to: ");
   Serial.println(String(val, 2));
  
  cal.value = val;
  cal.valid = validMagicVal;

  scale.set_scale(cal.value);
}

void loadCalib() {
#if DEBUG
  Serial.println("Loading calibration value...");
#endif
  EEPROM.get(eeAddressCal, cal);
  if ( !( cal.valid == validMagicVal ) )
  {
#if DEBUG
    Serial.print("No calibration found. Using default: ");
#endif
    cal.value = calibDefault;
    cal.valid = validMagicVal;
  }
  else
  {
#if DEBUG
    Serial.print("Calibration value found: ");
#endif
  }

#if DEBUG
  Serial.println(String(cal.value, 2));
#endif
  scale.set_scale(cal.value);
}

void storeCalib() {
  EEPROM.put(eeAddressCal, cal);
}

