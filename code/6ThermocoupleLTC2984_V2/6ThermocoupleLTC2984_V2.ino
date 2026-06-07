#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <stdint.h>
#include <math.h>

#include "configuration_constants_LTC2984.h"
#include "table_coeffs_LTC2984.h"
#include "support_functions_LTC2984.h"

// ======================================================
// USER SETTINGS
// ======================================================

#define CHIP_SELECT 7
#define LTC_SPI SPI1

#define SDCS 10
#define HEATER_PIN 8

#define CJ_DIODE_CHANNEL 1

#define NUM_TCs 5
const uint8_t LTC_TC_CHANNELS[NUM_TCs] = {4, 6, 8, 10, 12};

#if defined(SENSOR_TYPE__TYPE_T_THERMOCOUPLE)
  #define TC_SENSOR_TYPE SENSOR_TYPE__TYPE_T_THERMOCOUPLE
#else
  #define TC_SENSOR_TYPE SENSOR_TYPE__TYPE_K_THERMOCOUPLE
#endif

#define MAXFILES 10000
#define FLUSH_INTERVAL_SAMPLES 5

// Faster SPI than previous 100 kHz
SPISettings ltc2984_spi_settings(1000000, MSBFIRST, SPI_MODE0);

// ======================================================
// SPI hook
// ======================================================

void spi_transfer_block(uint8_t chip_select, uint8_t *tx, uint8_t *rx, uint8_t length)
{
  LTC_SPI.beginTransaction(ltc2984_spi_settings);
  digitalWrite(chip_select, LOW);

  for (int i = length - 1; i >= 0; --i)
  {
    rx[i] = LTC_SPI.transfer(tx[i]);
  }

  digitalWrite(chip_select, HIGH);
  LTC_SPI.endTransaction();
}

// ======================================================
// Variables
// ======================================================

#define BS 8
#define DEL 127

double updateIntervals[] = {
  100.0, 200.0, 300.0, 400.0, 500.0,
  600.0, 700.0, 800.0, 900.0, 1000.0, 4000.0
};

volatile char inbuff[200];
volatile unsigned int InputBufferIndex = 0;
char *inbuffPtr = (char *)inbuff;
volatile bool parseCommands = false;

bool monitorData = true;
bool Help = false;
bool createFile = false;
bool saveData = false;
bool allOff = false;
bool powerOn = false;
bool ls = false;
bool printAcqRate = false;
bool setAcqRate = false;
bool readTemp = false;
bool openLoop = false;
bool setDC = false;
bool controlOn = false;
bool setGains = false;
bool setKp = false;
bool setKi = false;
bool setKd = false;
bool reportGains = false;
bool pidUpdate = false;
bool pidsineUpdate = false;
bool controlErr1 = false;

char dcStr[12] = {0};
char dcStr2[12] = {0};
char tempStr[12] = {0};
char kpStr[12] = {0};
char kiStr[12] = {0};
char kdStr[12] = {0};

File logfile;

double temp[NUM_TCs];
double temp2[1];
uint8_t faultByte[NUM_TCs];

unsigned long numDataPoints = 0;
unsigned long loggedSamplesSinceFlush = 0;
unsigned int Interval = 4;

double acqInterval = 0;
double ttime = 0;
double Ts = 0.5;

double Err[] = {0.0, 0.0, 0.0};
double derr = 0.0;
double ierr = 0.0;

double setTemp = 50.0;
double DC = 0.0;
int iDC = 0;

int freq = 100;
int amp = 0;

struct PID_Gains {
  double Kp;
  double Ki;
  double Kd;
};

PID_Gains pidGains = {200.0, 10.0, 60.0};

unsigned long lastSampleMs = 0;
bool sdReady = false;

// ======================================================
// Non-blocking LTC2984 state machine
// ======================================================

uint8_t ltcCurrentIndex = 0;
uint8_t ltcConvertingChannel = 0;
bool ltcConversionInProgress = false;
bool ltcHaveFirstFullScan = false;
bool ltcReadingCJ = false;

uint16_t ltcCompletedChannels = 0;
const uint16_t LTC_FULL_SCAN_MASK = (1 << NUM_TCs) - 1;

const char HelpText[] =
"THC Giga/LTC2984 commands:\r\n"
"  A      -- Control, acquire, and store data\r\n"
"  a      -- Stop everything and close file\r\n"
"  h      -- Help\r\n"
"  L      -- Log data only\r\n"
"  o      -- Turn heater power off, continue logging\r\n"
"  ss     -- Print acquisition rate\r\n"
"  s#     -- Set acquisition interval index 0..10\r\n"
"  T#     -- Set target boundary temperature, turn PID on, and log\r\n"
"  q#     -- Set target boundary temperature only\r\n"
"  gg     -- Report PID gains\r\n"
"  gp#i#d#-- Set PID gains\r\n"
"  z      -- Toggle serial data monitor\r\n"
"  C####  -- Set manual duty cycle 0..1023 and log data\r\n"
"  W####  -- Set manual duty cycle 0..1023 and log data\r\n"
"  Q####  -- Sinusoidal input amplitude 0..1023 and log data\r\n"
"  P###   -- Sinusoidal period in seconds\r\n"
"  l      -- Basic SD status\r\n";

// ======================================================
// Serial input
// ======================================================

void serialEvent()
{
  while (Serial.available())
  {
    char InChar = Serial.read();

    if ((InChar == DEL || InChar == BS) && (InputBufferIndex != 0))
    {
      inbuff[--InputBufferIndex] = '\0';
      Serial.print((char)BS);
    }
    else if (InputBufferIndex < sizeof(inbuff) - 1)
    {
      inbuff[InputBufferIndex++] = InChar;
      inbuff[InputBufferIndex] = '\0';
    }

    if (InChar == '\n' || InChar == '\r')
    {
      parseCommands = true;
      InputBufferIndex = 0;
      break;
    }
  }
}

// ======================================================
// LTC2984 functions
// ======================================================

bool wait_for_ltc_ready(uint32_t timeout_ms = 3000)
{
  uint32_t t0 = millis();

  while ((millis() - t0) < timeout_ms)
  {
    uint8_t status = transfer_byte(CHIP_SELECT, READ_FROM_RAM, COMMAND_STATUS_REGISTER, 0);

    if ((status & 0x40) && !(status & 0x80))
    {
      return true;
    }

    delay(5);
  }

  return false;
}

bool ltc_ready_now()
{
  uint8_t status = transfer_byte(CHIP_SELECT, READ_FROM_RAM, COMMAND_STATUS_REGISTER, 0);
  return ((status & 0x40) && !(status & 0x80));
}

void configure_channels()
{
  uint32_t channel_assignment_data;

  channel_assignment_data =
      SENSOR_TYPE__OFF_CHIP_DIODE |
      DIODE_SINGLE_ENDED |
      DIODE_NUM_READINGS__2 |
      DIODE_AVERAGING_ON |
      DIODE_CURRENT__20UA_80UA_160UA |
      ((uint32_t)0x000000 << DIODE_IDEALITY_FACTOR_LSB);

  assign_channel(CHIP_SELECT, CJ_DIODE_CHANNEL, channel_assignment_data);

  for (uint8_t i = 0; i < NUM_TCs; i++)
  {
    channel_assignment_data =
        TC_SENSOR_TYPE |
        TC_COLD_JUNCTION_CH__1 |
        TC_DIFFERENTIAL |
        TC_OPEN_CKT_DETECT__YES |
        TC_OPEN_CKT_DETECT_CURRENT__10UA;

    assign_channel(CHIP_SELECT, LTC_TC_CHANNELS[i], channel_assignment_data);
  }
}

void configure_global_parameters()
{
  transfer_byte(CHIP_SELECT, WRITE_TO_RAM, 0x00F0, TEMP_UNIT__C | REJECTION__50_60_HZ);
  transfer_byte(CHIP_SELECT, WRITE_TO_RAM, 0x00FF, 0xFF);
}

uint32_t read_ltc_result_raw(uint8_t channel)
{
  const uint16_t RESULT_BASE = 0x0010;
  uint16_t addr = RESULT_BASE + 4UL * (channel - 1);
  return transfer_four_bytes(CHIP_SELECT, READ_FROM_RAM, addr, 0);
}

double convert_ltc_raw_to_temp(uint32_t raw)
{
  int32_t signed_result = raw & 0x00FFFFFFUL;

  if (signed_result & 0x00800000UL)
  {
    signed_result |= 0xFF000000UL;
  }

  return (double)signed_result / 1024.0;
}

void start_ltc_conversion(uint8_t channel)
{
  transfer_byte(CHIP_SELECT, WRITE_TO_RAM, COMMAND_STATUS_REGISTER, 0x80 | channel);
  ltcConvertingChannel = channel;
  ltcConversionInProgress = true;
}

void store_finished_ltc_channel(uint8_t channel)
{
  uint32_t raw = read_ltc_result_raw(channel);
  uint8_t fault = (uint8_t)(raw >> 24);
  double temperature_c = convert_ltc_raw_to_temp(raw);

  if (channel == CJ_DIODE_CHANNEL)
  {
    temp2[0] = temperature_c;
    ltcReadingCJ = false;
    return;
  }

  for (uint8_t i = 0; i < NUM_TCs; i++)
  {
    if (LTC_TC_CHANNELS[i] == channel)
    {
      temp[i] = temperature_c;
      faultByte[i] = fault;

      ltcCompletedChannels |= (1 << i);

      if (ltcCompletedChannels == LTC_FULL_SCAN_MASK)
      {
        ltcHaveFirstFullScan = true;
      }

      break;
    }
  }
}

void service_ltc()
{
  if (!ltcConversionInProgress)
  {
    start_ltc_conversion(LTC_TC_CHANNELS[ltcCurrentIndex]);
    return;
  }

  if (!ltc_ready_now())
  {
    return;
  }

  store_finished_ltc_channel(ltcConvertingChannel);

  if (ltcReadingCJ)
  {
    start_ltc_conversion(LTC_TC_CHANNELS[ltcCurrentIndex]);
    return;
  }

  ltcCurrentIndex++;

  if (ltcCurrentIndex >= NUM_TCs)
  {
    ltcCurrentIndex = 0;

    ltcReadingCJ = true;
    start_ltc_conversion(CJ_DIODE_CHANNEL);
    return;
  }

  start_ltc_conversion(LTC_TC_CHANNELS[ltcCurrentIndex]);
}

// ======================================================
// Heater / PID / sine control
// ======================================================

void setHeater(int duty)
{
  if (duty < 0) duty = 0;
  if (duty > 1023) duty = 1023;

  iDC = duty;
  analogWrite(HEATER_PIN, iDC);
}

void stopHeater()
{
  setHeater(0);
}

void printGains(void)
{
  Serial.print("Kp = ");
  Serial.print('\t');
  Serial.print(pidGains.Kp);
  Serial.print('\t');
  Serial.print("Ki = ");
  Serial.print('\t');
  Serial.print(pidGains.Ki);
  Serial.print('\t');
  Serial.print("Kd = ");
  Serial.println(pidGains.Kd);
}

void PID_Control(void)
{
  Err[0] = setTemp - temp[0];

  derr = Err[0] - Err[1];
  ierr = ierr + Err[0];

  if (ierr >= 250.0) ierr = 250.0;
  if (ierr <= -250.0) ierr = -250.0;

  Err[1] = Err[0];

  DC = pidGains.Kp * Err[0] + pidGains.Ki * ierr * Ts + pidGains.Kd * derr / Ts;

  setHeater((int)DC);
}

void PID_sine_Control(void)
{
  if (freq <= 0) freq = 1;

  DC = amp * (-cos(2.0 * PI / freq * ttime) + 1.0) / 2.0;

  setHeater((int)DC);
}

// ======================================================
// SD logging
// ======================================================

void WriteToSD(void)
{
  if (!logfile) return;

  logfile.print(ttime, 3);

  for (uint8_t i = 0; i < NUM_TCs; i++)
  {
    logfile.print(',');
    logfile.print(temp[i], 4);
  }

  logfile.print(',');
  logfile.print(temp2[0], 4);

  logfile.print(',');
  logfile.print(iDC);

  for (uint8_t i = 0; i < NUM_TCs; i++)
  {
    logfile.print(',');
    logfile.print(faultByte[i], HEX);
  }

  logfile.println();

  loggedSamplesSinceFlush++;

  if (loggedSamplesSinceFlush >= FLUSH_INTERVAL_SAMPLES)
  {
    logfile.flush();
    loggedSamplesSinceFlush = 0;

    Serial.print("SD flush complete at sample ");
    Serial.println(numDataPoints);
  }
}

void createLogFile()
{
  if (!sdReady)
  {
    Serial.println("SD is not ready. Logging not started.");
    saveData = false;
    createFile = false;
    return;
  }

  char filename[13];
  bool opened = false;

  for (uint16_t n = 0; n < MAXFILES; ++n)
  {
    snprintf(filename, sizeof(filename), "LOG%04u.CSV", n);

    if (!SD.exists(filename))
    {
      logfile = SD.open(filename, FILE_WRITE);

      if (logfile)
      {
        logfile.print("time_s");

        for (uint8_t i = 0; i < NUM_TCs; i++)
        {
          logfile.print(",TC_CH");
          logfile.print(LTC_TC_CHANNELS[i]);
          logfile.print("_C");
        }

        logfile.print(",CJ_CH");
        logfile.print(CJ_DIODE_CHANNEL);
        logfile.print("_C");

        logfile.print(",duty");

        for (uint8_t i = 0; i < NUM_TCs; i++)
        {
          logfile.print(",TC_CH");
          logfile.print(LTC_TC_CHANNELS[i]);
          logfile.print("_fault_hex");
        }

        logfile.println();
        logfile.flush();

        Serial.println("SD header written and flushed.");

        loggedSamplesSinceFlush = 0;
        opened = true;
      }

      break;
    }
  }

  if (!opened)
  {
    Serial.println("Could not create log file.");
    stopHeater();
    saveData = false;
  }
  else
  {
    Serial.print("Logging to ");
    Serial.println(filename);
  }
}

void stopAll()
{
  stopHeater();

  if (logfile)
  {
    logfile.flush();
    Serial.println("Final SD flush complete.");
    logfile.close();
  }

  saveData = false;
  controlOn = false;
  openLoop = false;
  powerOn = false;
  allOff = false;

  Serial.println("Stopped. File closed.");
}

// ======================================================
// Parser
// ======================================================

void parseSerialInput(void)
{
  char dataStr[4] = {0};

  bool kpSetLocal = false;
  bool kiSetLocal = false;
  bool kdSetLocal = false;

  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t k = 0;

  inbuffPtr = (char *)inbuff;

  if (*inbuffPtr == 'h')
  {
    Help = true;
    return;
  }

  if (*inbuffPtr == 'z')
  {
    monitorData = !monitorData;
    return;
  }

  if (*inbuffPtr == 'o')
  {
    powerOn = false;
    stopHeater();
    return;
  }

  if (*inbuffPtr == 'A')
  {
    saveData = true;
    powerOn = true;
    pidUpdate = true;
    controlOn = true;
    readTemp = true;
    createFile = true;
    openLoop = false;
    numDataPoints = 0;
    ttime = 0.0;

    if (Interval > 9) controlErr1 = true;

    return;
  }

  if (*inbuffPtr == 'a' || *inbuffPtr == 'c')
  {
    saveData = false;
    allOff = true;
    return;
  }

  if (*inbuffPtr == 's')
  {
    inbuffPtr++;

    if (*inbuffPtr == 's')
    {
      printAcqRate = true;
      return;
    }

    setAcqRate = true;

    while (*inbuffPtr != '\0')
    {
      if ((*inbuffPtr >= '0') && (*inbuffPtr <= '9') && i < sizeof(dataStr) - 1)
      {
        dataStr[i++] = *inbuffPtr;
      }

      inbuffPtr++;
    }

    Interval = (unsigned int)atoi(dataStr);

    if (Interval > 10) Interval = 10;
    if (controlOn && (Interval > 9)) controlErr1 = true;

    return;
  }

  if (*inbuffPtr == 'q' || *inbuffPtr == 'T')
  {
    bool startControl = (*inbuffPtr == 'T');

    while (*inbuffPtr != '\0')
    {
      if (((*inbuffPtr >= '0') && (*inbuffPtr <= '9')) || (*inbuffPtr == '.') || (*inbuffPtr == '-'))
      {
        if (i < sizeof(tempStr) - 1)
        {
          tempStr[i++] = *inbuffPtr;
        }
      }

      inbuffPtr++;
    }

    setTemp = atof(tempStr);
    memset(tempStr, 0, sizeof(tempStr));

    if (startControl)
    {
      saveData = true;
      powerOn = true;
      pidUpdate = true;
      controlOn = true;
      readTemp = true;
      createFile = true;
      openLoop = false;
      numDataPoints = 0;
      ttime = 0.0;
    }

    return;
  }

  if (*inbuffPtr == 'C' || *inbuffPtr == 'W')
  {
    readTemp = true;
    saveData = true;
    powerOn = true;
    createFile = true;
    numDataPoints = 0;
    ttime = 0.0;

    openLoop = true;
    controlOn = false;
    setDC = true;

    inbuffPtr++;

    while (*inbuffPtr != '\0')
    {
      if (((*inbuffPtr >= '0') && (*inbuffPtr <= '9')) && i < sizeof(dcStr) - 1)
      {
        dcStr[i++] = *inbuffPtr;
      }

      inbuffPtr++;
    }

    return;
  }

  if (*inbuffPtr == 'L')
  {
    createFile = true;
    saveData = true;
    readTemp = true;
    numDataPoints = 0;
    ttime = 0.0;
    return;
  }

  if (*inbuffPtr == 'l')
  {
    ls = true;
    return;
  }

  if (*inbuffPtr == 'g')
  {
    inbuffPtr++;

    if (*inbuffPtr == 'g')
    {
      reportGains = true;
      return;
    }

    setGains = true;

    while (*inbuffPtr != '\0')
    {
      switch (*inbuffPtr)
      {
        case 'p':
          setKp = true;
          kpSetLocal = true;
          kiSetLocal = false;
          kdSetLocal = false;
          break;

        case 'i':
          setKi = true;
          kpSetLocal = false;
          kiSetLocal = true;
          kdSetLocal = false;
          break;

        case 'd':
          setKd = true;
          kpSetLocal = false;
          kiSetLocal = false;
          kdSetLocal = true;
          break;

        default:
          if (((*inbuffPtr >= '0') && (*inbuffPtr <= '9')) || (*inbuffPtr == '.') || (*inbuffPtr == '-'))
          {
            if (kpSetLocal && i < sizeof(kpStr) - 1) kpStr[i++] = *inbuffPtr;
            if (kiSetLocal && j < sizeof(kiStr) - 1) kiStr[j++] = *inbuffPtr;
            if (kdSetLocal && k < sizeof(kdStr) - 1) kdStr[k++] = *inbuffPtr;
          }
          break;
      }

      inbuffPtr++;
    }

    return;
  }

  if (*inbuffPtr == 'Q')
  {
    saveData = true;
    pidsineUpdate = true;
    controlOn = true;
    readTemp = true;
    createFile = true;
    openLoop = false;
    numDataPoints = 0;
    ttime = 0.0;

    inbuffPtr++;

    while (*inbuffPtr != '\0')
    {
      if (((*inbuffPtr >= '0') && (*inbuffPtr <= '9')) && i < sizeof(dcStr) - 1)
      {
        dcStr[i++] = *inbuffPtr;
      }

      inbuffPtr++;
    }

    return;
  }

  if (*inbuffPtr == 'P')
  {
    inbuffPtr++;

    while (*inbuffPtr != '\0')
    {
      if (((*inbuffPtr >= '0') && (*inbuffPtr <= '9')) && i < sizeof(dcStr2) - 1)
      {
        dcStr2[i++] = *inbuffPtr;
      }

      inbuffPtr++;
    }

    return;
  }
}

// ======================================================
// Setup / loop
// ======================================================

void setup()
{
  Serial.begin(115200);
  delay(1500);

  analogWriteResolution(10);

  for (uint8_t i = 0; i < NUM_TCs; i++)
  {
    temp[i] = NAN;
    faultByte[i] = 0xFF;
  }

  temp2[0] = NAN;

  pinMode(HEATER_PIN, OUTPUT);
  stopHeater();

  pinMode(CHIP_SELECT, OUTPUT);
  digitalWrite(CHIP_SELECT, HIGH);

  pinMode(SDCS, OUTPUT);
  digitalWrite(SDCS, HIGH);

  LTC_SPI.begin();
  delay(300);

  Serial.println("READY!");
  Serial.println("Giga + LTC2984 harmonic boundary heating controller");

  if (!wait_for_ltc_ready())
  {
    Serial.println("ERROR: LTC2984 did not reach ready state.");
    Serial.println("Check LTC CS/SCK/CIPO/COPI/COM/power wiring.");

    while (1)
    {
      stopHeater();
      delay(1000);
    }
  }

  configure_channels();
  configure_global_parameters();

  Serial.println("LTC2984 configured.");

  service_ltc();

  Serial.println("Initializing SD card...");
  sdReady = SD.begin(SDCS);

  if (!sdReady)
  {
    Serial.println("SD card failed or not present.");
    Serial.println("Live serial readings still work, but logging commands will not save.");
  }
  else
  {
    Serial.println("SD card initialized.");
  }

  if (Interval > 10) Interval = 4;

  Ts = updateIntervals[Interval] / 1000.0;
  lastSampleMs = millis();

  Serial.println("READY");
}

void loop()
{
  serialEvent();

  service_ltc();

  if (parseCommands)
  {
    parseSerialInput();

    memset((void *)inbuff, 0, sizeof(inbuff));
    inbuffPtr = (char *)inbuff;
    parseCommands = false;

    if (controlErr1)
    {
      Serial.println("Invalid control interval.");
      stopAll();
      controlErr1 = false;
    }
  }

  if (Help)
  {
    Serial.print(HelpText);
    Help = false;
  }

  if (ls)
  {
    Serial.print("SD ready: ");
    Serial.println(sdReady ? "YES" : "NO");
    ls = false;
  }

  if (printAcqRate)
  {
    acqInterval = updateIntervals[Interval];

    Serial.print("Acquisition Interval = ");
    Serial.print(acqInterval);
    Serial.println(" ms");

    printAcqRate = false;
  }

  if (reportGains)
  {
    printGains();
    reportGains = false;
  }

  if (setGains)
  {
    if (setKp)
    {
      pidGains.Kp = atof(kpStr);
      memset(kpStr, 0, sizeof(kpStr));
      setKp = false;
    }

    if (setKi)
    {
      pidGains.Ki = atof(kiStr);
      memset(kiStr, 0, sizeof(kiStr));
      setKi = false;
    }

    if (setKd)
    {
      pidGains.Kd = atof(kdStr);
      memset(kdStr, 0, sizeof(kdStr));
      setKd = false;
    }

    Serial.println("PID gains updated.");
    setGains = false;
  }

  if (setAcqRate)
  {
    Ts = updateIntervals[Interval] / 1000.0;

    Serial.print("Interval = ");
    Serial.println(Interval);

    Serial.print("Ts = ");
    Serial.print(Ts, 3);
    Serial.println(" s");

    setAcqRate = false;
  }

  if (createFile && saveData)
  {
    createLogFile();
    createFile = false;
    readTemp = true;
  }

  if (setDC)
  {
    setHeater(atoi(dcStr));
    memset(dcStr, 0, sizeof(dcStr));
    setDC = false;
  }

  if (pidsineUpdate && controlOn)
  {
    amp = atoi(dcStr);

    if (amp < 0) amp = 0;
    if (amp > 1023) amp = 1023;

    freq = atoi(dcStr2);

    if (freq <= 0) freq = 100;

    PID_sine_Control();
    pidsineUpdate = false;
  }

  unsigned long now = millis();

  if ((now - lastSampleMs) >= (unsigned long)updateIntervals[Interval])
  {
    readTemp = true;
    lastSampleMs += (unsigned long)updateIntervals[Interval];
  }

  if (readTemp)
  {
    if (monitorData)
    {
      Serial.print(ttime, 3);

      for (uint8_t i = 0; i < NUM_TCs; i++)
      {
        Serial.print('\t');
        Serial.print(temp[i], 4);
      }

      Serial.print("\tCJ:");
      Serial.print(temp2[0], 4);

      Serial.print('\t');
      Serial.print(iDC);

      Serial.print("\tfaults:");

      for (uint8_t i = 0; i < NUM_TCs; i++)
      {
        Serial.print(" 0x");
        Serial.print(faultByte[i], HEX);
      }

      if (!ltcHaveFirstFullScan)
      {
        Serial.print("\twarming_up");
      }

      Serial.println();
    }

    numDataPoints++;
    readTemp = false;

    pidUpdate = true;
    pidsineUpdate = true;

    if (controlOn && powerOn && pidUpdate && !openLoop && ltcHaveFirstFullScan)
    {
      PID_Control();
      pidUpdate = false;
    }

    if (!powerOn && !openLoop)
    {
      stopHeater();
    }

    if (saveData)
    {
      WriteToSD();
    }

    ttime += Ts;
  }

  if (allOff)
  {
    stopAll();
  }
}
