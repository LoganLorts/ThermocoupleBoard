#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "configuration_constants_LTC2984.h"
#include "table_coeffs_LTC2984.h"
#include "support_functions_LTC2984.h"

#ifndef THC_ENABLE_RTC
  #define THC_ENABLE_RTC 1
#endif

#define THC_RTC_NONE    0
#define THC_RTC_DS3231  1
#define THC_RTC_PCF8523 2

#ifndef THC_RTC_TYPE
  #define THC_RTC_TYPE THC_RTC_PCF8523
#endif

#if THC_ENABLE_RTC && (THC_RTC_TYPE != THC_RTC_NONE)
  #include <Wire.h>
  #include <RTClib.h>
  #define THC_HAS_RTC 1
#else
  #define THC_HAS_RTC 0
#endif

#if defined(__has_include)
  #if __has_include(<EEPROM.h>) && !defined(ARDUINO_ARCH_SAM) && !defined(ARDUINO_ARCH_MBED)
    #include <EEPROM.h>
    #define THC_HAS_EEPROM 1
  #endif
#endif

#ifndef THC_HAS_EEPROM
  #define THC_HAS_EEPROM 0
#endif

static const uint32_t SERIAL_BAUD = 250000UL;
static const uint32_t THC_SERIAL_STARTUP_DELAY_MS = 1500UL;
static const uint32_t THC_SERIAL_READY_TIMEOUT_MS = 3000UL;

#define THC_PROGRAM_NAME       "ATMC Six Thermocouple Board - Giga LTC2984"
#define THC_VERSION_STRING     "3.0.0-giga-ltc2984"
#define THC_BUILD_DATE         __DATE__
#define THC_BUILD_TIME         __TIME__
#define THC_BUILD_STAMP        __DATE__ " " __TIME__

// SD and LTC CS pins must be different.
static const uint8_t SDCS   = 10;
static const uint8_t LTC_CS = 7;

// One MOSFET/heater output on Arduino Giga digital pin 8.
static const uint8_t ACTIVE_PWM = 8;

#define LTC_SPI SPI1
static SPISettings ltc2984SpiSettings(100000UL, MSBFIRST, SPI_MODE0);

static const uint8_t PWM_BITS = 10;
static const uint16_t PWM_MAX = 1023;
static const uint32_t MAX_LOG_FILES = 100000UL;

static const uint8_t NUM_TCS = 6;
static const uint8_t NUM_TCs = NUM_TCS;

// LTC2984 channel assignment:
// CH1 = off-chip diode cold junction sensor.
// TC1 = CH4-CH3
// TC2 = CH6-CH5
// TC3 = CH8-CH7
// TC4 = CH10-CH9
// TC5 = CH12-CH11
// TC6 = CH14-CH13
static const uint8_t CJ_DIODE_CHANNEL = 1;
static const uint8_t COLD_JUNCTION_CH = CJ_DIODE_CHANNEL;
static const uint8_t LTC_TC_CHANNELS[NUM_TCS] = {4, 6, 8, 10, 12, 14};

static const uint16_t updateIntervalsMs[] = {
  100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 4000
};

static const uint8_t NUM_INTERVAL_OPTIONS =
  sizeof(updateIntervalsMs) / sizeof(updateIntervalsMs[0]);

static const uint16_t eeAcqRateAddr = 13;
static const uint16_t eeBuildStampAddr = 20;
static const uint8_t INPUT_BUFFER_LEN = 200;

static char inputBuffer[INPUT_BUFFER_LEN];
static uint8_t inputIndex = 0;
static bool commandReady = false;
static bool inputOverflow = false;

static float temp[NUM_TCS];
static float tempC[NUM_TCS];
static float temp2[1];
static uint8_t faultByte[NUM_TCS];

static bool readTempNow = false;
static bool monitorData = true;
static bool saveData = false;
static bool createFile = false;
static bool listFilesFlag = false;
static bool stopAllFlag = false;
static bool printAcqRateFlag = false;
static bool setAcqRateFlag = false;
static bool setDutyFlag = false;

static uint8_t intervalIndex = 8;
static uint16_t samplePeriodMs = updateIntervalsMs[8];
static float samplePeriodS = 0.9f;
static uint32_t nextSampleMs = 0;

static uint32_t numDataPoints = 0;
static float elapsedLogTimeS = 0.0f;

static uint16_t requestedDuty = 0;
static uint16_t currentDuty = 0;

static bool sdReady = false;
static bool logFileOpen = false;
static File logfile;
static char currentLogFilename[13] = "";
static bool csvHeaderWritten = false;
static uint32_t nextLogFileNumber = 0;
static bool logFileScanDone = false;

#if THC_HAS_RTC
  #if THC_RTC_TYPE == THC_RTC_DS3231
    static RTC_DS3231 rtc;
  #elif THC_RTC_TYPE == THC_RTC_PCF8523
    static RTC_PCF8523 rtc;
  #endif
#endif

static bool rtcReady = false;

void spi_transfer_block(uint8_t chip_select, uint8_t *tx, uint8_t *rx, uint8_t length)
{
  LTC_SPI.beginTransaction(ltc2984SpiSettings);
  digitalWrite(chip_select, LOW);

  for (int i = length - 1; i >= 0; --i)
  {
    rx[i] = LTC_SPI.transfer(tx[i]);
  }

  digitalWrite(chip_select, HIGH);
  LTC_SPI.endTransaction();
}

static bool waitForLtcStartup(uint32_t timeoutMs = 3000UL)
{
  uint32_t t0 = millis();

  while ((millis() - t0) < timeoutMs)
  {
    uint8_t status = transfer_byte(LTC_CS, READ_FROM_RAM, COMMAND_STATUS_REGISTER, 0);

    if (status != 0x00 && status != 0xFF)
    {
      return true;
    }

    delay(250);
  }

  return false;
}

static bool waitForLtcReady(uint32_t timeoutMs = 3000UL)
{
  uint32_t t0 = millis();

  while ((millis() - t0) < timeoutMs)
  {
    uint8_t status = transfer_byte(LTC_CS, READ_FROM_RAM, COMMAND_STATUS_REGISTER, 0);

    if (status & 0x40)
    {
      return true;
    }

    delay(5);
  }

  return false;
}

static void configureLtcChannels()
{
  uint32_t data;

  data = SENSOR_TYPE__OFF_CHIP_DIODE |
         DIODE_SINGLE_ENDED |
         DIODE_NUM_READINGS__2 |
         DIODE_AVERAGING_ON |
         DIODE_CURRENT__20UA_80UA_160UA |
         ((uint32_t)0x000000 << DIODE_IDEALITY_FACTOR_LSB);

  assign_channel(LTC_CS, COLD_JUNCTION_CH, data);

  data = SENSOR_TYPE__TYPE_K_THERMOCOUPLE |
         TC_COLD_JUNCTION_CH__1 |
         TC_DIFFERENTIAL |
         TC_OPEN_CKT_DETECT__YES |
         TC_OPEN_CKT_DETECT_CURRENT__10UA;

  for (uint8_t k = 0; k < NUM_TCS; ++k)
  {
    assign_channel(LTC_CS, LTC_TC_CHANNELS[k], data);
  }
}

static void configureLtcGlobalParameters()
{
  transfer_byte(LTC_CS, WRITE_TO_RAM, 0x00F0, TEMP_UNIT__C | REJECTION__50_60_HZ);
  transfer_byte(LTC_CS, WRITE_TO_RAM, 0x00FF, 0xFF);
}

static uint16_t ltcResultAddress(uint8_t channel)
{
  return (uint16_t)(0x0010U + 4U * (channel - 1U));
}

static void read_ltc_temperature(uint8_t channel, float &temperature, uint8_t &fault)
{
  transfer_byte(LTC_CS, WRITE_TO_RAM, COMMAND_STATUS_REGISTER, 0x80 | channel);

  if (!waitForLtcReady())
  {
    fault = 0xFE;
    temperature = -999.0f;
    return;
  }

  uint16_t addr = ltcResultAddress(channel);

  uint32_t raw = 0;
  raw |= (uint32_t)transfer_byte(LTC_CS, READ_FROM_RAM, addr + 0, 0) << 24;
  raw |= (uint32_t)transfer_byte(LTC_CS, READ_FROM_RAM, addr + 1, 0) << 16;
  raw |= (uint32_t)transfer_byte(LTC_CS, READ_FROM_RAM, addr + 2, 0) << 8;
  raw |= (uint32_t)transfer_byte(LTC_CS, READ_FROM_RAM, addr + 3, 0);

  fault = (uint8_t)((raw >> 24) & 0xFF);

  int32_t signed24 = (int32_t)(raw & 0x00FFFFFFUL);

  if (signed24 & 0x00800000L)
  {
    signed24 |= 0xFF000000L;
  }

  temperature = (float)signed24 / 1024.0f;
}

void read_all_ltc_temperatures()
{
  for (uint8_t i = 0; i < NUM_TCs; i++)
  {
    read_ltc_temperature(LTC_TC_CHANNELS[i], temp[i], faultByte[i]);
    tempC[i] = temp[i];
  }

  uint8_t cjFault = 0;
  read_ltc_temperature(CJ_DIODE_CHANNEL, temp2[0], cjFault);
}

static void deselectAllSpiDevices()
{
  digitalWrite(SDCS, HIGH);
  digitalWrite(LTC_CS, HIGH);
}

static void printHelp(Stream &out)
{
  out.println(F("THC supports the following commands:"));
  out.println(F("  a -- Stop everything, save data, close the file, and wait until restart."));
  out.println(F("  h -- List supported commands."));
  out.println(F("  v -- Print program/version/status."));
  out.println(F("  d -- Print startup/runtime diagnostics."));
  out.println(F("  l -- List files on the SD card and show the next log filename."));
  out.println(F("  L -- Log data to a new CSV file."));
  out.println(F("  tt -- Tell RTC time, if an RTC is available."));
  out.println(F("  tb -- Set RTC to this sketch's compile/build date and time."));
  out.println(F("  tYYYY-MM-DD HH:MM:SS -- Set RTC."));
  out.println(F("  ss -- Print acquisition interval."));
  out.println(F("  s# -- Set acquisition interval, 0..10."));
  out.println(F("  W#### -- Set 0..1023 duty cycle and start logging, e.g. W512."));
}

static char *trimCommand(char *s)
{
  while (*s == ' ' || *s == '\t')
  {
    ++s;
  }

  char *end = s + strlen(s);

  while (end > s)
  {
    char c = *(end - 1);

    if (c == ' ' || c == '\t' || c == '\r' || c == '\n')
    {
      *(--end) = '\0';
    }
    else
    {
      break;
    }
  }

  return s;
}

static bool parseUnsignedInRange(const char *s, uint16_t minValue, uint16_t maxValue, uint16_t &value)
{
  if (!s || *s == '\0')
  {
    return false;
  }

  uint32_t accum = 0;

  while (*s)
  {
    if (*s < '0' || *s > '9')
    {
      return false;
    }

    accum = accum * 10UL + (uint32_t)(*s - '0');

    if (accum > maxValue)
    {
      return false;
    }

    ++s;
  }

  if (accum < minValue || accum > maxValue)
  {
    return false;
  }

  value = (uint16_t)accum;
  return true;
}

static void formatDateTimeParts(char *buffer, size_t len,
                                uint16_t year, uint8_t month, uint8_t day,
                                uint8_t hour, uint8_t minute, uint8_t second)
{
  snprintf(buffer, len, "%04u-%02u-%02u %02u:%02u:%02u",
           year, month, day, hour, minute, second);
}

#if THC_HAS_RTC
static void formatDateTime(char *buffer, size_t len, const DateTime &dt)
{
  formatDateTimeParts(buffer, len, dt.year(), dt.month(), dt.day(),
                      dt.hour(), dt.minute(), dt.second());
}
#endif

static const __FlashStringHelper *rtcTypeName()
{
#if !THC_HAS_RTC
  return F("none / not compiled");
#elif THC_RTC_TYPE == THC_RTC_DS3231
  return F("DS3231");
#elif THC_RTC_TYPE == THC_RTC_PCF8523
  return F("PCF8523");
#else
  return F("unknown");
#endif
}

static void printVersionInfo(Stream &out)
{
  out.print(F("Program: "));
  out.println(F(THC_PROGRAM_NAME));

  out.print(F("Version: "));
  out.println(F(THC_VERSION_STRING));

  out.print(F("Build: "));
  out.print(F(THC_BUILD_DATE));
  out.print(' ');
  out.println(F(THC_BUILD_TIME));

  out.print(F("RTC type: "));
  out.println(rtcTypeName());

  out.print(F("RTC detected: "));
  out.println(rtcReady ? F("yes") : F("no"));

  out.print(F("SD ready: "));
  out.println(sdReady ? F("yes") : F("no"));

  out.print(F("LTC CS pin: "));
  out.println(LTC_CS);

  out.print(F("SD CS pin: "));
  out.println(SDCS);

  out.print(F("Active PWM pin: "));
  out.println(ACTIVE_PWM);
}

#if THC_HAS_RTC
static void tellRtcTime(Stream &out)
{
  if (!rtcReady)
  {
    out.println(F("RTC is not available."));
    return;
  }

  DateTime now = rtc.now();
  char stamp[24];
  formatDateTime(stamp, sizeof(stamp), now);

  out.print(F("RTC time: "));
  out.println(stamp);
}

static bool setRtcToBuildTime()
{
  if (!rtcReady)
  {
    Serial.println(F("RTC is not available; build time was not written."));
    return false;
  }

  DateTime buildTime(THC_BUILD_DATE, THC_BUILD_TIME);
  rtc.adjust(buildTime);

  Serial.println(F("RTC set to build time."));
  return true;
}

static bool handleTimeCommand(const char *cmd)
{
  if (strcmp(cmd, "tt") == 0)
  {
    tellRtcTime(Serial);
    return true;
  }

  if (strcmp(cmd, "tb") == 0 || strcmp(cmd, "tB") == 0)
  {
    setRtcToBuildTime();
    return true;
  }

  if (cmd[0] != 't')
  {
    return false;
  }

  unsigned int yy, mo, dd, hh, mm, ss;
  char sep;

  if (sscanf(cmd + 1, "%u-%u-%u%c%u:%u:%u", &yy, &mo, &dd, &sep, &hh, &mm, &ss) == 7)
  {
    rtc.adjust(DateTime(yy, mo, dd, hh, mm, ss));
    Serial.println(F("RTC set."));
  }
  else
  {
    Serial.println(F("Invalid time command. Use tt, tb, or tYYYY-MM-DD HH:MM:SS."));
  }

  return true;
}

static void initRtc()
{
  Wire1.begin();
  delay(100);
  if (!rtc.begin(&Wire1))
  {
    rtcReady = false;
    Serial.println(F("RTC not detected."));
    return;
  }

  rtcReady = true;

  Serial.print(F("RTC detected: "));
  Serial.println(rtcTypeName());

  if (rtc.lostPower())
  {
    setRtcToBuildTime();
  }

  tellRtcTime(Serial);
}
#else
static void tellRtcTime(Stream &out)
{
  out.println(F("RTC support is not compiled."));
}

static bool handleTimeCommand(const char *cmd)
{
  if (cmd[0] == 't')
  {
    tellRtcTime(Serial);
    return true;
  }

  return false;
}

static void initRtc()
{
  rtcReady = false;
  Serial.println(F("RTC support is not compiled."));
}
#endif

static void initPersistentStorage()
{
}

static bool loadIntervalIndex(uint8_t &idx)
{
  (void)idx;
  return false;
}

static void saveIntervalIndex(uint8_t idx)
{
  (void)idx;
}

static void initHeaters()
{
#if defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_MBED) || defined(ARDUINO_ARCH_SAMD) || defined(TEENSYDUINO)
  analogWriteResolution(PWM_BITS);
#endif

  pinMode(ACTIVE_PWM, OUTPUT);
  analogWrite(ACTIVE_PWM, 0);
}

static void writeHeaterDuty(uint8_t pin, uint16_t duty)
{
  if (duty > PWM_MAX)
  {
    duty = PWM_MAX;
  }

#if defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_MBED) || defined(ARDUINO_ARCH_SAMD) || defined(TEENSYDUINO)
  analogWrite(pin, duty);
#else
  analogWrite(pin, (uint8_t)((uint32_t)duty * 255UL / PWM_MAX));
#endif
}

static void allHeatersOff()
{
  writeHeaterDuty(ACTIVE_PWM, 0);
  currentDuty = 0;
}

static const char *baseNameOnly(const char *name)
{
  const char *slash1 = strrchr(name, '/');
  const char *slash2 = strrchr(name, '\\');
  const char *base = name ? name : "";

  if (slash1 && slash1 + 1 > base)
  {
    base = slash1 + 1;
  }

  if (slash2 && slash2 + 1 > base)
  {
    base = slash2 + 1;
  }

  return base;
}

static bool parseLogFilenameIndex(const char *name, uint32_t &indexOut)
{
  const char *base = baseNameOnly(name);

  if (strlen(base) != 12)
  {
    return false;
  }

  if (toupper(base[0]) != 'L' || toupper(base[1]) != 'O' || toupper(base[2]) != 'G')
  {
    return false;
  }

  uint32_t v = 0;

  for (uint8_t i = 3; i < 8; ++i)
  {
    if (!isdigit(base[i]))
    {
      return false;
    }

    v = v * 10UL + base[i] - '0';
  }

  if (base[8] != '.' || toupper(base[9]) != 'C' || toupper(base[10]) != 'S' || toupper(base[11]) != 'V')
  {
    return false;
  }

  indexOut = v;
  return true;
}

static bool scanLogFileNumbers()
{
  nextLogFileNumber = 0;
  logFileScanDone = false;

  if (!sdReady)
  {
    return false;
  }

  File root = SD.open("/");

  if (!root)
  {
    return false;
  }

  uint32_t highest = 0;
  bool found = false;

  while (true)
  {
    File entry = root.openNextFile();

    if (!entry)
    {
      break;
    }

    if (!entry.isDirectory())
    {
      uint32_t idx;

      if (parseLogFilenameIndex(entry.name(), idx))
      {
        if (!found || idx > highest)
        {
          highest = idx;
        }

        found = true;
      }
    }

    entry.close();
  }

  root.close();

  nextLogFileNumber = found ? highest + 1UL : 0;
  logFileScanDone = true;

  return true;
}

static void printNextLogFilename(Stream &out)
{
  if (!sdReady)
  {
    out.println(F("Next log filename unknown: SD card is not ready."));
    return;
  }

  if (!logFileScanDone)
  {
    scanLogFileNumbers();
  }

  char filename[13];
  snprintf(filename, sizeof(filename), "LOG%05lu.CSV", (unsigned long)nextLogFileNumber);

  out.print(F("Next log filename: "));
  out.println(filename);
}

static bool initSdCard()
{
  Serial.print(F("Initializing SD card on CS pin "));
  Serial.print(SDCS);
  Serial.print(F("... "));

  pinMode(SDCS, OUTPUT);
  digitalWrite(SDCS, HIGH);

  if (!SD.begin(SDCS))
  {
    Serial.println(F("FAILED."));
    Serial.println(F("SD diagnostics: check card, formatting, wiring, chip-select pin, and SPI bus."));
    return false;
  }

  Serial.println(F("ready."));

  scanLogFileNumbers();
  return true;
}

static bool writeCsvHeader()
{
  if (!logfile)
  {
    return false;
  }

  logfile.print(F("# Program,"));
  logfile.println(F(THC_PROGRAM_NAME));

  logfile.print(F("# Version,"));
  logfile.println(F(THC_VERSION_STRING));

  logfile.print(F("# Build date/time,"));
  logfile.print(F(THC_BUILD_DATE));
  logfile.print(' ');
  logfile.println(F(THC_BUILD_TIME));

  logfile.print(F("# Columns,"));
  logfile.println(F("elapsed_time_s,rtc_time,TC1_C,TC2_C,TC3_C,TC4_C,TC5_C,TC6_C,F1,F2,F3,F4,F5,F6,power_command"));

  logfile.print(F("elapsed_time_s,rtc_time"));

  for (uint8_t k = 0; k < NUM_TCS; ++k)
  {
    logfile.print(F(",TC"));
    logfile.print(k + 1);
    logfile.print(F("_C"));
  }

  for (uint8_t k = 0; k < NUM_TCS; ++k)
  {
    logfile.print(F(",F"));
    logfile.print(k + 1);
  }

  logfile.println(F(",power_command"));
  logfile.flush();

  csvHeaderWritten = true;
  return true;
}

static bool openNewLogFile()
{
  if (!sdReady)
  {
    Serial.println(F("Cannot open log file: SD card is not ready."));
    return false;
  }

  if (logFileOpen)
  {
    return true;
  }

  if (!logFileScanDone)
  {
    scanLogFileNumbers();
  }

  for (uint32_t n = nextLogFileNumber; n < MAX_LOG_FILES; ++n)
  {
    char filename[13];
    snprintf(filename, sizeof(filename), "LOG%05lu.CSV", (unsigned long)n);

    if (!SD.exists(filename))
    {
      logfile = SD.open(filename, FILE_WRITE);

      if (!logfile)
      {
        return false;
      }

      strncpy(currentLogFilename, filename, sizeof(currentLogFilename));
      currentLogFilename[sizeof(currentLogFilename) - 1] = '\0';

      logFileOpen = true;
      nextLogFileNumber = n + 1UL;

      Serial.print(F("Logging to "));
      Serial.println(currentLogFilename);

      return writeCsvHeader();
    }
  }

  return false;
}

static void writeDataToSD()
{
  if (!logFileOpen)
  {
    return;
  }

  if (!csvHeaderWritten && !writeCsvHeader())
  {
    return;
  }

  logfile.print(elapsedLogTimeS, 3);
  logfile.print(',');

#if THC_HAS_RTC
  if (rtcReady)
  {
    DateTime now = rtc.now();
    char stamp[24];
    formatDateTime(stamp, sizeof(stamp), now);
    logfile.print(stamp);
  }
#endif

  for (uint8_t k = 0; k < NUM_TCS; ++k)
  {
    logfile.print(',');
    logfile.print(temp[k], 4);
  }

  for (uint8_t k = 0; k < NUM_TCS; ++k)
  {
    logfile.print(',');
    logfile.print(faultByte[k], HEX);
  }

  logfile.print(',');
  logfile.println(currentDuty);

  if ((numDataPoints % 10UL) == 0UL)
  {
    logfile.flush();
  }
}

static void listDirectory(File dir, uint8_t levels)
{
  while (true)
  {
    File entry = dir.openNextFile();

    if (!entry)
    {
      break;
    }

    Serial.print(entry.name());

    if (entry.isDirectory())
    {
      Serial.println(F("/"));

      if (levels > 0)
      {
        listDirectory(entry, levels - 1);
      }
    }
    else
    {
      Serial.print(F("\t"));
      Serial.println(entry.size());
    }

    entry.close();
  }
}

static void listSdFiles()
{
  if (!sdReady)
  {
    Serial.println(F("SD card is not ready."));
    return;
  }

  File root = SD.open("/");

  if (!root)
  {
    Serial.println(F("Could not open SD root directory."));
    return;
  }

  Serial.println(F("Files found on the card:"));
  listDirectory(root, 1);
  root.close();

  scanLogFileNumbers();
  printNextLogFilename(Serial);
}

static void handleSerialInput()
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n')
    {
      Serial.println();

      if (inputOverflow)
      {
        Serial.println(F("Input command too long; discarded."));
        inputOverflow = false;
        inputIndex = 0;
        commandReady = false;
      }
      else if (inputIndex > 0)
      {
        inputBuffer[inputIndex] = '\0';
        commandReady = true;
      }

      return;
    }

    if (inputIndex < INPUT_BUFFER_LEN - 1)
    {
      inputBuffer[inputIndex++] = c;
      Serial.print(c);
    }
    else
    {
      inputOverflow = true;
    }
  }
}

static void scheduleNextSampleFromNow()
{
  nextSampleMs = millis() + samplePeriodMs;
}

static void applyAcquisitionInterval(uint8_t newIndex, bool persist)
{
  intervalIndex = newIndex;
  samplePeriodMs = updateIntervalsMs[intervalIndex];
  samplePeriodS = (float)samplePeriodMs / 1000.0f;

  scheduleNextSampleFromNow();

  if (persist)
  {
    saveIntervalIndex(intervalIndex);
  }
}

static void printAcquisitionInterval(Stream &out)
{
  out.print(F("Acquisition interval index = "));
  out.print(intervalIndex);
  out.print(F(", period = "));
  out.print(samplePeriodMs);
  out.println(F(" ms"));
}

static void startLogging()
{
  Serial.println(F("Logging requested."));

  if (!logFileOpen)
  {
    createFile = true;
  }

  saveData = true;
  numDataPoints = 0;
  elapsedLogTimeS = 0.0f;
  readTempNow = true;
}

static void parseSerialInput()
{
  char *cmd = trimCommand(inputBuffer);

  if (*cmd == '\0')
  {
    return;
  }

  if (strcmp(cmd, "h") == 0)
  {
    printHelp(Serial);
    return;
  }

  if (strcmp(cmd, "v") == 0)
  {
    printVersionInfo(Serial);
    return;
  }

  if (strcmp(cmd, "d") == 0)
  {
    printVersionInfo(Serial);
    printAcquisitionInterval(Serial);
    return;
  }

  if (cmd[0] == 't')
  {
    handleTimeCommand(cmd);
    return;
  }

  if (strcmp(cmd, "a") == 0)
  {
    stopAllFlag = true;
    return;
  }

  if (strcmp(cmd, "l") == 0)
  {
    listFilesFlag = true;
    return;
  }

  if (strcmp(cmd, "L") == 0)
  {
    startLogging();
    return;
  }

  if (strcmp(cmd, "ss") == 0)
  {
    printAcqRateFlag = true;
    return;
  }

  if (cmd[0] == 's')
  {
    uint16_t idx;

    if (parseUnsignedInRange(cmd + 1, 0, NUM_INTERVAL_OPTIONS - 1, idx))
    {
      intervalIndex = (uint8_t)idx;
      setAcqRateFlag = true;
    }
    else
    {
      Serial.println(F("Invalid acquisition interval. Use s0 through s10, or ss."));
    }

    return;
  }

  if (cmd[0] == 'W')
  {
    uint16_t duty;

    if (parseUnsignedInRange(cmd + 1, 0, PWM_MAX, duty))
    {
      requestedDuty = duty;
      setDutyFlag = true;
      startLogging();
    }
    else
    {
      Serial.println(F("Invalid duty cycle. Use W0 through W1023."));
    }

    return;
  }

  Serial.print(F("Unknown command: "));
  Serial.println(cmd);
}

static void clearInputBuffer()
{
  memset(inputBuffer, 0, sizeof(inputBuffer));
  inputIndex = 0;
  commandReady = false;
  inputOverflow = false;
}

static void print_ltc_line()
{
  Serial.print(elapsedLogTimeS, 3);

  for (uint8_t i = 0; i < NUM_TCs; i++)
  {
    Serial.print('\t');
    Serial.print(temp[i], 4);
  }

  for (uint8_t i = 0; i < NUM_TCs; i++)
  {
    Serial.print('\t');
    Serial.print(faultByte[i], HEX);
  }

  Serial.print('\t');
  Serial.println(currentDuty);
}

static void sampleThermocouples()
{
  
  read_all_ltc_temperatures();

  if (monitorData)
  {
    print_ltc_line();
  }

  ++numDataPoints;

  if (saveData)
  {
    writeDataToSD();
  }

  elapsedLogTimeS += samplePeriodS;
}

static void stopAllAndHalt()
{
  allHeatersOff();
  deselectAllSpiDevices();

  if (logFileOpen)
  {
    logfile.flush();
    logfile.close();
    logFileOpen = false;
  }

  Serial.println(F("Stopped. Heater is off and log file is closed. Reset the board to restart."));

  while (true)
  {
    delay(1000);
  }
}

static void clearPendingSerialInput()
{
  while (Serial.available() > 0)
  {
    (void)Serial.read();
  }
}

void setup()
{
  Serial.begin(SERIAL_BAUD);

  uint32_t serialStart = millis();

  while (!Serial && (millis() - serialStart < THC_SERIAL_READY_TIMEOUT_MS))
  {
    delay(10);
  }

  delay(THC_SERIAL_STARTUP_DELAY_MS);
  clearPendingSerialInput();

  Serial.println();
  Serial.println(F("============================================================"));
  Serial.print(F(THC_PROGRAM_NAME));
  Serial.print(F(" v"));
  Serial.println(F(THC_VERSION_STRING));
  Serial.print(F("Build: "));
  Serial.print(F(THC_BUILD_DATE));
  Serial.print(' ');
  Serial.println(F(THC_BUILD_TIME));

  initPersistentStorage();
  initRtc();

  uint8_t storedInterval = intervalIndex;

  if (loadIntervalIndex(storedInterval))
  {
    intervalIndex = storedInterval;
  }

  applyAcquisitionInterval(intervalIndex, false);

  initHeaters();
  allHeatersOff();

  pinMode(SDCS, OUTPUT);
  pinMode(LTC_CS, OUTPUT);
  deselectAllSpiDevices();

  LTC_SPI.begin();
  delay(300);

  Serial.println(F("Initializing LTC2984..."));

  if (!waitForLtcStartup())
  {
    Serial.println(F("ERROR: LTC2984 not responding over SPI. Check power, SPI wiring, and CS pin."));
    while (true)
    {
      delay(1000);
    }
  }

  configureLtcChannels();
  configureLtcGlobalParameters();

  Serial.println(F("LTC2984 configured."));

  sdReady = initSdCard();

  printAcquisitionInterval(Serial);
  printVersionInfo(Serial);

  Serial.println(F("READY"));
  Serial.println(F("Type h for help."));
}

void loop()
{
  handleSerialInput();

  if (commandReady)
  {
    parseSerialInput();
    clearInputBuffer();
  }

  if (printAcqRateFlag)
  {
    printAcquisitionInterval(Serial);
    printAcqRateFlag = false;
  }

  if (setAcqRateFlag)
  {
    applyAcquisitionInterval(intervalIndex, true);
    printAcquisitionInterval(Serial);
    setAcqRateFlag = false;
  }

  if (listFilesFlag)
  {
    listSdFiles();
    listFilesFlag = false;
  }

  if (createFile)
  {
    if (!openNewLogFile())
    {
      Serial.println(F("Logging requested, but no log file could be opened. Heater will be turned off."));
      stopAllFlag = true;
    }

    createFile = false;
  }

  if (setDutyFlag)
  {
    currentDuty = requestedDuty;
    writeHeaterDuty(ACTIVE_PWM, currentDuty);

    Serial.print(F("Duty cycle = "));
    Serial.println(currentDuty);

    setDutyFlag = false;
  } 

  uint32_t now = millis();

  if ((int32_t)(now - nextSampleMs) >= 0)
  {
    readTempNow = true;
    nextSampleMs += samplePeriodMs;

    if ((int32_t)(now - nextSampleMs) >= 0)
    {
      scheduleNextSampleFromNow();
    }
  }

  if (readTempNow)
  {
    sampleThermocouples();
    readTempNow = false;
  }

  if (stopAllFlag)
  {
    stopAllAndHalt();
  }
}