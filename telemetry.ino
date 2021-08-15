#include "ADXL372.h"
#include "SdFat.h"

//#define TESTING 1
#ifdef TESTING
#define SERIAL_FORMAT SERIAL_8N1
#else
#define SERIAL_FORMAT SERIAL_8N1_RXINV_TXINV
#endif
#define SERIAL_BUFFER_SIZE 256

char serial8_buf[SERIAL_BUFFER_SIZE];
int serial8_buf_size;

#define ADXL372_CS 10
ADXL372 adxl = ADXL372(ADXL372_CS);
struct ADXL372_AccelTriplet accel;
struct ADXL372_AccelTripletG accelG;
char adxl_present = 0;

#define TIMER_PIN 12

#define SD_CONFIG SdioConfig(FIFO_SDIO)
SdExFat sd;
ExFile logfile;

char interactive_usage[] = "Usage: [l]ist, [p]rint, [w]ipe, e[x]it:\n";

struct vescData {
  char vescnum;
  char done;
  unsigned int timestamp;
  int length;
  int counter;
  char data[256];
  HardwareSerial *serial;
};

struct accelData {
  char accel = 0x41;
  int time;
  int16_t x;
  int16_t y;
  int16_t z;
  char terminator[4] = {'-', '-', '-', '-'};
}  __attribute__((packed)) accelData;

char vescQueryBytes[] = {0x02, 0x01, 0x04, 0x40, 0x84, 0x03};

struct vescData vescs[4] = {{0, 0, 0, 0, 0, {0,}, &Serial2},
{1, 0, 0, 0, 0, {0,}, &Serial3},
{2, 0, 0, 0, 0, {0,}, &Serial5},
{3, 0, 0, 0, 0, {0,}, &Serial7}};

void sd_out (char *data, int len) {
  static char failed = 0;
  /* Writes will block for up to a second before determining failure, which isn't acceptable. If we see a single failed write, give up on the SD card. */
  if (failed) {
    return;
  }
  if (logfile.write(data, len) != len) {
    Serial.print("Failed to write\n");
    failed = 1;
  }
  logfile.sync();
}

void vescRead(struct vescData *vesc) {
  // Don't read if we haven't been reinitialised yet
  if (vesc->done == 1)
      return;

  // Drain the port
  while (vesc->serial->available()) {
    vesc->data[vesc->counter] = vesc->serial->read();
    vesc->counter++;
    if (vesc->counter == 2) {
      vesc->length = vesc->data[1] + 5;
    }
    /* If we've read as much data as we expected to, get it ready for transmission and dump it to SD */
    if (vesc->length == vesc->counter) {
      vesc->data[vesc->length++] = '-';
      vesc->data[vesc->length++] = '-';
      vesc->data[vesc->length++] = '-';
      vesc->data[vesc->length++] = '-';
      vesc->timestamp = millis();
      vesc->done = 1;
      sd_out((char *)&vesc->vescnum, sizeof(vesc->vescnum));
      sd_out((char *)&vesc->timestamp, sizeof(vesc->timestamp));
      sd_out(vesc->data, vesc->length);
      return;
    }
  }
}

/* Write the GetValues command to the VESC */
void vescQuery(struct vescData *vesc) {
  vesc->serial->write(vescQueryBytes, sizeof(vescQueryBytes));
}

/* Set everything back to a reasonable default */
void vescReinit(struct vescData *vesc) {
  vesc->timestamp = millis();
  vesc->counter = 0;
  vesc->done = 0;
  vescQuery(vesc);
}

void accelerometerRead() {
  adxl.ReadAccTriplet(&accel);
}

void setup() {
  char filename[8];
  // Debug serial is always the default
  Serial.begin(115200);
  // VESC serials depend on whether we need inverted signals or not
  Serial2.begin(115200, SERIAL_FORMAT);
  Serial3.begin(115200, SERIAL_FORMAT);
  Serial5.begin(115200, SERIAL_FORMAT);
  Serial7.begin(115200, SERIAL_FORMAT);
  // Transmitter is normal polarity
  Serial8.begin(115200);

  delay(5000);
  Serial.print("Starting setup\n");

  /* Increase the amount of memory available for hardware buffering in order to reduce the probability of blocking when triggering a serial write */
  Serial8.addMemoryForWrite(serial8_buf, SERIAL_BUFFER_SIZE);

  /* And keep track of what the empty buffer size is for later comparison - empty buffer indicates that writes have completed */
  serial8_buf_size = Serial8.availableForWrite();

  Serial.print("------------");

  Serial.print("Setting up timer\n");
  // Output a 10kHz tick on the timer pin for measurement correlation
  analogWriteFrequency(TIMER_PIN, 10000);
  analogWrite(TIMER_PIN, 26);

  Serial.print("Setting up SD card\n");

  if (!sd.begin(SD_CONFIG)) {
    Serial.print("Unable to setup SD card\n");
    sd.initErrorPrint(&Serial);
  } else {
    /* Find the next filename */
    for (int i = 0; i < 4096; i++) {
      sprintf(filename, "%d", i);
      if (sd.exists(filename)) {
        continue;
      }
      if (!logfile.open(filename, O_RDWR|O_CREAT)) {
        Serial.print("Unable to open SD card\n");
      }
      break;
    }
  }

  /* Kick all the VESCs so they'll start sending us values */
  for (int i = 0; i < 4; i++) {
    vescReinit(&vescs[i]);
  }
  Serial.print("Finished VESC init\n");

  /* Look for an accelerometer */
  if(adxl.begin() == ADXL372_DEVID_VAL) {
    adxl_present = 1;
    Serial.println("ADXL372 found!");
    adxl.Set_BandWidth(BW_3200Hz);
    adxl.Set_low_noise(true);
    adxl.Set_ODR(ODR_6400Hz);
    adxl.Set_hpf_corner(HPF_CORNER0);
    adxl.Set_op_mode(FULL_BW_MEASUREMENT);
  } else {
    Serial.println("Failed to find ADXL372!");
  }

  delay(100);
}

void serialEvent2() {
  vescRead(&vescs[0]);
}

void serialEvent3() {
  vescRead(&vescs[1]);
}

void serialEvent5() {
  vescRead(&vescs[2]);
}

void serialEvent7() {
  vescRead(&vescs[3]);
}

void loop() {
  /* Variables that need to persist across iterations of the main loop */
  static int currently_writing = -1;
  static char interactive = 0;
  static char printing = 0;
  static char wiping = 0;
  static char buf[256];
  static char entercount = 0;
  static int readcount = 0;
  static int entertime = 0;
  static int last_x = 0, last_y = 0, last_z = 0;

  /* Variables that are only used in this iteration of the main loop */
  int i;
  unsigned int current_time = millis();
  unsigned int waiting = 0;
  signed char longest = -1;

  /* Interactive mode. This allows for file management without having to remove the SD card */
  if (interactive) {
    if (!Serial8.available())
    {
      return;
    }
    buf[readcount++] = Serial8.read();
    if (readcount == 256) {
      readcount = 0;
      return;
    }
    if (buf[readcount-1] != '\r') {
      return;
    }

    if (printing == 1) {
      ExFile file;
      int dat;

      buf[readcount-1] = '\0';
      file.open(buf, O_READ);
      while (1) {
        dat = file.read();
        if (dat == -1) {
          break;
        }
        Serial8.write((char *)&dat, 1);
      }
      readcount = 0;
      printing = 0;
      return;
    } else if (wiping == 1) {
      buf[readcount-1] = '\0';
      sd.remove(buf);
      readcount = 0;
      wiping = 0;
      return;
    } else {
      if (readcount != 2) {
        readcount = 0;
        return;
      }
      switch (buf[0]) {
        case 'l':
          sd.ls(&Serial8, "/", 0);
          break;
        case 'p':
          printing = 1;
          break;
        case 'w':
          wiping = 1;
          break;
        case 'x':
          interactive = 0;
          break;
        default:
          Serial8.print(interactive_usage);
      }
      readcount = 0;
    }
    return;
  }

  /* Re-kick any Vescs that haven't produced an update in over a second */
  for (i=0; i<4; i++) {
    if ((current_time - vescs[i].timestamp) > 1000) {
      vescReinit(&vescs[i]);
      if (i == currently_writing) {
        currently_writing = -1;
      }
    }
  }

  /* Avoid simultaneous queuing of output - it makes it harder to know when a writeout has completed */
  if (currently_writing == -1) {
    /* Find the output that's been waiting the longest */
    for (i=0; i<4; i++) {
      /* Ignore anything that doesn't have a full buffer */
      if (vescs[i].done == 0) {
        continue;
      }
      if ((current_time - vescs[i].timestamp) > waiting) {
        longest = i;
        waiting = current_time - vescs[i].timestamp;
      }
    }

    if (longest != -1) {
      /* Make sure there's nothing still waiting in the output queue */
      if (Serial8.availableForWrite() == serial8_buf_size) {
        /* There isn't, so let's go. Let the recipient know which vesc the data is coming from */
        Serial8.write((char *)&longest, (sizeof(longest)));
        /* And when it happened */
        Serial8.write((char *)&vescs[longest].timestamp, sizeof(vescs[longest].timestamp));
        Serial8.write(vescs[longest].data, vescs[longest].length);
        currently_writing = longest;
      }
    }
  } else {
    /* Check whether writeout is complete - if so, update the state and trigger a new query */
    if (Serial8.availableForWrite() == serial8_buf_size) {
      vescReinit(&vescs[currently_writing]);
      currently_writing = -1;
    }
  }

  if (adxl_present == 1) {
    accelerometerRead();

    /* The accelerometer returns values in units of 0.1g. If there's a >1g difference compared to previous values, send and update what we last sent */
    if (abs(accel.x - last_x) > 10 || abs(accel.y - last_y) > 10 || abs(accel.z > last_z) > 10) {
      last_x = accelData.x = accel.x;
      last_y = accelData.y = accel.y;
      last_z = accelData.z = accel.z;
      accelData.time = millis();

      /* With luck this doesn't happen too often, and it's small, so risk blocking */
      Serial8.write((char *)&accelData, sizeof(accelData));
      sd_out((char *)&accelData, sizeof(accelData));
    }
  }

  /* If we get three enters in a row on the telemetry radio, with nothing in-between, in under 2 seconds, switch to interactive mode */
  if (Serial8.available()) {
    if (Serial8.read() == '\r') {
      entercount++;
      if (entertime == 0) {
        entertime = millis();
      }
      if (entercount == 3) {
        interactive = 1;
        Serial8.print(interactive_usage);
      }
    } else {
      /* We received another character, so reset the counters */
      entercount = 0;
      entertime = 0;
    }
  }

  /* More than two seconds have passed since the first enter we saw, so assume it was spurious and clear state */
  if (entertime != 0 && ((millis() - entertime) > 2000)) {
    entercount = 0;
    entertime = 0;
  }
}
