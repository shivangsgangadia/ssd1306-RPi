#include "OLED.h"
#include <cmath>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <wiringPi.h>

uint8_t OLED::doubleScaleBuffer[2];
int OLED::fd = -1;

void OLED::init() {
  // Load file descriptor
  OLED::fd = wiringPiI2CSetup(OLED_I2C_ADDRESS);
  if (OLED::fd < 0) {
    printf("OLED init failed.\n");
    return;
  }

  uint8_t initWriteBuffer[26] = {
      OLED_CONTROL_BYTE_CMD_STREAM,
      OLED_CMD_DISPLAY_OFF,
      OLED_CMD_SET_MUX_RATIO, 0x3F,
      OLED_CMD_SET_DISPLAY_OFFSET, 0x00,
      OLED_CMD_SET_DISPLAY_START_LINE,
      OLED_CMD_SET_SEGMENT_REMAP, 0xC0,
      OLED_CMD_SET_COM_PIN_MAP, 0x12,
      OLED_CMD_SET_CONTRAST, 0x7F,
      OLED_CMD_DISPLAY_RAM,
      OLED_CMD_DISPLAY_NORMAL,
      OLED_CMD_SET_DISPLAY_CLK_DIV, 0x80,
      OLED_CMD_SET_CHARGE_PUMP, 0x14,
      OLED_CMD_SET_PRECHARGE, 0x22,
      OLED_CMD_SET_VCOMH_DESELCT, 0x30,
      OLED_CMD_SET_MEMORY_ADDR_MODE, 0x01,
      OLED_CMD_DISPLAY_ON};
  write(OLED::fd, initWriteBuffer, 26);
  delay(1);
  // OLED::clearDisplay();
}

void OLED::resetByteBuffer() {
  for (uint8_t i = 0; i < 2; i++) {
    OLED::doubleScaleBuffer[i] = 0;
  }
}

int stringLength, charIndex;
uint8_t rowMax, columnMax, currentByte;

void OLED::writeString(char *str, int scaleFactor, int row, int column) {
  stringLength = strlen(str);
  rowMax = row + scaleFactor - 1;
  columnMax = column + (stringLength * scaleFactor * 8);

  OLED::clearDisplayAt(row, column, columnMax, scaleFactor);

  for (int i = 0; i < stringLength; i++) {
    charIndex = str[i] * 8;

    // we have 8 byte fonts
    for (int j = 0; j < 8; j++) {

      // Keep the wire interaction inside this loop, i.e. 1 transaction for 1 character otherwise display gets messed up
      currentByte = font[j + charIndex];
      scale(currentByte, scaleFactor);

      OLED::setCursor(row, rowMax, (column + (j * scaleFactor) + (i * 8 * scaleFactor)), columnMax);

      // Working with verticle addressing mode
      for (uint8_t y = 0; y < scaleFactor; y++) {
        uint8_t writeBuffer[scaleFactor + 1];
        int n = 1;
        writeBuffer[0] = OLED_CONTROL_BYTE_DATA_STREAM;

        for (int x = scaleFactor - 1; x >= 0; x--) {
          writeBuffer[n] = OLED::doubleScaleBuffer[x];
          n++;
        }

        write(OLED::fd, writeBuffer, scaleFactor + 1);

        // delay(1);
      }
    }
  }
}

void OLED::writeStringMultiLine(char *str, int scaleFactor, int row, int column) {
  int totalLength = strlen(str);
  int lineLength = ceil((128 - column) / (8 * scaleFactor));

  char oneLineBuffer[lineLength + 1];
  int bufferCounter = 0, currentRow = row, leftToWrite = totalLength;

  int currentLineLength = (leftToWrite < lineLength) ? leftToWrite : lineLength;
  for (int i = 0; i < totalLength; i++) {
    oneLineBuffer[bufferCounter] = str[i];
    bufferCounter++;
    if (bufferCounter == currentLineLength) {
      oneLineBuffer[currentLineLength] = '\0';
      OLED::writeString(oneLineBuffer, scaleFactor, currentRow, column);
      bufferCounter = 0;
      leftToWrite -= currentLineLength;
      currentLineLength = (leftToWrite < lineLength) ? leftToWrite : lineLength;
      currentRow += scaleFactor;
      if (currentRow > 7) {
        currentRow = 0;
      }
    }
  }
}

void OLED::writeDisplayByte(char *str, int scaleFactor, int row, int column) {
  stringLength = strlen(str);
  rowMax = row + scaleFactor - 1;
  columnMax = column + (stringLength * scaleFactor * 8);

  for (int i = 0; i < stringLength; i++) {
    // Keep the wire interaction inside this loop, i.e. 1 transaction for 1 character otherwise display gets messed up
    scale(str[i], scaleFactor);

    setCursor(row, rowMax, (column + (i * scaleFactor)), columnMax);

    // Working with verticle addressing mode
    for (uint8_t y = 0; y < scaleFactor; y++) {
      uint8_t writeBuffer[scaleFactor + 1];
      int n = 1;
      writeBuffer[0] = OLED_CONTROL_BYTE_DATA_STREAM;

      for (int x = scaleFactor - 1; x >= 0; x--) {
        writeBuffer[n] = OLED::doubleScaleBuffer[x];
        n++;
        // wiringPiI2CWrite(OLED::fd, OLED::doubleScaleBuffer[x]);
      }

      write(OLED::fd, writeBuffer, scaleFactor + 1);
    }
  }
}

void OLED::clearDisplay() {
  // Set the GDDRAM to (Row0, Col0), ie: top-left and establish range as the whole screen - 128x64
  uint8_t writeBuffer[7] = {
      OLED_CONTROL_BYTE_CMD_STREAM,
      OLED_CMD_SET_COLUMN_RANGE,
      0,
      127,
      OLED_CMD_SET_PAGE_RANGE,
      0,
      7};

  write(OLED::fd, writeBuffer, 7);
  // delay(1);

  uint8_t blankWriteBuffer[16 + 1];
  for (uint8_t x = 0; x < 64; x++) {
    blankWriteBuffer[0] = OLED_CONTROL_BYTE_DATA_STREAM;

    for (uint8_t i = 0; i < 16; i++) {
      blankWriteBuffer[1 + i] = 0;
    }
    write(OLED::fd, blankWriteBuffer, 16 + 1);
    // delay(1);
  }
}

void OLED::clearDisplayAt(uint8_t row, uint8_t column, uint8_t count, uint8_t scale) {
  // Set the GDDRAM to (Row0, Col0), ie: top-left and establish range as the whole screen - 128x64
  uint8_t writeBuffer[7] = {
      OLED_CONTROL_BYTE_CMD_STREAM,
      OLED_CMD_SET_COLUMN_RANGE,
      column,
      column + count * 8 * scale,
      OLED_CMD_SET_PAGE_RANGE,
      7 - row - (row + (scale * 1) - row - 1),
      7 - row};
  write(OLED::fd, writeBuffer, 7);
  // delay(1);

  uint8_t blankWriteBuffer[16 + 1];
  for (uint8_t x = 0; x < count * scale; x++) {
    blankWriteBuffer[0] = OLED_CONTROL_BYTE_DATA_STREAM;

    for (uint8_t i = 0; i < 16; i++) {
      blankWriteBuffer[1 + i] = 0;
    }
    write(OLED::fd, blankWriteBuffer, 16 + 1);
    // delay(1);
  }
}

void OLED::setCursor(int rowStart, int rowEnd, int columnStart, int columnEnd) {
  uint8_t writeBuffer[7] = {
      OLED_CONTROL_BYTE_CMD_STREAM,
      OLED_CMD_SET_COLUMN_RANGE,
      columnStart,
      columnEnd,
      OLED_CMD_SET_PAGE_RANGE,
      7 - rowStart - (rowEnd - rowStart),
      7 - rowStart};
  write(OLED::fd, writeBuffer, 7);
  // delay(1);
}

uint8_t byteToScale, bitPoint, temp;

void OLED::scale(uint8_t inp, uint8_t scale) {
  OLED::resetByteBuffer();
  byteToScale = 0, bitPoint = 0, temp = 0;

  for (int i = 0; i < 8; i++) {
    temp = inp & ANDER;
    temp = temp >> bitPoint;
    for (uint8_t j = 0; j < scale; j++) {
      OLED::doubleScaleBuffer[byteToScale] |= temp;
      temp = temp >> 1;
      bitPoint++;

      if (bitPoint > 7) {
        bitPoint = 0;
        byteToScale++;
        temp = inp & ANDER;
      }
    }
    inp = inp << 1;
  }
}
