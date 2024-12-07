#include "OLED.h"
#include "StandardFont.h"
#include <array>
#include <cstdint>
#include <stdexcept>
#include <functional>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#define BYTE_SIZE 8u


void i2cWrite(int fileDescriptor, std::vector<uint8_t> &buf) {
  struct i2c_rdwr_ioctl_data metadata;
  struct i2c_msg i2cmsg;

  metadata.msgs = &i2cmsg;
  metadata.nmsgs = 1;

  i2cmsg.addr = OLED_I2C_ADDRESS;
  i2cmsg.len = buf.size();
  i2cmsg.buf = buf.data();
  i2cmsg.flags = 0;



  int error = ioctl(fileDescriptor, I2C_RDWR, &metadata);
  if (error < 0) {
    throw std::runtime_error("I2C write failed at line " + std::to_string(__LINE__) + "; requested " + std::to_string(sizeof(buf)) + " bytes;");
  }
}


OLED::OLED(std::string deviceAddress) {
  // Load file descriptor
  m_fd = open(deviceAddress.c_str(), I2C_RDWR);
  if (m_fd < 0) {
    throw std::runtime_error("i2c init failed");
  }

  // set slave address
  if (ioctl(m_fd, I2C_SLAVE, OLED_I2C_ADDRESS) < 0) {
    close(m_fd);
    throw std::runtime_error("setting slave address failed");
  }

  auto initWriteBuffer = std::vector<uint8_t>({
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
      OLED_CMD_DISPLAY_ON});

  i2cWrite(m_fd, initWriteBuffer);
  clearDisplay();
}

OLED::~OLED() {
  close(m_fd);
}

auto OLED::initialize(std::string i2cDevAddr) -> void {
  std::function<std::string()> getAddr = [&]() {
    return i2cDevAddr;
  };

  (void) getInstance(&getAddr);
}

auto OLED::getInstance(std::function<std::string()> *addressGet) -> OLED& {
  static OLED instance {(*addressGet)()};

  return instance;
}

void OLED::resetByteBuffer() {
  std::memset(
    reinterpret_cast<wchar_t*>(m_doubleScaleBuffer),
    0,
    DOUBLE_SCALE_BUFFER_SIZE
  );
}

void OLED::writeString(std::string str, int scaleFactor, int row, int column) {
  m_stringLength = str.length();
  m_rowMax = row + scaleFactor - 1;
  m_columnMax = column + (m_stringLength * scaleFactor * BYTE_SIZE);

  clearDisplayAt(row, column, m_columnMax, scaleFactor);

  for (int i = 0; i < m_stringLength; i++) {
    m_charIndex = str[i] * BYTE_SIZE;

    // we have 8 byte fonts
    for (uint8_t j = 0; j < BYTE_SIZE; j++) {

      // Keep the wire interaction inside this loop, i.e. 1 transaction for 1 character otherwise display gets messed up
      m_currentByte = font[j + m_charIndex];
      scale(m_currentByte, scaleFactor);

      setCursor(
        row,
        m_rowMax,
        (column + (j * scaleFactor) + (i * BYTE_SIZE * scaleFactor)),
        m_columnMax
      );

      // Working with verticle addressing mode
      for (uint8_t y = 0; y < scaleFactor; y++) {
        std::vector<uint8_t> writeBuffer (scaleFactor + 1);
        int n = 1;
        writeBuffer[0] = OLED_CONTROL_BYTE_DATA_STREAM;

        for (int x = scaleFactor - 1; x >= 0; x--) {
          writeBuffer[n] = m_doubleScaleBuffer[x];
          n++;
        }

        i2cWrite(m_fd, writeBuffer);
      }
    }
  }
}

void OLED::writeStringMultiLine(std::string str, int scaleFactor, int row, int column) {
  int totalLength = str.length();
  int lineLength = ceil((SCREEN_HORIZONTAL_SIZE - column) / (BYTE_SIZE * scaleFactor));

  char oneLineBuffer[lineLength + 1];
  int bufferCounter = 0;
  int currentRow = row;
  int leftToWrite = totalLength;

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

void OLED::clearDisplay() {
  // Set the GDDRAM to (Row0, Col0), ie: top-left and establish range as the whole screen - 128x64
  std::vector<uint8_t> writeBuffer {
      OLED_CONTROL_BYTE_CMD_STREAM,
      OLED_CMD_SET_COLUMN_RANGE,
      0,
      SCREEN_HORIZONTAL_SIZE,
      OLED_CMD_SET_PAGE_RANGE,
      0,
      7
  };

  i2cWrite(m_fd, writeBuffer);

  std::vector<uint8_t> blankWriteBuffer (16 + 1);
  for (uint8_t x = 0; x < SCREEN_VERTICAL_SIZE; x++) {
    blankWriteBuffer[0] = OLED_CONTROL_BYTE_DATA_STREAM;

    for (uint8_t i = 0; i < 16; i++) {
      blankWriteBuffer[1 + i] = 0;
    }
    i2cWrite(m_fd, blankWriteBuffer);
  }
}

void OLED::clearDisplayAt(uint8_t row, uint8_t column, uint8_t count, uint8_t scale) const {
  // Set the GDDRAM to (Row0, Col0), ie: top-left and establish range as the whole screen - 128x64
  std::vector<uint8_t> writeBuffer {
      OLED_CONTROL_BYTE_CMD_STREAM,
      OLED_CMD_SET_COLUMN_RANGE,
      column,
      static_cast<uint8_t>(column + count * BYTE_SIZE * scale),
      OLED_CMD_SET_PAGE_RANGE,
      static_cast<uint8_t>(7 - row - (row + (scale * 1) - row - 1)),
      static_cast<uint8_t>(7 - row)
  };

  i2cWrite(m_fd, writeBuffer);

  std::vector<uint8_t> blankWriteBuffer(16 + 1);
  for (uint8_t x = 0; x < count * scale; x++) {
    blankWriteBuffer[0] = OLED_CONTROL_BYTE_DATA_STREAM;

    for (uint8_t i = 0; i < 16; i++) {
      blankWriteBuffer[1 + i] = 0;
    }
    i2cWrite(m_fd, blankWriteBuffer);
  }
}

void OLED::setCursor(uint8_t rowStart, uint8_t rowEnd, uint8_t columnStart, uint8_t columnEnd) const {
  std::vector<uint8_t> writeBuffer {
      OLED_CONTROL_BYTE_CMD_STREAM,
      OLED_CMD_SET_COLUMN_RANGE,
      columnStart,
      columnEnd,
      OLED_CMD_SET_PAGE_RANGE,
      static_cast<uint8_t>(7 - rowStart - (rowEnd - rowStart)),
      static_cast<uint8_t>(7 - rowStart)
  };

  i2cWrite(m_fd, writeBuffer);
}

uint8_t byteToScale, bitPoint, temp;

void OLED::scale(uint8_t inp, uint8_t scale) {
  OLED::resetByteBuffer();
  byteToScale = 0, bitPoint = 0, temp = 0;

  for (int i = 0; i < 8; i++) {
    temp = inp & ANDER;
    temp = temp >> bitPoint;
    for (uint8_t j = 0; j < scale; j++) {
      m_doubleScaleBuffer[byteToScale] |= temp;
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
