#include "SenseAirS8.h"

SenseAirS8::SenseAirS8(uint8_t port) {

  switch (port) {
    case 0:
      Serial.begin(9600);
      m_stream = &Serial;
      break;
    case 1:
      Serial1.begin(9600);
      m_stream = &Serial1;
      break;
 #ifdef Serial2
    case 2:
      Serial2.begin(9600);
      m_stream = &Serial2;
      break;
 #endif
 #ifdef Serial3
    case 2:
      Serial3.begin(9600);
      m_stream = &Serial3;
      break;
 #endif
    default:
      m_stream = m_hw_serial= new HardwareSerial(port);
      m_hw_serial->begin(9600);        
  }  
}


SenseAirS8::SenseAirS8(uint8_t rx_pin, uint8_t tx_pin) {
  m_stream = m_sw_serial = new SoftwareSerial(rx_pin, tx_pin, false);
  m_sw_serial->begin(9600);
}


SenseAirS8::~SenseAirS8() {
  
  if (m_sw_serial) {
    delete m_sw_serial;
  }
  if (m_hw_serial) {
    delete m_hw_serial;
  }
}

void SenseAirS8::begin() {
  
  m_sensorId        = _getSensorId();
  m_firmwareVersion = _getFirmwareVersion();
  m_abcPeriod       = _getABCPeriod();
  
#if S8_DEBUG
  Serial.print("\nSetup SenseAir S8\n");
  Serial.printf("Sensor ID: %08X\n", m_sensorId); 
  Serial.printf("Firmware: %d.%d\n", m_firmwareVersion / 256, m_firmwareVersion % 256);
  Serial.printf("ABC Period: %d\n", m_abcPeriod);
#endif  
}

int16_t SenseAirS8::getCO2() {

  _clrReceiveBuffer();
  
  uint16_t co2 = _readInputRegister(0x03);
#if S8_DEBUG    
  Serial.printf("CO2: %d\n", co2);
#endif  
  return co2;
}

uint32_t SenseAirS8::_getSensorId() {
  uint32_t id = _readInputRegister(0x1D) * 65536;
  return id + _readInputRegister(0x1E);   
}

uint16_t SenseAirS8::_getFirmwareVersion() {

  return _readInputRegister(0x1C);  
}


uint16_t SenseAirS8::_getABCPeriod() {

  uint16_t period = _readHoldingRegister(0x1F);
#if S8_DEBUG
  Serial.printf("ABC period: %d\n", period);
#endif  
  return period;
}


void SenseAirS8::setABCPeriod(uint16_t period) {

  _clrReceiveBuffer();

  _writeHoldingRegister(0x1F, period);

  delay(50);
  
  m_abcPeriod = _getABCPeriod();
#if S8_DEBUG
  Serial.printf("ABC period: %d\n", m_abcPeriod);
#endif
}


bool SenseAirS8::backgroundCalibration() {
  uint16_t ack;
  
  _clrReceiveBuffer();
  
  // clear Acknowledgedment register
  if (_writeHoldingRegister(0x0000, 0x0000) == false) {
    return false;
  }

  // write command register 
  if (_writeHoldingRegister(0x0001, 0x7C06) == false) {
    return false;
  }

  // wait 3 seconds
  delay(3000);

  // read Acknowledgedment register
  if ((ack = _readHoldingRegister(0x00)) == 0xFFFF) {
#if S8_DEBUG
    Serial.println("failed");
#endif
    return false; 
  } else {
    ack &= 0x0020;
  }
  
#if S8_DEBUG
  Serial.println(ack ? "succes" : "failed");
#endif

  return ack; 
}


uint16_t SenseAirS8::_readInputRegister(uint8_t reg) {
  uint16_t crc, got;
  uint8_t  buf[8] = {0xFE, 0x04, 0x00, reg, 0x00, 0x01, 0x00, 0x00};

  crc = _modRTU_CRC(buf, 6);
  buf[6] = crc & 0xFF;     // crc low
  buf[7] = crc >> 8;       // crc high  

  _writeModbus(buf, 8);

  if (_readModbus(buf, 7) == false) {
    return 0xFFFF;
  }   
     
  if (buf[2] != 2) {
#if S8_DEBUG
    Serial.printf("Invalid bytecount. Expect: 2 Got: %d \n", buf[2]);
#endif
    return 0xFFFF;   
  }
  
  crc = _modRTU_CRC(buf, 5);
  got = (uint16_t)buf[5] + (uint16_t)buf[6] * 256;
  if (crc != got) {
#if S8_DEBUG    
    Serial.print("Invalid checksum. Expect: ");
    Serial.print(crc, HEX);
    Serial.print("  Got: ");
    Serial.println(got, HEX);
#endif
    return 0xFFFF;
  } 
  got = (uint16_t)buf[3] * 256 + (uint16_t)buf[4]; 
  return got;
}


uint16_t SenseAirS8::_readHoldingRegister(uint8_t reg) {
  uint16_t crc, got;
  uint8_t  buf[8] = { 0xFE, 0x03, 0x00, reg, 0x00, 0x01, 0x00, 0x00};

  crc    = _modRTU_CRC(buf, 6);
  buf[6] = crc & 0xFF;     // crc low
  buf[7] = crc >> 8;       // crc high  

  _writeModbus(buf, 8);

  if (_readModbus(buf, 7) == false) {
    return 0xFFFF;
  }
  if (buf[2] != 2) {
 #if S8_DEBUG   
    Serial.printf("Invalid bytecount. Expect: 2 Got: %d \n", (int)buf[2]);
 #endif
    return 0xFFFF;
  }
  
  crc = _modRTU_CRC(buf, 5);
  got = (uint16_t)buf[5] + (uint16_t)buf[6] * 256;
  if (crc != got) {
#if S8_DEBUG
    Serial.print("Invalid checksum. Expect: ");
    Serial.print(crc, HEX);
    Serial.print("  Got: ");
    Serial.println(got, HEX);
#endif
    return 0xFFFF;    
  }
  got = (uint16_t)buf[3] * 256 + (uint16_t)buf[4]; 
  return got;
}


bool SenseAirS8::_writeHoldingRegister(uint8_t reg, uint16_t val) {
  uint16_t crc, got;
  uint8_t  buf[8] = {0xFE, 0x06, 0x00, reg, val >> 8, val & 0xFF, 0x00, 0x00};

  crc    = _modRTU_CRC(buf, 6);
  buf[6] = crc & 0xFF;     // crc low
  buf[7] = crc >> 8;       // crc high  

  _writeModbus(buf, 8);

  if (_readModbus(buf, 8) == false) {
    return false;
  }

  got =  (uint16_t)buf[3] * 256 + (uint16_t)buf[4];
  if (val != got) {
#if S8_DEBUG    
    Serial.print("Invalid value. Expect: ");
    Serial.print(val);
    Serial.print("  Got: ");
    Serial.println(got);
#endif    
    return false;
  }
  
  crc = _modRTU_CRC(buf, 6);
  got = (uint16_t)buf[5] + (uint16_t)buf[6] * 256;
  if (crc != got) {
#if S8_DEBUG    
    Serial.print("Invalid checksum. Expect: ");
    Serial.print(crc, HEX);
    Serial.print("  Got: ");
    Serial.println(got, HEX);
#endif
    return false;
  }
  return true;
}


bool SenseAirS8::_readModbus(uint8_t* buf, int len) {
  uint32_t timeout = millis() + 100; // 100 ms
  
  memset(buf, 0, len);
  
  for (int i = 0; i < len; ) {
    if (m_stream->available() > 0) {
      buf[i] = m_stream->read();
      i++;
    } else if (millis() > timeout) {
      return false;
    } else {
      yield();
      delay(15);
    }
  }
  return true;
}


void SenseAirS8::_writeModbus(const byte* buf, int len) {
  
  delay(5); 
  m_stream->write(buf, len);
}


// Compute the MODBUS RTU CRC
uint16_t SenseAirS8::_modRTU_CRC(byte* buf, int len) {
  uint16_t crc = 0xFFFF;
  
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc
  
    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;  
}

void SenseAirS8::_clrReceiveBuffer() {
  
  if (m_sw_serial) {
    m_sw_serial->end();
    m_sw_serial->begin(9600);
    delay(10);
  }
    
  while (m_stream->available()) {
    m_stream->read();
  }
}
