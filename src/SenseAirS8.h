#ifndef SENSEAIRS8_HEADER_INCLUDED
#define SENSEAIRS8_HEADER_INCLUDED

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>

#define S8_DEBUG 1

class SenseAirS8 {

public:
   SenseAirS8(uint8_t rx_pin, uint8_t tx_pin); 
   SenseAirS8(uint8_t port);
   ~SenseAirS8();
   
   void           begin();
   int16_t        getCO2();
   uint32_t       getId() { return m_sensorId; }
   uint16_t       getFirmwareVersion() { return m_firmwareVersion; } 
   uint16_t       getABCPeriod() { return m_abcPeriod; }
   void           setABCPeriod(uint16_t);
   bool           backgroundCalibration();   
  
private:
  Stream*         m_stream = nullptr;
  SoftwareSerial* m_sw_serial = nullptr;
  HardwareSerial* m_hw_serial = nullptr;
  uint32_t        m_sensorId;
  uint16_t        m_firmwareVersion;
  uint16_t        m_abcPeriod;

  uint32_t        _getSensorId(); 
  uint16_t        _getFirmwareVersion();
  uint16_t        _getABCPeriod();
  uint16_t        _readInputRegister(uint8_t reg);
  uint16_t        _readHoldingRegister(uint8_t reg);
  bool            _writeHoldingRegister(uint8_t reg, uint16_t val);
  bool            _readModbus(byte* buf, int len);
  void            _writeModbus(const byte* buf, int len);
  uint16_t        _modRTU_CRC(byte* buf, int len);
  void            _clrReceiveBuffer();  
};

#endif  
