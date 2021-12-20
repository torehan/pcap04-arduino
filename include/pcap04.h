#ifndef pcap04_h
#define pcap04_h

#include <Wire.h>
#include <SPI.h>


#include "pcap_types.h"
#include "pcap_registers.h"
#include "pcap_firmware.h"
#include "pcap_calibrations.h"
#include "pcap_configurations.h"

class PCAP04
{
public:
  
  PCAP04(pcap04_version_t version, pcap_serial_interface_t pcap_sif_mode, pcap_measurement_modes_t measurement_mode, char device_specifier, pcap_config_handler_t config_handler, pcap_config_t config); // f_clock:1 MHz, CPOL:0, CPHA:1, MSB First

  // method to initalize pcap04 as slave
  virtual void init_slave();

  virtual pcap_config_handler_t init_nvram();

  // serial print nvram as 16x64 hex array. e.g. 22 8A A0 01 .. .. ..
  virtual void print_nvram();
  
  // serial print configuration registers as 16x4 hex array. e.g. 3F 80 D8 18 .. .. ..
  virtual void print_config();

  virtual pcap_results_t get_results();

  virtual pcap_status_t get_status(bool);

  virtual pcap_config_t get_config();

  virtual void update_config(pcap_config_t );

  virtual bool send_command(unsigned char);

  virtual void reset_pcap_dsp();
private:

  // pin nubmer of the SSN
  unsigned char slave_select_pin = 0;

  bool initialized = false;

  pcap04_version_t pcap_version;

  pcap_measurement_modes_t pcap_measurement_mode;

  pcap_config_t pcap_config;

  pcap_results_t pcap_results;

  // 1KB NVRAM of the PCAP04
  __PCAP_NVRAM_T pcap_nvram;

  // mirror of the device NVRAM
  __PCAP_NVRAM_T pcap_nvram_mirror;

  __PCAP_RESULTS_REGS_T pcap_results_regs;

  // SPI settings for PCAP.
  SPISettings pcap_spi_settings = SPISettings(4000000, MSBFIRST, SPI_MODE1);

  pcap_serial_interface_t sif_mode = PCAP_SPI_MODE;

  pcap_i2c_addr_t device_addr = {.i2caddr = {.wr = 0, .addr = 0, .fixed_addr = 0x0A}};

  // OP Codes
  pcap_opcode_nvram_t wr_mem = {.nvram = {.data = 0, .addr = 0, .op_code = WR_NVRAM}};
  pcap_opcode_nvram_t rd_mem = {.nvram = {.data = 0, .addr = 0, .op_code = RD_NVRAM}};

  pcap_opcode_config_t wr_config = {.config = {.data = 0, .addr = 0, .op_code = WR_CONFIG}};
  pcap_opcode_config_t rd_config = {.config = {.data = 0, .addr = 0, .op_code = RD_CONFIG}};

  pcap_opcode_result_t rd_result = {.result = {.data = 0, .addr = 0, .op_code = RD_RESULT}};

  pcap_opcode_command_t por_reset = {.command = {.op_code = POR_RESET}};
  pcap_opcode_command_t initialize = {.command = {.op_code = INITIALIZE}};
  pcap_opcode_command_t cdc_start = {.command = {.op_code = CDC_START}};
  pcap_opcode_command_t rdc_start = {.command = {.op_code = RDC_START}};
  pcap_opcode_command_t dsp_trig = {.command = {.op_code = DSP_TRIG}};
  pcap_opcode_command_t nv_store = {.command = {.op_code = NV_STORE}};
  pcap_opcode_command_t nv_recall = {.command = {.op_code = NV_RECALL}};
  pcap_opcode_command_t nv_erase = {.command = {.op_code = NV_ERASE}};
  pcap_opcode_testread_t test_read = {.testread = {.fixed = TEST_READ_LOW, .op_code = TEST_READ_HIGH}};


  virtual void spi_read_test();

  virtual unsigned char spi_transmit(unsigned char data);
  virtual unsigned char spi_transmit(unsigned short data);
  virtual unsigned char spi_transmit(unsigned int data);

  //virtual void configure_registers();

  virtual void readall_nvram();
  virtual void read_nvram(unsigned short addr);

  virtual void writeall_nvram();
  virtual void write_nvram(unsigned short addr);

  virtual void readall_config();
  virtual void read_config(unsigned char addr);

  virtual void writeall_config();
  virtual void write_config(unsigned char addr, unsigned char data);

  virtual void readall_result();
  virtual void read_result(unsigned char addr);

  virtual void readall_status();

  virtual void validate_nvram();
};

PCAP04::PCAP04(pcap04_version_t version, pcap_serial_interface_t pcap_sif_mode, pcap_measurement_modes_t measurement_mode, char device_specifier, pcap_config_handler_t config_handler, pcap_config_t config)
{
  sif_mode = pcap_sif_mode;

  if (sif_mode == PCAP_I2C_MODE)
  {
    // set device I2C address
    device_addr.i2caddr.addr = device_specifier;
  }
  else if (sif_mode == PCAP_SPI_MODE)
  {
    // set slave select pin for PCAP04
    slave_select_pin = device_specifier;
  }

  pcap_version = version;

  pcap_measurement_mode = measurement_mode;

  //init_nvram();
  
}
// void PCAP04::i2c_read_test()
// {
//   Serial.print("i2c start\n");
//   device_addr.i2caddr.wr = 0;
//   Wire.beginTransmission((uint8_t)device_addr.i2c_addr);
//   Serial.print(device_addr.i2c_addr, HEX);
//   Serial.print("\n");
//   Serial.print(device_addr.i2c_addr, BIN);
//   Serial.print("\n");
//   Wire.endTransmission(device_addr.i2c_addr);
//   Serial.print("i2c stop\n");
// };

void PCAP04::spi_read_test()
{
  Serial.println("spi read test start");
  SPI.beginTransaction(pcap_spi_settings);
  digitalWrite(slave_select_pin, LOW);

  unsigned short recval = 0;
  //recval = SPI.transfer16(test_read.opcode);
  recval = SPI.transfer((test_read.opcode >> 8) & 0xFF);
  recval = SPI.transfer((test_read.opcode) & 0xFF);

  if (recval == 0x11)
  {
    Serial.println("spi read test:success");
  }
  else
  {
    Serial.println("spi read test:failed");
  }

  digitalWrite(slave_select_pin, HIGH);
  SPI.endTransaction();
  Serial.println("spi read test end");
};

unsigned char PCAP04::spi_transmit(unsigned int data)
{
  static unsigned char recval = 0;
  digitalWrite(slave_select_pin, LOW);
  SPI.transfer((unsigned char)((data >> 16) & 0xFF));
  recval = SPI.transfer16((unsigned short)(data & 0xFFFF)) & 0xFF;
  delayMicroseconds(1);
  digitalWrite(slave_select_pin, HIGH);
  delayMicroseconds(1);
  return recval;
};

unsigned char PCAP04::spi_transmit(unsigned short data)
{
  static unsigned short recval = 0;
  SPI.beginTransaction(pcap_spi_settings);
  digitalWrite(slave_select_pin, LOW);
  recval = SPI.transfer16(data);
  delayMicroseconds(1);
  digitalWrite(slave_select_pin, HIGH);
  delayMicroseconds(1);
  return (unsigned char)(recval & 0xFF);
};

unsigned char PCAP04::spi_transmit(unsigned char data)
{
  static unsigned char recval = 0;
  SPI.beginTransaction(pcap_spi_settings);
  digitalWrite(slave_select_pin, LOW);
  SPI.transfer(data);
  delayMicroseconds(1);
  digitalWrite(slave_select_pin, HIGH);
  delayMicroseconds(1);
  return recval;
};

bool PCAP04::send_command(unsigned char opcode)
{

  if (opcode == POR_RESET)
  {
    spi_transmit(por_reset.opcode);
  }
  else if (opcode == INITIALIZE)
  {
    spi_transmit(initialize.opcode);
  }
  else if (opcode == CDC_START)
  {
    spi_transmit(cdc_start.opcode);
  }
  else if (opcode == RDC_START)
  {
    spi_transmit(rdc_start.opcode);
  }
  else if (opcode == DSP_TRIG)
  {
    spi_transmit(dsp_trig.opcode);
  }
  else if (opcode == NV_STORE)
  {
    spi_transmit(nv_store.opcode);
  }
  else if (opcode == NV_RECALL)
  {
    spi_transmit(nv_recall.opcode);
  }
  else if (opcode == NV_ERASE)
  {
    spi_transmit(nv_erase.opcode);
  }
  else
  {
    return false;
  }
  return true;
}

void PCAP04::init_slave()
{

  initialized = true;

  spi_read_test();

  delay(100);

  send_command(POR_RESET);
  
  delay(2000);

  write_config(47,0); // STOP DSP

  delay(1000);

  //read_config(47);

  readall_nvram();

  Serial.print("RUNBIT:") ;Serial.println(pcap_nvram_mirror.CFG.CFG47.REGVAL,HEX);
  
  delay(1000);

  writeall_nvram();

  delay(5000);

  validate_nvram();

  delay(5000);

  // send_command(CDC_START);
  send_command(INITIALIZE);
  delay(1000);

};

void PCAP04::writeall_config()
{
  // Serial.println("writeall_config start");

  static unsigned char data = 0;

  for (size_t addr = 0; addr < PCAP_NVRAM_CFG_SIZE; addr++)
  {
    if (addr != 47){
      data = *(unsigned char *)(&pcap_nvram.CFG.CFG0 + addr);
      write_config(addr, data);
    }
  }
  write_config(47, pcap_nvram.CFG.CFG47.REGVAL);
  // Serial.println("writeall_config end");
}

void PCAP04::write_config(unsigned char addr, unsigned char data)
{
  // Serial.println("write_config start");

  wr_config.config.addr = addr;
  wr_config.config.data = data;

  SPI.beginTransaction(pcap_spi_settings);
  if ((addr >= 0) && (addr < PCAP_NVRAM_CFG_SIZE - 2)) // size-2 to skip the CHARGE_PUMP registers
  {
/*    
    if (addr==0) {
      Serial.println(pcap_nvram.CFG.CFG0.REGVAL,HEX);
  //      Serial.println((unsigned char)CFG_REG(s));
    }else if (addr==1) {
      Serial.println(pcap_nvram.CFG.CFG1.REGVAL,HEX);
  //      Serial.println((unsigned char)CFG_REG(s));
    }else if (addr==2) {
      Serial.println(pcap_nvram.CFG.CFG2.REGVAL,HEX);
  //      Serial.println((unsigned char)CFG_REG(s));
    }else if (addr==3) {
      Serial.println(pcap_nvram.CFG.CFG3.REGVAL,HEX);
  //      Serial.println((unsigned char)CFG_REG(s));
    }else if (addr==4) {
      Serial.println(pcap_nvram.CFG.CFG4.REGVAL,HEX);
  //      Serial.println((unsigned char)CFG_REG(s));
    }else if (addr==5) {
      Serial.println(pcap_nvram.CFG.CFG5.REGVAL,HEX);
  //      Serial.println((unsigned char)CFG_REG(s));
    }else if (addr==6) {
      Serial.println(pcap_nvram.CFG.CFG6.REGVAL,HEX);
  //      Serial.println((unsigned char)CFG_REG(s));
    }else if (addr==7) {
      Serial.println(pcap_nvram.CFG.CFG7.REGVAL,HEX);
  //      Serial.println((unsigned char)CFG_REG(s));
    }else if (addr==8) {
      Serial.println(pcap_nvram.CFG.CFG8.REGVAL,HEX);
  //      Serial.println((unsigned char)CFG_REG(s));
    }else if (addr==9) {
      Serial.println(pcap_nvram.CFG.CFG9.REGVAL,HEX);
  //      Serial.println((unsigned char)CFG_REG(s)));
    }else if (addr==10) {
      Serial.println(pcap_nvram.CFG.CFG10.REGVAL,HEX);
  //      Serial.println((unsigned char)CFG_REG(s));
    }else if (addr==11) {
      Serial.println(pcap_nvram.CFG.CFG11.REGVAL,HEX);
  //      Serial.println((unsigned char)CFG_REG(s));
    }
    Serial.println(wr_config.config.data,HEX);
*/
    spi_transmit(wr_config.opcode);
  }
  else if ((addr == PCAP_NVRAM_CFG_SIZE - 2) || (addr == PCAP_NVRAM_CFG_SIZE - 1))
  {
    Serial.print("write config to address: ");
    Serial.print(addr);
    Serial.println(" - skipped (CHARGE_PUMP)");
  }
  else
  {
    Serial.println("write config address not in range [0-63] :");
    Serial.print(addr);
  }
  SPI.endTransaction();
  // Serial.println("write_config end");
}

pcap_config_handler_t PCAP04::init_nvram(){
  static pcap_data_vector_t firmware;
  static pcap_data_vector_t calibration;
  static pcap_data_vector_t configuration;

  static unsigned short calib_offset;
  static unsigned short calib_index;

  if (pcap_version == PCAP04_V0)
  {
    if (pcap_measurement_mode == STANDARD)
    {
      firmware.data = &PCap04v0_standard_v1[0];
      firmware.size = sizeof(PCap04v0_standard_v1);
      firmware.nvram_loc = 0x00;

      configuration.data = &PCap04v0_standard_v1_configuration[0];
      configuration.size = sizeof(PCap04v0_standard_v1_configuration);
      configuration.nvram_loc = 0x03C0;

      calibration.data = NULL;
      calibration.size = 0;
      calibration.nvram_loc = 0x320;
    }
    else if (pcap_measurement_mode == HUMIDITY)
    {
      firmware.data = &PCap04v0_linearize_v1_1[0];
      firmware.size = sizeof(PCap04v0_linearize_v1_1);
      firmware.nvram_loc = 0x00;

      configuration.data = &PCap04v0_humidity_v1_configuration[0];
      configuration.size = sizeof(PCap04v0_humidity_v1_configuration);
      configuration.nvram_loc = 0x03C0;

      calibration.data = &PCap04v0_humidity_v1_calibration[0];
      calibration.size = sizeof(PCap04v0_humidity_v1_calibration);
      calibration.nvram_loc = 0x0320;
    }
    else if (pcap_measurement_mode == PRESSURE)
    {
      firmware.data = &PCap04v0_linearize_v1_1[0];
      firmware.size = sizeof(PCap04v0_linearize_v1_1);
      firmware.nvram_loc = 0x00;

      configuration.data = &PCap04v0_pressure_v1_configuration[0];
      configuration.size = sizeof(PCap04v0_pressure_v1_configuration);
      configuration.nvram_loc = 0x03C0;

      calibration.data = &PCap04v0_pressure_v1_calibration[0];
      calibration.size = sizeof(PCap04v0_pressure_v1_calibration);
      calibration.nvram_loc = 0x0320;
    }
  }
  else if (pcap_version == PCAP04_V1)
  {
    if (pcap_measurement_mode == STANDARD)
    {
      firmware.data = &PCap04v1_standard_v1[0];
      firmware.size = sizeof(PCap04v1_standard_v1);
      firmware.nvram_loc = 0x00;

      configuration.data = &PCap04v1_standard_v1_configuration[0];
      configuration.size = sizeof(PCap04v1_standard_v1_configuration);
      configuration.nvram_loc = 0x03C0;

      calibration.data = NULL;
      calibration.size = 0;
      calibration.nvram_loc = 0x320;
    }
    else if (pcap_measurement_mode == HUMIDITY)
    {
      firmware.data = &PCap04v1_linearize_v1[0];
      firmware.size = sizeof(PCap04v1_linearize_v1);
      firmware.nvram_loc = 0x00;

      configuration.data = &PCap04v1_humidity_v1_configuration[0];
      configuration.size = sizeof(PCap04v1_humidity_v1_configuration);
      configuration.nvram_loc = 0x03C0;

      calibration.data = &PCap04v1_humidity_v1_calibration[0];
      calibration.size = sizeof(PCap04v1_humidity_v1_calibration);
      calibration.nvram_loc = 0x0320;
    }
    else if (pcap_measurement_mode == PRESSURE)
    {
      firmware.data = &PCap04v1_linearize_v1[0];
      firmware.size = sizeof(PCap04v1_linearize_v1);
      firmware.nvram_loc = 0x00;

      configuration.data = &PCap04v1_pressure_v1_configuration[0];
      configuration.size = sizeof(PCap04v1_pressure_v1_configuration);
      configuration.nvram_loc = 0x03C0;

      calibration.data = &PCap04v1_pressure_v1_calibration[0];
      calibration.size = sizeof(PCap04v1_humidity_v1_calibration);
      calibration.nvram_loc = 0x0320;
    }
  }

  calib_offset = calibration.nvram_loc - PCAP_NVRAM_FW_SIZE;
  calib_index = PCAP_NVRAM_MAX_INDEX_FW_CAL0 - calib_offset;

  // copy firmware to pcap nvram
  if (firmware.size <= PCAP_NVRAM_FW_SIZE){
    memcpy(&pcap_nvram.FW, firmware.data, firmware.size);
  }else {
    memcpy(&pcap_nvram.FW, firmware.data, PCAP_NVRAM_FW_SIZE);
    memcpy(&pcap_nvram.FW_CAL0, firmware.data + PCAP_NVRAM_FW_SIZE, firmware.size-PCAP_NVRAM_FW_SIZE);
  }

  // copy calibration data to pcap nvram if exists
  if (calibration.size > 0){
    if(calibration.size <= calib_index) {
        memcpy(&pcap_nvram.FW_CAL0.data[calib_offset], calibration.data, calibration.size);
    }else {
      memcpy(&pcap_nvram.FW_CAL0.data[calib_offset], calibration.data, calib_index);
      memcpy(&pcap_nvram.FW_CAL1.data[0], calibration.data + calib_index, calibration.size - calib_index);
    }
  }



/*   Serial.println((long long)&pcap_nvram,HEX);
  Serial.println((long long)&pcap_nvram.CFG,HEX);
  
  Serial.print("configuration size: "); Serial.println(configuration.size);
  //Serial.println(pcap_nvram.CFG.CFG0.CFG0);
  for (uint32_t j = 0;j<configuration.size;j++){
    delay(50);
    if (*(configuration.data+j) <= 0x0F){ 
      Serial.print("0");Serial.print(*(configuration.data+j),HEX);Serial.print(" ");
    }else{
      Serial.print(*(configuration.data+j),HEX);Serial.print(" ");
    }
    if ((j+1)%16 == 0){
      Serial.print(j);
      Serial.println();
    }    
  }
 */

  // copy configuration data to pcap nvram
  memcpy(&pcap_nvram.CFG.CFG0, configuration.data, configuration.size);

  return pcap_config_handler_t (&pcap_nvram.CFG);
}

void PCAP04::readall_nvram()
{
  // Serial.println("readall_nvram start");

  for (size_t addr = 0; addr < PCAP_NVRAM_SIZE; addr++)
  {
    read_nvram(addr);
  }

  // Serial.println("readall_nvram end");
};

void PCAP04::read_nvram(unsigned short addr)
{
  static unsigned char *cfg_p = nullptr;

  // Serial.println("read_nvram start");

  SPI.beginTransaction(pcap_spi_settings);

  if ((addr >= 0) && (addr < PCAP_NVRAM_SIZE))
  {
    rd_mem.nvram.addr = 0xFFFF & addr;
    rd_mem.nvram.data = 0;

    rd_mem.nvram.data = spi_transmit(rd_mem.opcode);

    size_t s = 0;

    if (rd_mem.nvram.addr >= 0 && rd_mem.nvram.addr < PCAP_NVRAM_MAX_INDEX_FW)
    {
      s = rd_mem.nvram.addr;
      pcap_nvram_mirror.FW.data[s] = rd_mem.nvram.data;
    }
    else if (rd_mem.nvram.addr >= PCAP_NVRAM_MAX_INDEX_FW && rd_mem.nvram.addr < PCAP_NVRAM_MAX_INDEX_FW_CAL0)
    {
      s = rd_mem.nvram.addr - PCAP_NVRAM_MAX_INDEX_FW;
      pcap_nvram_mirror.FW_CAL0.data[s] = rd_mem.nvram.data;
    }
    else if (rd_mem.nvram.addr >= PCAP_NVRAM_MAX_INDEX_FW_CAL0 && rd_mem.nvram.addr < PCAP_NVRAM_MAX_INDEX_FW_CAL1)
    {
      s = rd_mem.nvram.addr - PCAP_NVRAM_MAX_INDEX_FW_CAL0;
      pcap_nvram_mirror.FW_CAL1.data[s] = rd_mem.nvram.data;
    }
    else if (rd_mem.nvram.addr >= PCAP_NVRAM_MAX_INDEX_FW_CAL1 && rd_mem.nvram.addr < PCAP_NVRAM_SIZE)
    {

      s = rd_mem.nvram.addr - PCAP_NVRAM_MAX_INDEX_FW_CAL1;

      cfg_p = (unsigned char *)(&pcap_nvram_mirror.CFG.CFG0 + s);
      // Serial.print("&(pcap_nvram_mirror.CFG.CFG0 + s):"); Serial.println((unsigned long long)(&pcap_nvram_mirror.CFG.CFG0 + s),HEX);
      *cfg_p = (unsigned char)rd_mem.nvram.data;
/*
      if (s==0) {
        Serial.print("&pcap_nvram_mirror.CFG.CFG0.REGVAL:"); Serial.println((unsigned long long)(&pcap_nvram_mirror.CFG.CFG0.REGVAL),HEX);
        Serial.println((unsigned char)pcap_nvram_mirror.CFG.CFG0.REGVAL,HEX);
      }else if (s==1) {
        Serial.print("&pcap_nvram_mirror.CFG.CFG1.REGVAL:"); Serial.println((unsigned long long)(&pcap_nvram_mirror.CFG.CFG1.REGVAL),HEX);
        Serial.println(pcap_nvram_mirror.CFG.CFG1.REGVAL,HEX);
      }else if (s==2) {
        Serial.print("&pcap_nvram_mirror.CFG.CFG2.REGVAL:"); Serial.println((unsigned long long)(&pcap_nvram_mirror.CFG.CFG2.REGVAL),HEX);
        Serial.println(pcap_nvram_mirror.CFG.CFG2.REGVAL,HEX);
      }else if (s==3) {
        Serial.print("&pcap_nvram_mirror.CFG.CFG3.REGVAL:"); Serial.println((unsigned long long)(&pcap_nvram_mirror.CFG.CFG3.REGVAL),HEX);
        Serial.println(pcap_nvram_mirror.CFG.CFG3.REGVAL,HEX);
      }else if (s==4) {
        Serial.print("&pcap_nvram_mirror.CFG.CFG4.REGVAL:"); Serial.println((unsigned long long)(&pcap_nvram_mirror.CFG.CFG4.REGVAL),HEX);
        Serial.println(pcap_nvram_mirror.CFG.CFG4.REGVAL,HEX);
      }else if (s==5) {
        Serial.print("&pcap_nvram_mirror.CFG.CFG5.REGVAL:"); Serial.println((unsigned long long)(&pcap_nvram_mirror.CFG.CFG5.REGVAL),HEX);
        Serial.println(pcap_nvram_mirror.CFG.CFG5.REGVAL,HEX);
      }else if (s==6) {
        Serial.print("&pcap_nvram_mirror.CFG.CFG6.REGVAL:"); Serial.println((unsigned long long)(&pcap_nvram_mirror.CFG.CFG6.REGVAL),HEX);
        Serial.println(pcap_nvram_mirror.CFG.CFG6.REGVAL,HEX);
      }else if (s==7) {
        Serial.print("&pcap_nvram_mirror.CFG.CFG7.REGVAL:"); Serial.println((unsigned long long)(&pcap_nvram_mirror.CFG.CFG7.REGVAL),HEX);
        Serial.println(pcap_nvram_mirror.CFG.CFG7.REGVAL,HEX);
      }else if (s==8) {
        Serial.print("&pcap_nvram_mirror.CFG.CFG8.REGVAL:"); Serial.println((unsigned long long)(&pcap_nvram_mirror.CFG.CFG8.REGVAL),HEX);
        Serial.println(pcap_nvram_mirror.CFG.CFG8.REGVAL,HEX);
      }else if (s==9) {
        Serial.print("&pcap_nvram_mirror.CFG.CFG9.REGVAL:"); Serial.println((unsigned long long)(&pcap_nvram_mirror.CFG.CFG9.REGVAL),HEX);
        Serial.println(pcap_nvram_mirror.CFG.CFG9.REGVAL,HEX);
      }else if (s==10) {
        Serial.print("&pcap_nvram_mirror.CFG.CFG10.REGVAL:"); Serial.println((unsigned long long)(&pcap_nvram_mirror.CFG.CFG10.REGVAL),HEX);
        Serial.println(pcap_nvram_mirror.CFG.CFG10.REGVAL,HEX);
      }else if (s==11) {
        Serial.print("&pcap_nvram_mirror.CFG.CFG11.REGVAL:"); Serial.println((unsigned long long)(&pcap_nvram_mirror.CFG.CFG11.REGVAL),HEX);
        Serial.println(pcap_nvram_mirror.CFG.CFG11.REGVAL,HEX);
      }
*/      
    }
  }
  else
  {
    Serial.print("read nvram address not in range [0-1023] :");
    Serial.println(addr);
  }
  SPI.endTransaction();
  // Serial.println("read_nvram end");
};

void PCAP04::readall_config()
{

  // Serial.println("readall_config start");
  for (size_t addr = 0; addr < PCAP_NVRAM_CFG_SIZE; addr++)
  {
    read_config(addr);
  }
  // Serial.println("readall_config end");
};

void PCAP04::read_config(unsigned char addr)
{
  static unsigned char *cfg_p = nullptr;

  // Serial.println("read_config start");

  SPI.beginTransaction(pcap_spi_settings);

  if ((addr >= 0) && (addr < PCAP_NVRAM_CFG_SIZE))
  {
    rd_config.config.addr = addr;
    rd_config.config.data = 0;

    rd_config.config.data = spi_transmit(rd_config.opcode);

    cfg_p = (unsigned char *)(&pcap_nvram_mirror.CFG.CFG0 + rd_config.config.addr);
    *cfg_p = (unsigned char)rd_config.config.data;
  }
  else
  {
    Serial.print("read config address not in range [0-63] :");
    Serial.println(addr);
  }
  SPI.endTransaction();
  // Serial.println("read_config end");
}

void PCAP04::readall_result()
{

//  Serial.println("readall_result start");

  for (size_t addr = 0; addr < PCAP_RESULTS_SIZE; addr++)
  {
    read_result(addr);
  }
//  Serial.println("readall_result end");
};

void PCAP04::readall_status()
{

//  Serial.println("readall_result start");

  for (size_t addr = PCAP_RESULTS_SIZE-3; addr < PCAP_RESULTS_SIZE; addr++)
  {
    read_result(addr);
  }
//  Serial.println("readall_result end");
};

void PCAP04::read_result(unsigned char addr)
{
  static unsigned int *res_p = nullptr;
  static int byte_offset = 0;
  static unsigned char _addr = 0;

  static unsigned int temp = 0;

  // Serial.println("read_result start");
  _addr = (unsigned char) addr / 4;

  byte_offset = (int)addr % 4;

  SPI.beginTransaction(pcap_spi_settings);

  if ((addr >= 0) && (addr < PCAP_RESULTS_SIZE))
  {
    rd_result.result.addr = addr;
    rd_result.result.data = 0;

    rd_result.result.data = spi_transmit(rd_result.opcode);

    if (_addr < 8)
    {
      if (rd_result.result.addr != 0)
      {
        if ((rd_result.result.addr % 4) == 0)
        {
          temp = 0;
        }
      }
      // Serial.print("(&pcap_results_regs.RES0.REGVAL + _addr :");
      // Serial.println((unsigned long long)(&pcap_results_regs.RES0.REGVAL) + _addr,HEX);

      res_p = (unsigned int *)(&pcap_results_regs.RES0.REGVAL + _addr);

      // Serial.print("rd_result.result.data :"); Serial.println((unsigned int)((rd_result.result.data & 0xFF) << byte_offset*8),HEX);
      // Serial.print("byte_offset :"); Serial.println(byte_offset,HEX);

      temp |= (rd_result.result.data << byte_offset * 8);
      // Serial.print("temp :"); Serial.println((unsigned int) temp,HEX);

      *res_p = temp;
/*
      if (_addr == 0 && byte_offset == 3)
      {
        Serial.print("pcap_results_regs.RES0.REGVAL : ");
        Serial.println(pcap_results_regs.RES0.REGVAL, HEX);
      };
      if (_addr == 1 && byte_offset == 3)
      {
        Serial.print("pcap_results_regs.RES1.REGVAL : ");
        Serial.println(pcap_results_regs.RES1.REGVAL, HEX);
      };
      if (_addr == 2 && byte_offset == 3)
      {
        Serial.print("pcap_results_regs.RES2.REGVAL : ");
        Serial.println(pcap_results_regs.RES2.REGVAL, HEX);
      };
      if (_addr == 3 && byte_offset == 3)
      {
        Serial.print("pcap_results_regs.RES3.REGVAL : ");
        Serial.println(pcap_results_regs.RES3.REGVAL, HEX);
      };
      if (_addr == 4 && byte_offset == 3)
      {
        Serial.print("pcap_results_regs.RES4.REGVAL : ");
        Serial.println(pcap_results_regs.RES4.REGVAL, HEX);
      };
      if (_addr == 5 && byte_offset == 3)
      {
        Serial.print("pcap_results_regs.RES5.REGVAL : ");
        Serial.println(pcap_results_regs.RES5.REGVAL, HEX);
      };
      if (_addr == 6 && byte_offset == 3)
      {
        Serial.print("pcap_results_regs.RES6.REGVAL : ");
        Serial.println(pcap_results_regs.RES6.REGVAL, HEX);
      };
      if (_addr == 7 && byte_offset == 3)
      {
        Serial.print("pcap_results_regs.RES7.REGVAL : ");
        Serial.println(pcap_results_regs.RES7.REGVAL, HEX);
      };
*/
    }
    else if (_addr == 8)
    {
      if (byte_offset == 0)
      {
        pcap_results_regs.STATUS0.REGVAL = rd_result.result.data;
        // Serial.print("pcap_results_regs.STATUS0.REGVAL : ");
        // Serial.print(" a: ");Serial.print(_addr); Serial.print(" b: "); Serial.print(byte_offset); Serial.print(" "); 
        //Serial.print(pcap_results_regs.STATUS0.REGVAL, BIN);Serial.print(" ");
        
        // Serial.println(pcap_results_regs.REGS.STATUS0.REGVAL, HEX);
      }
      else if (byte_offset == 1)
      {
        pcap_results_regs.STATUS1.REGVAL = rd_result.result.data;
        // Serial.print("pcap_results_regs.STATUS1.REGVAL : ");
        // Serial.print(" a: ");Serial.print(_addr); Serial.print(" b: "); Serial.print(byte_offset); Serial.print(" ");
         //Serial.print(pcap_results_regs.STATUS1.REGVAL, BIN);Serial.print(" ");
        
        // Serial.println(pcap_results_regs.REGS.STATUS1.REGVAL, HEX);
      }
      else if (byte_offset == 2)
      {
        pcap_results_regs.STATUS2.REGVAL = rd_result.result.data;
        // Serial.print("pcap_results_regs.STATUS2.REGVAL : ");
        // Serial.print(" a: ");Serial.print(_addr); Serial.print(" b: "); Serial.print(byte_offset); Serial.print(" ");
        //Serial.print(pcap_results_regs.STATUS2.REGVAL, BIN);Serial.println(" ***");
        // Serial.println(pcap_results_regs.REGS.STATUS2.REGVAL, HEX);
      }
    }
  
  }
  else
  {
    Serial.print("read result address not in range [0-34] :");
    Serial.println(addr);
  }
  SPI.endTransaction();
  // Serial.println("read_result end");
}

void PCAP04::update_config(pcap_config_t pcap_config)
{

  pcap_nvram.CFG.CFG0.CFG0.I2C_A = pcap_config.I2C_A;
  pcap_nvram.CFG.CFG0.CFG0.OLF_CTUNE = pcap_config.OLF_CTUNE;
  pcap_nvram.CFG.CFG0.CFG0.OLF_FTUNE = pcap_config.OLF_FTUNE;

  pcap_nvram.CFG.CFG1.CFG1.OX_DIS = pcap_config.OX_DIS;
  pcap_nvram.CFG.CFG1.CFG1.OX_DIV4 = pcap_config.OX_DIV4;
  //pcap_nvram.CFG.CFG1.CFG1.OX_AUTOSTOP_DIS = 0;
  //pcap_nvram.CFG.CFG1.CFG1.OX_STOP = 0;
  pcap_nvram.CFG.CFG1.CFG1.OX_RUN = pcap_config.OX_RUN;

  pcap_nvram.CFG.CFG2.CFG2.RDCHG_INT_SEL0 = pcap_config.RDCHG_INT_SEL0;
  pcap_nvram.CFG.CFG2.CFG2.RDCHG_INT_SEL1 = pcap_config.RDCHG_INT_SEL1;
  pcap_nvram.CFG.CFG2.CFG2.RDCHG_INT_EN = pcap_config.RDCHG_INT_EN;
  pcap_nvram.CFG.CFG2.CFG2.RDCHG_EXT_EN = pcap_config.RDCHG_EXT_EN;

  pcap_nvram.CFG.CFG3.CFG3.AUX_PD_DIS = pcap_config.AUX_PD_DIS;
  pcap_nvram.CFG.CFG3.CFG3.AUX_CINT = pcap_config.AUX_CINT;
  pcap_nvram.CFG.CFG3.CFG3.RDCHG_OPEN = pcap_config.RDCHG_OPEN;
  pcap_nvram.CFG.CFG3.CFG3.RDCHG_PERM_EN = pcap_config.RDCHG_PERM_EN;
  pcap_nvram.CFG.CFG3.CFG3.RDCHG_EXT_PERM = pcap_config.RDCHG_EXT_PERM;
  pcap_nvram.CFG.CFG3.CFG3.RCHG_SEL = pcap_config.RCHG_SEL;

  pcap_nvram.CFG.CFG4.CFG4.C_REF_INT = pcap_config.C_REF_INT;
  pcap_nvram.CFG.CFG4.CFG4.C_COMP_EXT = pcap_config.C_COMP_EXT;
  pcap_nvram.CFG.CFG4.CFG4.C_COMP_INT = pcap_config.C_COMP_INT;
  pcap_nvram.CFG.CFG4.CFG4.C_DIFFERENTIAL = pcap_config.C_DIFFERENTIAL;
  pcap_nvram.CFG.CFG4.CFG4.C_FLOATING = pcap_config.C_FLOATING;

  pcap_nvram.CFG.CFG5.CFG5.CY_PRE_MR1_SHORT = pcap_config.CY_PRE_MR1_SHORT;
  pcap_nvram.CFG.CFG5.CFG5.C_PORT_PAT = pcap_config.C_PORT_PAT;
  pcap_nvram.CFG.CFG5.CFG5.CY_HFCLK_SEL = pcap_config.CY_HFCLK_SEL;
  pcap_nvram.CFG.CFG5.CFG5.CY_DIV4_DIS = pcap_config.CY_DIV4_DIS;
  pcap_nvram.CFG.CFG5.CFG5.CY_PRE_LONG = pcap_config.CY_PRE_LONG;
  pcap_nvram.CFG.CFG5.CFG5.C_DC_BALANCE = pcap_config.C_DC_BALANCE;

  pcap_nvram.CFG.CFG6.CFG6.C_PORT_EN = pcap_config.C_PORT_EN;

  pcap_nvram.CFG.CFG7.CFG7.C_AVRG_LOW = pcap_config.C_AVRG & 0xFF;
  pcap_nvram.CFG.CFG8.CFG8.C_AVRG_HIGH = (pcap_config.C_AVRG >> 8) & 0x1F;

  pcap_nvram.CFG.CFG9.CFG9.CONV_TIME_LOW = pcap_config.CONV_TIME & 0xFF;
  pcap_nvram.CFG.CFG10.CFG10.CONV_TIME_MID = (pcap_config.CONV_TIME >> 8) & 0xFF;
  pcap_nvram.CFG.CFG11.CFG11.CONV_TIME_HIGH = (pcap_config.CONV_TIME >> 16) & 0x7F;

  pcap_nvram.CFG.CFG12.CFG12.DISCHARGE_TIME_LOW = pcap_config.DISCHARGE_TIME & 0xFF;
  pcap_nvram.CFG.CFG13.CFG13.DISCHARGE_TIME_HIGH = (pcap_config.DISCHARGE_TIME >> 8) & 0x03;
  pcap_nvram.CFG.CFG13.CFG13.C_STARTONPIN = pcap_config.C_STARTONPIN;
  pcap_nvram.CFG.CFG13.CFG13.C_TRIG_SEL = pcap_config.C_TRIG_SEL;

  pcap_nvram.CFG.CFG14.CFG14.PRECHARGE_TIME_LOW = pcap_config.PRECHARGE_TIME & 0xFF;
  pcap_nvram.CFG.CFG15.CFG15.PRECHARGE_TIME_HIGH = (pcap_config.PRECHARGE_TIME >> 8) & 0x03;
  pcap_nvram.CFG.CFG15.CFG15.C_FAKE = pcap_config.C_FAKE;

  pcap_nvram.CFG.CFG16.CFG16.FULLCHARGE_TIME_LOW = pcap_config.FULLCHARGE_TIME & 0xFF;
  pcap_nvram.CFG.CFG17.CFG17.FULLCHARGE_TIME_HIGH = (pcap_config.FULLCHARGE_TIME >> 8) & 0x03;
  pcap_nvram.CFG.CFG17.CFG17.C_REF_SEL = pcap_config.C_REF_SEL;

  pcap_nvram.CFG.CFG18.CFG18.C_G_OP_RUN = pcap_config.C_G_OP_RUN;
  pcap_nvram.CFG.CFG18.CFG18.C_G_OP_EXT = pcap_config.C_G_OP_EXT;
  pcap_nvram.CFG.CFG18.CFG18.C_G_EN = pcap_config.C_G_EN;

  pcap_nvram.CFG.CFG19.CFG19.C_G_OP_VU = pcap_config.C_G_OP_VU;
  pcap_nvram.CFG.CFG19.CFG19.C_G_OP_ATTN = pcap_config.C_G_OP_ATTN;
  pcap_nvram.CFG.CFG19.CFG19.C_G_TIME = pcap_config.C_G_TIME;

  pcap_nvram.CFG.CFG20.CFG20.R_CY = pcap_config.R_CY;
  pcap_nvram.CFG.CFG20.CFG20.C_G_OP_TR = pcap_config.C_G_OP_TR;

  pcap_nvram.CFG.CFG21.CFG21.R_TRIG_PREDIV_LOW = pcap_config.R_TRIG_PREDIV & 0xFF;
  pcap_nvram.CFG.CFG22.CFG22.R_TRIG_PREDIV_HIGH = (pcap_config.R_TRIG_PREDIV >> 8) & 0xFF;
  pcap_nvram.CFG.CFG22.CFG22.R_TRIG_SEL = pcap_config.R_TRIG_SEL;
  pcap_nvram.CFG.CFG22.CFG22.R_AVRG = pcap_config.R_AVRG;

  pcap_nvram.CFG.CFG23.CFG23.R_PORT_EN = pcap_config.R_PORT_EN;
  pcap_nvram.CFG.CFG23.CFG23.R_PORT_EN_IMES = pcap_config.R_PORT_EN_IMES;
  pcap_nvram.CFG.CFG23.CFG23.R_PORT_EN_IREF = pcap_config.R_PORT_EN_IREF;
  pcap_nvram.CFG.CFG23.CFG23.R_FAKE = pcap_config.R_FAKE;
  pcap_nvram.CFG.CFG23.CFG23.R_STARTONPIN = pcap_config.R_STARTONPIN;

  pcap_nvram.CFG.CFG27.CFG27.DSP_MOFLO_EN = pcap_config.DSP_MOFLO_EN;
  pcap_nvram.CFG.CFG27.CFG27.DSP_SPEED = pcap_config.DSP_SPEED;
  pcap_nvram.CFG.CFG27.CFG27.PG0xPG2 = pcap_config.PG0xPG2;
  pcap_nvram.CFG.CFG27.CFG27.PG1xPG3 = pcap_config.PG1xPG3;

  pcap_nvram.CFG.CFG28.CFG28.WD_DIS = pcap_config.WD_DIS;

  pcap_nvram.CFG.CFG29.CFG29.DSP_STARTONPIN = pcap_config.DSP_STARTONPIN;
  pcap_nvram.CFG.CFG29.CFG29.DSP_FF_IN = pcap_config.DSP_FF_IN;

  pcap_nvram.CFG.CFG30.CFG30.PG5_INTN_EN = pcap_config.PG5_INTN_EN;
  pcap_nvram.CFG.CFG30.CFG30.PG4_INTN_EN = pcap_config.PG4_INTN_EN;
  pcap_nvram.CFG.CFG30.CFG30.DSP_START_EN = pcap_config.DSP_START_EN;

  pcap_nvram.CFG.CFG31.CFG31.PI1_TOGGLE_EN = pcap_config.PI1_TOGGLE_EN;
  pcap_nvram.CFG.CFG31.CFG31.PI0_TOGGLE_EN = pcap_config.PI0_TOGGLE_EN;
  pcap_nvram.CFG.CFG31.CFG31.PI0_RES = pcap_config.PI0_RES;
  pcap_nvram.CFG.CFG31.CFG31.PI0_PDM_SEL = pcap_config.PI0_PDM_SEL;
  pcap_nvram.CFG.CFG31.CFG31.PI0_CLK_SEL = pcap_config.PI0_CLK_SEL;

  pcap_nvram.CFG.CFG32.CFG32.PI1_RES = pcap_config.PI1_RES;
  pcap_nvram.CFG.CFG32.CFG32.PI1_PDM_SEL = pcap_config.PI1_PDM_SEL;
  pcap_nvram.CFG.CFG32.CFG32.PI1_CLK_SEL = pcap_config.PI1_CLK_SEL;

  pcap_nvram.CFG.CFG33.CFG33.PG_DIR_IN = pcap_config.PG_DIR_IN;
  pcap_nvram.CFG.CFG33.CFG33.PG_PU = pcap_config.PG_PU;

  pcap_nvram.CFG.CFG34.CFG34.INT_TRIG_BG = pcap_config.INT_TRIG_BG;
  pcap_nvram.CFG.CFG34.CFG34.DSP_TRIG_BG = pcap_config.DSP_TRIG_BG;
  pcap_nvram.CFG.CFG34.CFG34.BG_PERM = pcap_config.BG_PERM;
  pcap_nvram.CFG.CFG34.CFG34.AUTOSTART = pcap_config.AUTOSTART;

  pcap_nvram.CFG.CFG35.CFG35.CDC_GAIN_CORR = pcap_config.CDC_GAIN_CORR;
  pcap_nvram.CFG.CFG38.CFG38.BG_TIME = pcap_config.BG_TIME;

  pcap_nvram.CFG.CFG39.CFG39.PULSE_SEL0 = pcap_config.PULSE_SEL0;
  pcap_nvram.CFG.CFG39.CFG39.PULSE_SEL1 = pcap_config.PULSE_SEL1;

  pcap_nvram.CFG.CFG40.CFG40.C_SENSE_SEL = pcap_config.C_SENSE_SEL;

  pcap_nvram.CFG.CFG41.CFG41.R_SENSE_SEL = pcap_config.R_SENSE_SEL;

  pcap_nvram.CFG.CFG42.CFG42.ALARM1_SELECT = pcap_config.ALARM1_SELECT;
  pcap_nvram.CFG.CFG42.CFG42.ALARM0_SELECT = pcap_config.ALARM0_SELECT;
  pcap_nvram.CFG.CFG42.CFG42.EN_ASYNC_READ = pcap_config.EN_ASYNC_READ;
  pcap_nvram.CFG.CFG42.CFG42.HS_MODE_SEL = pcap_config.HS_MODE_SEL;
  pcap_nvram.CFG.CFG42.CFG42.R_MEDIAN_EN = pcap_config.R_MEDIAN_EN;
  pcap_nvram.CFG.CFG42.CFG42.C_MEDIAN_EN = pcap_config.C_MEDIAN_EN;

  pcap_nvram.CFG.CFG47.CFG47.RUNBIT = pcap_config.RUNBIT;

  pcap_nvram.CFG.CFG48.CFG48.MEM_LOCK = pcap_config.MEM_LOCK;

  pcap_nvram.CFG.CFG49.CFG49.SERIAL_NUMBER_LOW = pcap_config.SERIAL_NUMBER & 0xFF;
  pcap_nvram.CFG.CFG50.CFG50.SERIAL_NUMBER_HIGH = (pcap_config.SERIAL_NUMBER >> 8) & 0xFF;

  pcap_nvram.CFG.CFG54.CFG54.MEM_CTRL = pcap_config.MEM_CTRL;

  writeall_config();

};

pcap_config_t PCAP04::get_config()
{
  static pcap_config_t pcap_config;

  pcap_config.I2C_A = pcap_nvram.CFG.CFG0.CFG0.I2C_A;
  pcap_config.OLF_CTUNE = pcap_nvram.CFG.CFG0.CFG0.OLF_CTUNE;
  pcap_config.OLF_FTUNE = pcap_nvram.CFG.CFG0.CFG0.OLF_FTUNE;

  pcap_config.OX_DIS = pcap_nvram.CFG.CFG1.CFG1.OX_DIS;
  pcap_config.OX_DIV4 = pcap_nvram.CFG.CFG1.CFG1.OX_DIV4;
  pcap_config.OX_RUN = pcap_nvram.CFG.CFG1.CFG1.OX_RUN;

  pcap_config.RDCHG_INT_SEL0 = pcap_nvram.CFG.CFG2.CFG2.RDCHG_INT_SEL0;
  pcap_config.RDCHG_INT_SEL1 = pcap_nvram.CFG.CFG2.CFG2.RDCHG_INT_SEL1;
  pcap_config.RDCHG_INT_EN = pcap_nvram.CFG.CFG2.CFG2.RDCHG_INT_EN;
  pcap_config.RDCHG_EXT_EN = pcap_nvram.CFG.CFG2.CFG2.RDCHG_EXT_EN;

  pcap_config.AUX_PD_DIS = pcap_nvram.CFG.CFG3.CFG3.AUX_PD_DIS;
  pcap_config.AUX_CINT = pcap_nvram.CFG.CFG3.CFG3.AUX_CINT;
  pcap_config.RDCHG_OPEN = pcap_nvram.CFG.CFG3.CFG3.RDCHG_OPEN;
  pcap_config.RDCHG_PERM_EN = pcap_nvram.CFG.CFG3.CFG3.RDCHG_PERM_EN;
  pcap_config.RDCHG_EXT_PERM = pcap_nvram.CFG.CFG3.CFG3.RDCHG_EXT_PERM;
  pcap_config.RCHG_SEL = pcap_nvram.CFG.CFG3.CFG3.RCHG_SEL;

  pcap_config.C_REF_INT = pcap_nvram.CFG.CFG4.CFG4.C_REF_INT;
  pcap_config.C_COMP_EXT = pcap_nvram.CFG.CFG4.CFG4.C_COMP_EXT;
  pcap_config.C_COMP_INT = pcap_nvram.CFG.CFG4.CFG4.C_COMP_INT;
  pcap_config.C_DIFFERENTIAL = pcap_nvram.CFG.CFG4.CFG4.C_DIFFERENTIAL;
  pcap_config.C_FLOATING = pcap_nvram.CFG.CFG4.CFG4.C_FLOATING;

  pcap_config.CY_PRE_MR1_SHORT = pcap_nvram.CFG.CFG5.CFG5.CY_PRE_MR1_SHORT;
  pcap_config.C_PORT_PAT = pcap_nvram.CFG.CFG5.CFG5.C_PORT_PAT;
  pcap_config.CY_HFCLK_SEL = pcap_nvram.CFG.CFG5.CFG5.CY_HFCLK_SEL;
  pcap_config.CY_DIV4_DIS = pcap_nvram.CFG.CFG5.CFG5.CY_DIV4_DIS;
  pcap_config.CY_PRE_LONG = pcap_nvram.CFG.CFG5.CFG5.CY_PRE_LONG;
  pcap_config.C_DC_BALANCE = pcap_nvram.CFG.CFG5.CFG5.C_DC_BALANCE;

  pcap_config.C_PORT_EN = pcap_nvram.CFG.CFG6.CFG6.C_PORT_EN;

  pcap_config.C_AVRG = (pcap_nvram.CFG.CFG8.CFG8.C_AVRG_HIGH << 8) |\
                       pcap_nvram.CFG.CFG7.CFG7.C_AVRG_LOW;

  // Serial.print("pcap_config.C_AVRG :");  Serial.println(pcap_config.C_AVRG,HEX);

  pcap_config.CONV_TIME = (pcap_nvram.CFG.CFG11.CFG11.CONV_TIME_HIGH << 16) |\
                          (pcap_nvram.CFG.CFG10.CFG10.CONV_TIME_MID << 8 ) |\
                           pcap_nvram.CFG.CFG9.CFG9.CONV_TIME_LOW;


  pcap_config.DISCHARGE_TIME = (pcap_nvram.CFG.CFG13.CFG13.DISCHARGE_TIME_HIGH << 8) |\
                                pcap_nvram.CFG.CFG12.CFG12.DISCHARGE_TIME_LOW;

  pcap_config.C_STARTONPIN = pcap_nvram.CFG.CFG13.CFG13.C_STARTONPIN;
  pcap_config.C_TRIG_SEL = pcap_nvram.CFG.CFG13.CFG13.C_TRIG_SEL;


  pcap_config.PRECHARGE_TIME = (pcap_nvram.CFG.CFG15.CFG15.PRECHARGE_TIME_HIGH << 8) |\
                                pcap_nvram.CFG.CFG14.CFG14.PRECHARGE_TIME_LOW;

  pcap_config.C_FAKE = pcap_nvram.CFG.CFG15.CFG15.C_FAKE;


  pcap_config.FULLCHARGE_TIME = (pcap_nvram.CFG.CFG17.CFG17.FULLCHARGE_TIME_HIGH << 8) |\
                                 pcap_nvram.CFG.CFG16.CFG16.FULLCHARGE_TIME_LOW;

  // Serial.print("pcap_config.FULLCHARGE_TIME :");  Serial.println(pcap_config.FULLCHARGE_TIME,HEX);
  // Serial.print("pcap_nvram.CFG.CFG17.CFG17.FULLCHARGE_TIME_HIGH :");  Serial.println(pcap_nvram.CFG.CFG17.CFG17.FULLCHARGE_TIME_HIGH,HEX);
  // Serial.print("pcap_nvram.CFG.CFG17.CFG17.FULLCHARGE_TIME_HIGH :");  Serial.println(pcap_nvram.CFG.CFG17.CFG17.FULLCHARGE_TIME_HIGH,HEX);


  pcap_config.C_REF_SEL = pcap_nvram.CFG.CFG17.CFG17.C_REF_SEL;


  pcap_config.C_G_OP_RUN = pcap_nvram.CFG.CFG18.CFG18.C_G_OP_RUN;
  pcap_config.C_G_OP_EXT = pcap_nvram.CFG.CFG18.CFG18.C_G_OP_EXT;
  pcap_config.C_G_EN = pcap_nvram.CFG.CFG18.CFG18.C_G_EN;

  pcap_config.C_G_OP_VU = pcap_nvram.CFG.CFG19.CFG19.C_G_OP_VU;
  pcap_config.C_G_OP_ATTN = pcap_nvram.CFG.CFG19.CFG19.C_G_OP_ATTN;
  pcap_config.C_G_TIME = pcap_nvram.CFG.CFG19.CFG19.C_G_TIME;

  pcap_config.R_CY = pcap_nvram.CFG.CFG20.CFG20.R_CY;
  pcap_config.C_G_OP_TR = pcap_nvram.CFG.CFG20.CFG20.C_G_OP_TR;

  pcap_config.R_TRIG_PREDIV = (pcap_nvram.CFG.CFG22.CFG22.R_TRIG_PREDIV_HIGH << 8) |\
                               pcap_nvram.CFG.CFG21.CFG21.R_TRIG_PREDIV_LOW;


  pcap_config.R_TRIG_SEL = pcap_nvram.CFG.CFG22.CFG22.R_TRIG_SEL;
  pcap_config.R_AVRG = pcap_nvram.CFG.CFG22.CFG22.R_AVRG;

  pcap_config.R_PORT_EN = pcap_nvram.CFG.CFG23.CFG23.R_PORT_EN;
  pcap_config.R_PORT_EN_IMES = pcap_nvram.CFG.CFG23.CFG23.R_PORT_EN_IMES;
  pcap_config.R_PORT_EN_IREF = pcap_nvram.CFG.CFG23.CFG23.R_PORT_EN_IREF;
  pcap_config.R_FAKE = pcap_nvram.CFG.CFG23.CFG23.R_FAKE;
  pcap_config.R_STARTONPIN = pcap_nvram.CFG.CFG23.CFG23.R_STARTONPIN;

  pcap_config.DSP_MOFLO_EN = pcap_nvram.CFG.CFG27.CFG27.DSP_MOFLO_EN;
  pcap_config.DSP_SPEED = pcap_nvram.CFG.CFG27.CFG27.DSP_SPEED;
  pcap_config.PG0xPG2 = pcap_nvram.CFG.CFG27.CFG27.PG0xPG2;
  pcap_config.PG1xPG3 = pcap_nvram.CFG.CFG27.CFG27.PG1xPG3;

  pcap_config.WD_DIS = pcap_nvram.CFG.CFG28.CFG28.WD_DIS;

  pcap_config.DSP_STARTONPIN = pcap_nvram.CFG.CFG29.CFG29.DSP_STARTONPIN;
  pcap_config.DSP_FF_IN = pcap_nvram.CFG.CFG29.CFG29.DSP_FF_IN;

  pcap_config.PG5_INTN_EN = pcap_nvram.CFG.CFG30.CFG30.PG5_INTN_EN;
  pcap_config.PG4_INTN_EN = pcap_nvram.CFG.CFG30.CFG30.PG4_INTN_EN;
  pcap_config.DSP_START_EN = pcap_nvram.CFG.CFG30.CFG30.DSP_START_EN;

  pcap_config.PI1_TOGGLE_EN = pcap_nvram.CFG.CFG31.CFG31.PI1_TOGGLE_EN;
  pcap_config.PI0_TOGGLE_EN = pcap_nvram.CFG.CFG31.CFG31.PI0_TOGGLE_EN;
  pcap_config.PI0_RES = pcap_nvram.CFG.CFG31.CFG31.PI0_RES;
  pcap_config.PI0_PDM_SEL = pcap_nvram.CFG.CFG31.CFG31.PI0_PDM_SEL;
  pcap_config.PI0_CLK_SEL = pcap_nvram.CFG.CFG31.CFG31.PI0_CLK_SEL;

  pcap_config.PI1_RES = pcap_nvram.CFG.CFG32.CFG32.PI1_RES;
  pcap_config.PI1_PDM_SEL = pcap_nvram.CFG.CFG32.CFG32.PI1_PDM_SEL;
  pcap_config.PI1_CLK_SEL = pcap_nvram.CFG.CFG32.CFG32.PI1_CLK_SEL;

  pcap_config.PG_DIR_IN = pcap_nvram.CFG.CFG33.CFG33.PG_DIR_IN;
  pcap_config.PG_PU = pcap_nvram.CFG.CFG33.CFG33.PG_PU;

  pcap_config.INT_TRIG_BG = pcap_nvram.CFG.CFG34.CFG34.INT_TRIG_BG;
  pcap_config.DSP_TRIG_BG = pcap_nvram.CFG.CFG34.CFG34.DSP_TRIG_BG;
  pcap_config.BG_PERM = pcap_nvram.CFG.CFG34.CFG34.BG_PERM;
  pcap_config.AUTOSTART = pcap_nvram.CFG.CFG34.CFG34.AUTOSTART;

  pcap_config.CDC_GAIN_CORR = pcap_nvram.CFG.CFG35.CFG35.CDC_GAIN_CORR;
  pcap_config.BG_TIME = pcap_nvram.CFG.CFG38.CFG38.BG_TIME;

  pcap_config.PULSE_SEL0 = pcap_nvram.CFG.CFG39.CFG39.PULSE_SEL0;
  pcap_config.PULSE_SEL1 = pcap_nvram.CFG.CFG39.CFG39.PULSE_SEL1;

  pcap_config.C_SENSE_SEL = pcap_nvram.CFG.CFG40.CFG40.C_SENSE_SEL;

  pcap_config.R_SENSE_SEL = pcap_nvram.CFG.CFG41.CFG41.R_SENSE_SEL;

  pcap_config.ALARM1_SELECT = pcap_nvram.CFG.CFG42.CFG42.ALARM1_SELECT;
  pcap_config.ALARM0_SELECT = pcap_nvram.CFG.CFG42.CFG42.ALARM0_SELECT;
  pcap_config.EN_ASYNC_READ = pcap_nvram.CFG.CFG42.CFG42.EN_ASYNC_READ;
  pcap_config.HS_MODE_SEL = pcap_nvram.CFG.CFG42.CFG42.HS_MODE_SEL;
  pcap_config.R_MEDIAN_EN = pcap_nvram.CFG.CFG42.CFG42.R_MEDIAN_EN;
  pcap_config.C_MEDIAN_EN = pcap_nvram.CFG.CFG42.CFG42.C_MEDIAN_EN;

  pcap_config.RUNBIT = pcap_nvram.CFG.CFG47.CFG47.RUNBIT;

  pcap_config.MEM_LOCK = pcap_nvram.CFG.CFG48.CFG48.MEM_LOCK;

  pcap_config.SERIAL_NUMBER = (pcap_nvram.CFG.CFG50.CFG50.SERIAL_NUMBER_HIGH << 8) |\
                               pcap_nvram.CFG.CFG49.CFG49.SERIAL_NUMBER_LOW;


  pcap_config.MEM_CTRL = pcap_nvram.CFG.CFG54.CFG54.MEM_CTRL;

  return pcap_config;
  
};

void PCAP04::print_nvram(){
  int s = 0;
  unsigned char* cfg_p;

  for (int i = 0; i < PCAP_NVRAM_SIZE; i++){
    delay(50);
    if (i < PCAP_NVRAM_MAX_INDEX_FW)
    {
      s = i;
      if (pcap_nvram.FW.data[s] <= 0x0F){ 
        Serial.print("0");Serial.print(pcap_nvram.FW.data[s],HEX);Serial.print(" ");
      }else{
        Serial.print(pcap_nvram.FW.data[s],HEX);Serial.print(" ");
      }

    }
    else if (i >= PCAP_NVRAM_MAX_INDEX_FW && i < PCAP_NVRAM_MAX_INDEX_FW_CAL0)
    {
      s = i - PCAP_NVRAM_MAX_INDEX_FW_CAL0;

      if (pcap_nvram.FW_CAL0.data[s] <= 0x0F){ 
        Serial.print("0");Serial.print(pcap_nvram.FW_CAL0.data[s],HEX);Serial.print(" ");
      }else{
        Serial.print(pcap_nvram.FW_CAL0.data[s],HEX);Serial.print(" ");
      }
    }
    else if (i >= PCAP_NVRAM_MAX_INDEX_FW_CAL0 && i < PCAP_NVRAM_MAX_INDEX_FW_CAL1)
    {
      s = i - PCAP_NVRAM_MAX_INDEX_FW_CAL0;

      if (pcap_nvram.FW_CAL1.data[s] <= 0x0F){ 
        Serial.print("0");Serial.print(pcap_nvram.FW_CAL1.data[s],HEX);Serial.print(" ");
      }else{
        Serial.print(pcap_nvram.FW_CAL1.data[s],HEX);Serial.print(" ");
      }
    }
    else if (i >= PCAP_NVRAM_MAX_INDEX_FW_CAL1 && i < PCAP_NVRAM_SIZE)
    {
      s = i - PCAP_NVRAM_MAX_INDEX_FW_CAL1;

      cfg_p = (unsigned char *)(&pcap_nvram.CFG.CFG0 + s);

      if (*cfg_p <= 0x0F){ 
        Serial.print("0");Serial.print(*cfg_p,HEX);Serial.print(" ");
      }
      else{
        Serial.print(*cfg_p, HEX); Serial.print(" ");
      }
    }
    if ((i+1)%16 == 0){
      Serial.print(i);
      Serial.println();
    }
  }
}

void PCAP04::print_config(){
  int s = 0;
  unsigned char* cfg_p = nullptr;

  // Serial.println((long long)&pcap_nvram,HEX);
  // Serial.println((long long)&pcap_nvram.CFG,HEX);

  for (uint32_t i = PCAP_NVRAM_MAX_INDEX_FW_CAL1; i < PCAP_NVRAM_SIZE; i++){
    delay(50);
    
    s = i - (PCAP_NVRAM_MAX_INDEX_FW_CAL1);

    cfg_p = (unsigned char *)(&pcap_nvram.CFG.CFG0 + s);

    // Serial.print("i, s :");Serial.print(i);Serial.print(", ");Serial.println(s);
    // Serial.print("&pcap_nvram.CFG.CFG0 + s:");Serial.println((long long)(&pcap_nvram.CFG.CFG0 + s),HEX);

    if (*cfg_p <= 0x0F){ 
      Serial.print("0");Serial.print(*cfg_p,HEX);Serial.print(" ");
    }
    else{
      Serial.print(*cfg_p, HEX); Serial.print(" ");
    }
    
    if ((i+1)%4 == 0){
      Serial.print(i);
      Serial.println();
    }
  }
}
void PCAP04::writeall_nvram(){

  // Serial.println("writeall_nvram start");
  for (size_t addr = 0; addr < PCAP_NVRAM_SIZE; addr++)
  {
    if (addr != PCAP_NVRAM_RUNBIT_INDEX){
      write_nvram(addr);
    }
  }
  write_nvram(PCAP_NVRAM_RUNBIT_INDEX);

  // Serial.println("writeall_nvram end");
};

void PCAP04::write_nvram(unsigned short addr){
  static size_t s = 0;
  static unsigned char * cfg_p = nullptr;

  // Serial.println("write_nvram start");

  SPI.beginTransaction(pcap_spi_settings);

  if ((addr >= 0) && (addr < PCAP_NVRAM_SIZE))
  {
    wr_mem.nvram.addr = 0xFFFF & addr;
    wr_mem.nvram.data = 0;

    if (wr_mem.nvram.addr >= 0 && wr_mem.nvram.addr < PCAP_NVRAM_MAX_INDEX_FW)
    {
      s = wr_mem.nvram.addr;
      wr_mem.nvram.data = pcap_nvram.FW.data[s];
    }
    else if (wr_mem.nvram.addr >= PCAP_NVRAM_MAX_INDEX_FW && wr_mem.nvram.addr < PCAP_NVRAM_MAX_INDEX_FW_CAL0)
    {
      s = wr_mem.nvram.addr - PCAP_NVRAM_MAX_INDEX_FW;
      wr_mem.nvram.data = pcap_nvram.FW_CAL0.data[s];
    }
    else if (wr_mem.nvram.addr >= PCAP_NVRAM_MAX_INDEX_FW_CAL0 && wr_mem.nvram.addr < PCAP_NVRAM_MAX_INDEX_FW_CAL1)
    {
      s = wr_mem.nvram.addr - PCAP_NVRAM_MAX_INDEX_FW_CAL0;
      wr_mem.nvram.data = pcap_nvram.FW_CAL1.data[s];
    }
    else if (wr_mem.nvram.addr >= PCAP_NVRAM_MAX_INDEX_FW_CAL1 && wr_mem.nvram.addr < PCAP_NVRAM_SIZE)
    {

      s = wr_mem.nvram.addr - PCAP_NVRAM_MAX_INDEX_FW_CAL1;

      cfg_p = (unsigned char*)(&pcap_nvram.CFG.CFG0 + s);
      wr_mem.nvram.data = *cfg_p;
      
      // Serial.print("&(pcap_nvram.CFG.CFG0 + s):"); Serial.println((unsigned long long)(&pcap_nvram.CFG.CFG0 + s),HEX);
    }

    spi_transmit(wr_mem.opcode);

  }
  else
  {
    Serial.print("write nvram address not in range [0-1023] :");
    Serial.println(addr);
  }
  SPI.endTransaction();
  // Serial.println("write_nvram end");
};

pcap_results_t PCAP04::get_results(){
  // Serial.println("get_results start");
  
  static unsigned int decimal_part = 0;
  static float decimal_part_f = 0;
  static unsigned int integer_part = 0;

  readall_result();

  if (pcap_measurement_mode == STANDARD){

    // Serial.println((unsigned int)pcap_results_regs.RES0.REGVAL, HEX);
    // Serial.println((unsigned int)pcap_results_regs.RES0.REGVAL, BIN);
    // Serial.println((unsigned int)(pcap_results_regs.RES0.REGVAL >> 27),BIN);
    // Serial.println((unsigned int)(pcap_results_regs.RES0.REGVAL & 0x07FFFFFF),BIN);
    // Serial.println((unsigned int)(pcap_results_regs.RES0.REGVAL & 0x07FFFFFF));
    decimal_part = pcap_results_regs.RES0.REGVAL & 0x07FFFFFF;
    integer_part = pcap_results_regs.RES0.REGVAL >> 27;
    decimal_part_f =  (float)(decimal_part/134217727.0f)*0.9999995f;
    // Serial.print(integer_part); Serial.print(" - "); Serial.print(decimal_part); Serial.print(" - ");  Serial.print(decimal_part_f,7); Serial.print(" - ");
    pcap_results.C0_over_CREF = integer_part  + decimal_part_f;

    decimal_part = pcap_results_regs.RES1.REGVAL & 0x07FFFFFF;
    integer_part = pcap_results_regs.RES1.REGVAL >> 27;
    decimal_part_f =  (float)(decimal_part/134217727.0f)*0.9999995f;
    // Serial.print(integer_part); Serial.print(" - "); Serial.print(decimal_part); Serial.print(" - ");  Serial.print(decimal_part_f,7); Serial.print(" - ");
    pcap_results.C1_over_CREF = integer_part + decimal_part_f;

    decimal_part = pcap_results_regs.RES2.REGVAL & 0x07FFFFFF;
    integer_part = pcap_results_regs.RES2.REGVAL >> 27;
    decimal_part_f =  (float)(decimal_part/134217727.0f)*0.9999995f;
    // Serial.print(integer_part); Serial.print(" - "); Serial.print(decimal_part); Serial.print(" - ");  Serial.print(decimal_part_f,7); Serial.print(" - ");
    pcap_results.C2_over_CREF = integer_part + decimal_part_f;

    decimal_part = pcap_results_regs.RES3.REGVAL & 0x07FFFFFF;
    integer_part = pcap_results_regs.RES3.REGVAL >> 27;
    decimal_part_f =  (float)(decimal_part/134217727.0f)*0.9999995f;
    // Serial.print(integer_part); Serial.print(" - "); Serial.print(decimal_part); Serial.print(" - ");  Serial.print(decimal_part_f,7); Serial.print(" - ");
    pcap_results.C3_over_CREF = integer_part + decimal_part_f;

    decimal_part = pcap_results_regs.RES4.REGVAL & 0x07FFFFFF;
    integer_part = pcap_results_regs.RES4.REGVAL >> 27;
    decimal_part_f =  (float)(decimal_part/134217727.0f)*0.9999995f;
    // Serial.print(integer_part); Serial.print(" - "); Serial.print(decimal_part); Serial.print(" - ");  Serial.print(decimal_part_f,7); Serial.print(" - ");
    pcap_results.C4_over_CREF = integer_part + decimal_part_f;

    decimal_part = pcap_results_regs.RES5.REGVAL & 0x07FFFFFF;
    integer_part = pcap_results_regs.RES5.REGVAL >> 27;
    decimal_part_f =  (float)(decimal_part/134217727.0f)*0.9999995f;
    // Serial.print(integer_part); Serial.print(" - "); Serial.print(decimal_part); Serial.print(" - ");  Serial.print(decimal_part_f,7); Serial.print(" - ");
    pcap_results.C5_over_CREF = integer_part + decimal_part_f;

    decimal_part = pcap_results_regs.RES6.REGVAL & 0x07FFFFFF;
    integer_part = pcap_results_regs.RES6.REGVAL >> 27;
    decimal_part_f =  (float)(decimal_part/134217727.0f)*0.9999995f;
    // Serial.print(integer_part); Serial.print(" - "); Serial.print(decimal_part); Serial.print(" - ");  Serial.print(decimal_part_f,7); Serial.print(" - ");
    pcap_results.PT1_over_PTREF = integer_part + decimal_part_f;

    decimal_part = pcap_results_regs.RES7.REGVAL & 0x07FFFFFF;
    integer_part = pcap_results_regs.RES7.REGVAL >> 27;
    decimal_part_f =  (float)(decimal_part/134217727.0f)*0.9999995f;
    // Serial.print(integer_part); Serial.print(" - "); Serial.print(decimal_part); Serial.print(" - ");  Serial.print(decimal_part_f,7); Serial.print(" - ");
    pcap_results.PTInternal_over_PTREF = integer_part + decimal_part_f;
  }

  return pcap_results;
  
  // Serial.println("get_results end");

};

pcap_status_t PCAP04::get_status(bool print_status){

  readall_status();
  
  static pcap_status_t status;

  status.RUNBIT =  pcap_results_regs.STATUS0.STATUS0.RUNBIT;
  status.CDC_ACTIVE =  pcap_results_regs.STATUS0.STATUS0.CDC_ACTIVE;
  status.RDC_READY =  pcap_results_regs.STATUS0.STATUS0.RDC_READY;
  status.AUTOBOOT_BUSY = pcap_results_regs.STATUS0.STATUS0.AUTOBOOT_BUSY;
  status.POR_CDC_DSP_COLL =  pcap_results_regs.STATUS0.STATUS0.POR_CDC_DSP_COLL;
  status.POR_FLAG_WDOG = pcap_results_regs.STATUS0.STATUS0.POR_FLAG_WDOG;

  status.COMB_ERR = pcap_results_regs.STATUS1.STATUS1.COMB_ERR;
  status.ERR_OVERFL = pcap_results_regs.STATUS1.STATUS1.ERR_OVFL;
  status.MUP_ERR = pcap_results_regs.STATUS1.STATUS1.MUP_ERR;
  status.RDC_ERR = pcap_results_regs.STATUS1.STATUS1.RDC_ERR;

  status.C_PORT_ERR0 = pcap_results_regs.STATUS2.STATUS2.C_PORTERR0;
  status.C_PORT_ERR1 = pcap_results_regs.STATUS2.STATUS2.C_PORTERR1;
  status.C_PORT_ERR2 = pcap_results_regs.STATUS2.STATUS2.C_PORTERR2;
  status.C_PORT_ERR3 = pcap_results_regs.STATUS2.STATUS2.C_PORTERR3;
  status.C_PORT_ERR4 = pcap_results_regs.STATUS2.STATUS2.C_PORTERR4;
  status.C_PORT_ERR5 = pcap_results_regs.STATUS2.STATUS2.C_PORTERR5;
  status.C_PORT_ERR_INT = pcap_results_regs.STATUS2.STATUS2.C_PORTERR_INT;

  if (print_status){
    Serial.print(pcap_results_regs.STATUS0.REGVAL, BIN);Serial.print(" ");
    Serial.print(pcap_results_regs.STATUS1.REGVAL, BIN);Serial.print(" ");
    Serial.print(pcap_results_regs.STATUS2.REGVAL, BIN);Serial.print(" ");  
  }


  return status;

};

void PCAP04::validate_nvram(){

  static unsigned char* nvram_p;
  static unsigned char* nvram_mirror_p;

  readall_nvram();

  Serial.print(sizeof(__PCAP_NVRAM_T)); Serial.print(" ");
  Serial.print(sizeof(__PCAP_CONFIG_REGS_T)); Serial.print(" ");
  Serial.print(sizeof(__PCAP_FW_T)); Serial.println(" ");

  Serial.print("(&pcap_nvram)"); Serial.println((long long)(&pcap_nvram),HEX);
  Serial.print("(&pcap_nvram.FW)"); Serial.println((long long)(&pcap_nvram.FW),HEX);

  nvram_p = &pcap_nvram.FW.data[0];
  nvram_mirror_p = &pcap_nvram_mirror.FW.data[0];

  for (size_t addr = 0; addr < PCAP_NVRAM_SIZE; addr++)
  {
    //delay(50);

    //Serial.print("values (mcu, pcap) :(");Serial.print(*nvram_p); Serial.print(", ");Serial.print(*nvram_mirror_p);Serial.print(").");

    if (*nvram_p != *nvram_mirror_p){
      Serial.print("nvram different at address :");Serial.print(addr); 
      Serial.print(" with values (mcu, pcap) :(");Serial.print(*nvram_p); Serial.print(", ");Serial.print(*nvram_mirror_p);Serial.print(").");
      delay(1000);
    }
    nvram_p++;
    nvram_mirror_p++;
  } 


};

void PCAP04::reset_pcap_dsp(){

  write_config(47,0);
  delay(100);
  write_config(47,1);
  delay(100);

};
#endif