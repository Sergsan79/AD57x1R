#include "AD57x1R.h"
#include "Arduino.h"
#include "SPI.h"

    uint8_t _Part_DAC;
    int     _ALERT;
    int     _SYNC;
    int     _RESET;
    int     _CLEAR;
    int     _LDAC;
    
AD57x1R::AD57x1R (uint8_t Part_DAC, int ALERT, int SYNC, int RESET, int CLEAR, int LDAC)
{
    pinMode (ALERT, INPUT); //Активное Низкое Предупреждение. Этот штифт утверждается низким, когда температура матрицы превышает приблизительно 150 ° C, или
                            //когда происходит короткое замыкание на выходе или отключение питания. Этот вывод также устанавливается на низкий уровень при включении питания, полный
                            //программный сброс или аппаратный сброс, для которого запись в регистр управления подтверждает высокий вывод.
    pinMode (SYNC, OUTPUT); //Активный низкий вход синхронизации. Этот вывод является сигналом синхронизации кадра для последовательного интерфейса. Пока
                            //Низкий уровень синхронизации, данные передаются по падающему фронту SCLK. Данные фиксируются на переднем крае SYNC
    pinMode (RESET, OUTPUT);
    pinMode (CLEAR, OUTPUT);  //Падающая кромка Очистить ввод. Утверждение этого вывода устанавливает регистр ЦАП в нулевой, средний или полномасштабный код
                              //(выбирается пользователем) и обновляет вывод ЦАП. Этот штифт можно оставить плавающим, потому что есть внутренний
                              //подтягивающий резистор
    pinMode (LDAC, OUTPUT);   //LDAC Загрузка ЦАП. Этот логический вход обновляет регистр ЦАП и, следовательно, аналоговый выход. Когда связаны
                              //постоянно низкий, регистр ЦАП обновляется при обновлении входного регистра. Если LDAC поддерживается на высоком уровне
                              //во время записи во входной регистр выходной регистр ЦАП не обновляется, а выходной ЦАП обновляется
                              //сдерживается до падения края LDAC. Этот вывод можно оставить плавающим, поскольку имеется внутренний подтягивающий резистор.
    
    digitalWrite(LDAC, 1);
    digitalWrite(SYNC, 1);
    digitalWrite(RESET, 1);
    digitalWrite(CLEAR, 1);

    


  _Part_DAC = Part_DAC;
  _ALERT = ALERT;
  _SYNC  = SYNC;
  _RESET = RESET;
  _CLEAR = CLEAR;
  _LDAC  = LDAC;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

AD57x1R::AD57x1R_update_dac_LDAC() // LDAC Синхронизация
{
    digitalWrite(_LDAC, 0);
    digitalWrite(_LDAC, 1);
}

int32_t AD57x1R::AD57x1R_write_input_register( uint16_t dac_data)//Запись в регистр ввода (без обновления) ADD 01
{
  uint16_t reg_data;
  uint32_t ret = 0;

    reg_data = AD57x1R_DATA(dac_data);
    
    ret = AD57x1R_write(CMD_WR_TO_INPUT_REG, reg_data);
    return ret;
}

int32_t AD57x1R::AD57x1R_update_dac_register() // Обновить регистр ЦАП из входного регистра ADD 02
{
  return AD57x1R_write(CMD_UPDATE_DAC_REG, 0);
}

int32_t AD57x1R::AD57x1R_write_update_dac_register( uint16_t dac_data)//Написать и обновить регистр ЦАП ADD 03
{
  uint16_t reg_data;
  uint32_t ret = 0;
  
    reg_data = AD57x1R_DATA(dac_data);

    ret = AD57x1R_write(CMD_WR_UPDATE_DAC_REG, reg_data);
    return ret;
}

int32_t AD57x1R::AD57x1R_write_control_register (struct control_register dev)//Написать в контрольный регистр ADD 04
{
  uint32_t ret = 0;
  ret = AD57x1R_write(CMD_WR_CTRL_REG,(AD57x1R_CTRL_CV(dev.CV) | dev.OVR 
                                                                | dev.B2C 
                                                                | dev.ETS 
                                                                | dev.IRO 
                                                                | AD57x1R_CTRL_PV (dev.PV) 
                                                                | AD57x1R_CTRL_RA (dev.RA)));
  return ret;
}


int32_t AD57x1R::AD57x1R_software_data_reset()// Сброс данных программного обеспечения Сброс DAC ADD 07
{
  return AD57x1R_write(CMD_SW_DATA_RESET, 0);
}

int32_t AD57x1R::AD57x1R_set_daisy_chain_en_dis(bool en_dis) //Отключить функцию гирляндной цепи ADD 09
{

  return AD57x1R_write(CMD_DIS_DAISY_CHAIN, AD57x1R_DIS_DAISY_CHAIN_DDC(!en_dis));
}

////////////////////////////========================/////////////////////////////
int32_t AD57x1R::AD57x1R_register_readback(uint8_t reg) //ADD 10 ADD 11 ADD 12
{
  uint8_t reg_addr;
  int32_t ret;



  switch (reg) {
  case AD57x1R_REG_INPUT:
    reg_addr = CMD_RD_INPUT_REG;
    break;
  case AD57x1R_REG_DAC:
    reg_addr = CMD_RD_DAC_REG;
    break;
  case AD57x1R_REG_CTRL:
    reg_addr = CMD_RD_CTRL_REG;
    break;
  default:
    return -1;
  }

  ret = AD57x1R_read(reg_addr);

  ret = AD57x1R_read(reg_addr);

  return ret;
}
int32_t AD57x1R_read(
         uint8_t reg_addr_cmd)
{
  uint8_t data[3];

  int32_t ret;

  data[0] = reg_addr_cmd;
  data[1] = 0;
  data[2] = 0;
  ret = spi_write(data, 3);
  return ret;
}
//=====================//////////////============================

int32_t AD57x1R::AD57x1R_software_full_reset()// ADD 15
{
  return AD57x1R_write(CMD_SW_FULL_RESET, 0);
}
////////////////////////////////////////////////////////////////////////////////////////////////

bool AD57x1R::AD57x1R_get_brownout_condition() //обнаружены условия отключения.
{
  return  (((AD57x1R_register_readback(AD57x1R_REG_CTRL)) & AD57x1R_CTRL_BO)>>11);
}

bool AD57x1R::AD57x1R_get_short_circuit_condition() //обнаружено состояние короткого замыкания.
{
  return  (((AD57x1R_register_readback(AD57x1R_REG_CTRL)) & AD57x1R_CTRL_SC)>>12);
}

uint32_t AD57x1R_write(
          uint8_t reg_addr_cmd,
          uint16_t reg_data)
{
  uint8_t data[3];
  uint32_t ret = 0;

  data[0] = reg_addr_cmd;
  data[1] = (reg_data & 0xFF00) >> 8;
  data[2] = (reg_data & 0x00FF) >> 0;
  ret = spi_write(data, 3);
  return ret;
}

uint32_t spi_write(
          uint8_t *reg_data,
          uint8_t reg_ind)
{

uint32_t ret = 0;
uint32_t ret_buf = 0;
uint8_t i;
digitalWrite(_SYNC, 0);
  for(i=0; i<reg_ind; i++) {
          ret_buf = SPI.transfer (reg_data [i]);
          ret = ret | (ret_buf<<(8*((reg_ind-1)-i)));                
  }
digitalWrite(_SYNC, 1);
  return ret;
}

uint16_t AD57x1R_DATA (uint16_t dac_data){
  uint16_t reg_data = 0; 
    switch ( _Part_DAC) {
                          case AD5721R:
                          reg_data = AD5721R_DATA(dac_data);
                          break;
                          case AD5761R:
                          reg_data = AD5761R_DATA(dac_data);
                          break;
                        }
return reg_data;
}                        
