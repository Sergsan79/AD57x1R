#ifndef AD57x1R_H_
#define AD57x1R_H_

#include "Arduino.h"
#include "SPI.h"

/* Input Shift Register Format */
#define AD57x1R_INPUT_ZERO_BIT   (1 << 20)
#define AD57x1R_INPUT_ADDR_CMD(x) (((x) & 0xF) << 16)
#define AD57x1R_INPUT_DATA(x)   (((x) & 0xFFFF) << 0)

#define AD5761R_DATA(x)      (((x) & 0xFFFF) << 0)
#define AD5721R_DATA(x)     (((x) & 0xFFF) << 4)

/* Input Shift Register Commands */
#define CMD_NOP       0x0
#define CMD_WR_TO_INPUT_REG   0x1
#define CMD_UPDATE_DAC_REG    0x2
#define CMD_WR_UPDATE_DAC_REG   0x3
#define CMD_WR_CTRL_REG     0x4
#define CMD_NOP_ALT_1     0x5
#define CMD_NOP_ALT_2     0x6
#define CMD_SW_DATA_RESET   0x7
#define CMD_RESERVED      0x8
#define CMD_DIS_DAISY_CHAIN   0x9
#define CMD_RD_INPUT_REG    0xA
#define CMD_RD_DAC_REG      0xB
#define CMD_RD_CTRL_REG     0xC
#define CMD_NOP_ALT_3     0xD
#define CMD_NOP_ALT_4     0xE
#define CMD_SW_FULL_RESET   0xF

/* Control Register Format */
#define AD57x1R_CTRL_SC     (1 << 12)   // RO
#define AD57x1R_CTRL_BO     (1 << 11)   // RO
#define AD57x1R_CTRL_CV(x)    (((x) & 0x3) << 9)  // RW
#define AD57x1R_CTRL_OVR    (1 << 8)    // RW
#define AD57x1R_CTRL_B2C    (1 << 7)    // RW
#define AD57x1R_CTRL_ETS    (1 << 6)    // RW
#define AD57x1R_CTRL_IRO    (1 << 5)    // RW
#define AD57x1R_CTRL_PV(x)    (((x) & 0x3) << 3)  // RW
#define AD57x1R_CTRL_RA(x)    (((x) & 0x7) << 0)  // RW

/* Disable Daisy-Chain Register Format */
#define AD57x1R_DIS_DAISY_CHAIN_DDC(x)  (((x) & 0x1) << 0)


/*
0 0 0 0 Нет операции
0 0 0 1 Запись в регистр ввода (без обновления)
0 0 1 0 Обновить регистр ЦАП из входного регистра
0 0 1 1 Написать и обновить регистр ЦАП
0 1 0 0 Запись в контрольный регистр
0 1 0 1 Нет операции
0 1 1 0 Нет операции
0 1 1 1 Сброс данных программного обеспечения
1 0 0 0 Зарезервировано
1 0 0 1 Отключить функцию гирляндной цепи
1 0 1 0 Считывание входного регистра
1 0 1 1 Считывание регистра ЦАП
1 1 0 0 регистр управления обратным чтением
1 1 0 1 Нет операции
1 1 1 0 Нет операции
1 1 1 1 Программный полный сброс

CV [1: 0] CLEAR выбор напряжения.
      00: нулевая шкала
      01: средний
      10, 11: полная шкала
OVR 5% превышения.
    0: 5% отключен
    1: 5% разрешено превышение диапазона
B2C Биполярный диапазон.
    0: вход ЦАП для биполярного выходного диапазона имеет прямое двоичное кодирование
    1: вход ЦАП для биполярного выходного диапазона кодируется двумя комплементами
ETS Предупреждение о тепловом отключении. Предупреждение может работать неправильно, если устройство включается при температурных условиях> 150 ° C
(больше, чем максимальная оценка устройства).
     0: внутреннее цифровое питание не отключается, если температура матрицы превышает 150 ° C.
     1: внутреннее цифровое питание отключается, если температура матрицы превышает 150 ° C.
IRO Внутренняя ссылка.
     0: внутренняя ссылка отключена
     1: внутренняя ссылка включена
PV [1: 0] Напряжение питания.
      00: нулевая шкала
      01: средний
      10, 11: полная шкала
RA [2: 0] Выходной диапазон. Перед настройкой выходного диапазона устройство необходимо перезагрузить.
     000: от -10 В до +10 В
     001: от 0 до +10 В
     010: от -5 В до +5 В
     011: от 0 до 5 В
    100: -2,5 В до +7,5 В
    101: -3 В до +3 В
    110: 0 до 16 В
    111: 0 В до 20 В*/

    
class AD57x1R
{
  public:
    AD57x1R(uint8_t Part_DAC, int ALERT, int SYNC, int RESET, int CLEAR, int LDAC);
    int32_t AD57x1R_write_control_register (struct control_register dev);           // Написать в контрольный регистр ADD 04
    int32_t AD57x1R_write_input_register( uint16_t dac_data);                       // Запись в регистр ввода (без обновления) ADD 01
    int32_t AD57x1R_write_update_dac_register( uint16_t dac_data);                  // Написать и обновить регистр ЦАП ADD 03
    int32_t AD57x1R_update_dac_register();                                          // Обновить регистр ЦАП из входного регистра ADD 02
    AD57x1R_update_dac_LDAC();                                                      // LDAC Синхронизация
    int32_t AD57x1R_software_data_reset();                                          // Сброс DAC
    int32_t AD57x1R_set_daisy_chain_en_dis(bool en_dis);                            // Отключить функцию гирляндной цепи ADD 09
    int32_t AD57x1R_register_readback(uint8_t reg);                                 // ADD 10 ADD 11 ADD 12
    int32_t AD57x1R_software_full_reset();                                          // Сброс данных программного обеспечения Сброс DAC ADD 07
    bool AD57x1R_get_brownout_condition();                                          // обнаружены условия отключения.
    bool AD57x1R_get_short_circuit_condition();                                     // обнаружено состояние короткого замыкания.

    
  private:

          
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////                                                                                        ////////////
enum AD57x1R_type {
  AD5761R,
  AD5721R,
};
enum AD57x1R_reg {
  AD57x1R_REG_INPUT,
  AD57x1R_REG_DAC,
  AD57x1R_REG_CTRL,
};
enum AD57x1R_scale {
  AD57x1R_SCALE_ZERO,
  AD57x1R_SCALE_HALF,
  AD57x1R_SCALE_FULL,
};
enum AD57x1R_range {
  AD57x1R_RANGE_M_10V_TO_P_10V,
  AD57x1R_RANGE_0V_TO_P_10V,
  AD57x1R_RANGE_M_5V_TO_P_5V,
  AD57x1R_RANGE_0V_TO_P_5V,
  AD57x1R_RANGE_M_2V5_TO_P_7V5,
  AD57x1R_RANGE_M_3V_TO_P_3V,
  AD57x1R_RANGE_0V_TO_P_16V,
  AD57x1R_RANGE_0V_TO_P_20V,
};
struct control_register{
  int8_t CV;
  bool   OVR;
  bool   B2C;
  bool   ETS;
  bool   IRO;
  int8_t PV;
  int8_t RA;
};


uint32_t spi_write(
          uint8_t *reg_data,
          uint8_t reg_ind);
          
uint32_t AD57x1R_write(
          uint8_t reg_addr_cmd,
          uint16_t reg_data);
int32_t AD57x1R_read(
         uint8_t reg_addr_cmd); 
uint16_t AD57x1R_DATA (
         uint16_t dac_data);

#endif // AD57x1R_H_
