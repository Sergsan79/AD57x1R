#include <SPI.h>
#include <ad57x1r.h>


//AD57x1R (Part_DAC, ALERT, SYNC, RESET, CLEAR, LDAC)
AD57x1R DAC (AD5721R, A5,  9,    4,     10,    8);


  control_register  WCR;
  
void setup() {

  WCR.CV = AD57x1R_SCALE_HALF;
  WCR.OVR = 0;
  WCR.B2C = 0;
  WCR.IRO =0;
  WCR.ETS = 0;
  WCR.PV = AD57x1R_SCALE_ZERO;
  WCR.RA = AD57x1R_RANGE_0V_TO_P_10V;


  Serial.begin(9600);
  
  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1)); //2-16MHz

  print_DAC();
  
    while (DAC.AD57x1R_get_brownout_condition()) DAC.AD57x1R_write_control_register (WCR); // Конфирурация регистра control_register
    
  print_DAC();

   int32_t test_reg1 = DAC.AD57x1R_write_input_register( 4095 );

  print_DAC(); 

    DAC.AD57x1R_update_dac_LDAC();

  print_DAC(); 

    int32_t test_reg2 = DAC. AD57x1R_write_update_dac_register ( 4095/2 );

  print_DAC();

}


void loop() {
  delay(1000);


}

void print_DAC(){
       int32_t test1 = DAC.AD57x1R_register_readback(AD57x1R_REG_INPUT); //ADD 10
     Serial.print("AD57x1R_REG_INPUT : ");
     Serial.println(test1 , BIN);
   
     int32_t test2 = DAC.AD57x1R_register_readback(AD57x1R_REG_DAC); //ADD 11
     Serial.print("AD57x1R_REG_DAC : ");
     Serial.println(test2 , BIN);
   
     int32_t test3 = DAC.AD57x1R_register_readback(AD57x1R_REG_CTRL); //ADD 12
     Serial.print("AD57x1R_REG_CTRL : ");
     Serial.println(test3 , BIN);

     Serial.println("");
}
