#include <Arduino.h>
#include <option1.hpp>
#include <string.h>
#include <Wire.h>

int SPI_select(int Slave);
int SPI_deselect();
int SPI_reset(int Slave);
int CAN1_silent();
int CAN1_not_silent();
int CAN2_silent();
int CAN2_not_silent();
int Status_LED_ON();
int Status_LED_OFF();
int learn_RFControl(int mode);
int GPIO_Exp_WriteRegister(int reg, int value);
int GPIO_Exp_ReadRegister(int reg);
int GPIO_Exp_WriteBit(int reg, int bit, int value);
int GPIO_Exp_ReadBit(int reg, int bit);




int SPI_select(int Slave){
/**
 * @brief Slaves selects the Component SPI wants to communicate with. Depending on Slave this function activates ChipSelect for corresponding Components
 * 
 * @param Slave Component to activate
 * 
 * @return 1 at Success, -1 at fail
 */
  if(Slave == DISPLAY)
  {
    digitalWrite(SPI_CS_DISPLAY_PIN, LOW);
    return SUCCESS;
  }
  else if (Slave == Flash)
  {
    digitalWrite(SPI_CS_FLASH_PIN, LOW);
    return SUCCESS;
  }
  else if (Slave == CAN2)
  {
    digitalWrite(SPI_CS_CAN2_PIN, LOW);
    return SUCCESS;
  }  
  else if (Slave == RFID)
  {
    int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 2, LOW);

    if(rv == ERROR){
      Serial.println("ChipSelect RFID failed");
      return ERROR;
    }

    return SUCCESS;
  }
  else
  {
    //Error in Defines
    return ERROR;
  }
}

int SPI_deselect(){
/**
 * @brief This functions Sets every CS to HIGH, no CS is active after this function
 * 
 * @return -1: I2C Write failed
 * @return 1: Bit Successful set
 */
  digitalWrite(SPI_CS_DISPLAY_PIN, HIGH);
  digitalWrite(SPI_CS_FLASH_PIN, HIGH);
  digitalWrite(SPI_CS_CAN2_PIN, HIGH);

  int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 2, HIGH);

  if(rv == ERROR){
    Serial.println("ChipSelect RFID could not be Reset");
    return ERROR;
  }

  return SUCCESS;

}

int SPI_reset(int Slave){
/**
 * @brief Resets given SPI Slave
 * 
 * @param Slave Component to be reset
 * 
 * @return 1 at Success and finished reset procedure, 0 if there is no Reset is pending, -1 at fail
 */

  static long wait_time_SPI_reset = 0;
  static bool reset_active = false;

  if(wait_time_SPI_reset == 0){
    wait_time_SPI_reset = millis() + 300;
  }
  long now = millis();

  // Check for active reset
  if(reset_active == false){

    // run once, if timer is now over
    // set SPI Reset Pins Active
    if(now < wait_time_SPI_reset and reset_active == false){
      reset_active = true;

      if(Slave == DISPLAY_LCD){
        digitalWrite(SPI_RST_LCD_PIN, LOW);// LOW active
        return 0;
      }
      else if(Slave == DISPLAY_TOUCH){
        digitalWrite(SPI_RST_TP_PIN, LOW); // LOW active
        return 0;
      }
      else if(Slave == RFID){
        int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 3, LOW); // LOW active
        if(rv == ERROR){
          Serial.println("Writing failed");
          return ERROR;
        }
        return 0;
      }
      else{  
        Serial.println("Unknown Slave Address");
        return ERROR;
      }
    }

    // Timer has expired, stop reset
    if(now >= wait_time_SPI_reset and reset_active == true){
      reset_active = false;

      if(Slave == DISPLAY_LCD){
        digitalWrite(SPI_RST_LCD_PIN, HIGH);// LOW active
        wait_time_SPI_reset = 0;
        return SUCCESS;
      }
      else if(Slave == DISPLAY_TOUCH){
        digitalWrite(SPI_RST_TP_PIN, HIGH); // LOW active
        wait_time_SPI_reset = 0;
        return SUCCESS;
      }
      else if(Slave == RFID){
        int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 3, HIGH); // LOW active
        if(rv == ERROR){
          Serial.println("Writing failed");
          return ERROR;
        }
        wait_time_SPI_reset = 0;
        return SUCCESS;
      }
      else{  
        Serial.println("Unknown Slave Address");
        return ERROR;
      }
    }

  }
  return 0;
  
}

int CAN1_silent(){
/**
 * @brief Sets the CAN-Transciever for CAN1 to silent, Transciever con only listen not send
 * 
 * @return 1 at Success, -1 at fail
 */
    // Write To PIN 6 on GPIO Expansion
  int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 6, HIGH);  // Silent on HIGh Pin

  if(rv == ERROR){
    Serial.println("Write failed");
    return ERROR;
  }

  return SUCCESS;
}

int CAN1_not_silent(){
/**
 * @brief Sets the CAN-Transciever for CAN1 back from silent, Transciever can now send and listen
 * 
 * @return 1 at Success, -1 at fail
 */
  // Write to Pin 6 on GPIO Expansion
  int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 6, LOW);  // Silent on HIGH Pin 

  if(rv == ERROR){
    Serial.println("Write failed");
    return ERROR;
  }

  return SUCCESS;
}

int CAN2_silent(){
/**
 * @brief Sets the CAN-Transciever for CAN2 to silent, Transciever con only listen not send
 * 
 * @return 1 at Success, -1 at fail
 */
  // Write to PIN 7 on GPIO Expansion
  int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 7, HIGH);  // Silent on HIGh Pin

  if(rv == ERROR){
    Serial.println("Write failed");
    return ERROR;
  }

  return SUCCESS;
}

int CAN2_not_silent(){
/**
 * @brief Sets the CAN-Transciever for CAN2 back from silent, Transciever can now send and listen
 * 
 * @return 1 at Success, -1 at fail
 */
  // Write to PIN 7 on GPIO Expansion
  int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 7, LOW);  // Silent on HIGh Pin

  if(rv == ERROR){
    Serial.println("Write failed");
    return ERROR;
  }

  return SUCCESS;
}

int Status_LED_ON(){
/**
 * @brief Switches the Status-LED to on
 * 
 * @return 1 at Success, -1 at fail
 */
  // Write to PIN 0 on GPIO Expansion
  int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 0, HIGH);

  if(rv == ERROR){
    Serial.println("Write failed");
    return ERROR;
  }

  return SUCCESS;
}

int Status_LED_OFF(){
/**
 * @brief Switches the Status-LED to off
 *  
 * @return 1 at Success, -1 at fail
 */
  // Write to PIN 0 on GPIO Expansion
  int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 0, LOW);

  if(rv == ERROR){
    Serial.println("Write failed");
    return ERROR;
  }

  return SUCCESS;
}

int learn_RFControl(int mode){
/**
 * @brief Activates diffrent learn modes for the RF Module like pairing the RemoteControl. Mode is specified via parameter mode
 * 
 * @param mode Difrent modes for learning and removing remotecontrols paired with the RF Module
 * 
 * @return mode at Success, -1 at fail
 */
  static int time_wait_Learn_RF = 0;
  static int ctr = 0;
  static bool signal_active = false;

  int rv = 0;
  
  // Check for valid mode
  if(mode >= 1 && mode <= 6){

    // Wait for Timer
    if(millis() < time_wait_Learn_RF && time_wait_Learn_RF > 0){
      return 0;
    }

    // Activate Signal
    if(signal_active = false){
      // Write to PIN 1 on GPIO B Expansion
      signal_active = true;

      // Set Timer
      if(mode <= 4){
        time_wait_Learn_RF = millis() + 100;
      }
      else{
        time_wait_Learn_RF = millis() + 3100;
      }

      // lowactive Signal activate
      rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW); 
      if(rv == ERROR){
        signal_active = false;
        time_wait_Learn_RF = 0;
        ctr = 0;
        return ERROR;
      }
      return 0;
    }

    // Deactivate Signal
    if(signal_active == true){
      signal_active = false;
      ctr++;

      // lowactive Signal reset
      rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
      if(rv == ERROR){
        time_wait_Learn_RF = 0;
        ctr = 0;
        return ERROR;
      }

      int cmp_mode = mode;
      if(cmp_mode >= 5){
        cmp_mode -= 4; 
      }
      if(ctr < cmp_mode){
        // Signal not complete, continue
        // Set Timer for Wait
        if(mode <= 4){
          time_wait_Learn_RF = millis() + 100;
        }
        else{
          time_wait_Learn_RF = millis() + 3100;
        }
        return 0;
      }
      else{
        // Signal complete, End Function
        time_wait_Learn_RF = 0;
        ctr = 0;
        return SUCCESS;
      }
    }
  }
}

/***************************

I²C Read and Write Functions

****************************/

int GPIO_Exp_WriteRegister(int reg, int value){

  // Check for Value out of Range
  if(value > 255 || value < 0){
    // Value out of Range
    Serial.println("Value out of Range");
    return ERROR;
  }

  if(reg<0x00 || (reg < 0x10 && reg > 0x0A) || reg > 0x1A){
    Serial.println("Register Address out of Range");
    return ERROR;
  }

  // Write Address and Write Bit to Buffer
  // I2C Adresse + Write Bit
  Wire.beginTransmission(GPIO_EXP_ADRESS << 1); 
  
  // Write register Address to Buffer
  int rv = Wire.write(reg); 
  if(rv <= 0){
    Serial.println("Write register Adress failed");
    return ERROR;
  }  

  // Write Data to Buffer
  rv = Wire.write(value);      // Write Data
  if(rv <= 0){
    Serial.println("Write data failed");
    return ERROR;
  }  

  // Send Buffer to Communication
  rv = Wire.endTransmission();
  if(rv != 0){
    Serial.println("Sending Bufferr to Communication failed");
    return ERROR;
  }
  return SUCCESS;
}

int GPIO_Exp_ReadRegister(int reg){

  // Check for Address out of Range
  if(reg<0x00 || (reg < 0x10 && reg > 0x0A) || reg > 0x1A){
    Serial.println("Register Address out of Range");
    return ERROR;
  }

  // Write Address and Write Bit to Buffer
  Wire.beginTransmission((GPIO_EXP_ADRESS << 1) || READ); // I2C Adresse + Read Bit

  // Write register address to Buffer
  int rv = Wire.write(reg);              // Registeradresse senden
  if(rv <= 0){ return ERROR;}

  // Send Buffer to Communication
  rv = Wire.endTransmission(false);  // Stop-Bedingung vermeiden (Repeated Start)
  if(rv != 0){ return ERROR;}

  // Write Address and Read bit to Buffer
  // Send Request for Register value
  Wire.requestFrom(GPIO_EXP_ADRESS, 1);

  // Read Register from Communication
  if (Wire.available()) {
    // Read from Receive Buffer
    return Wire.read(); // Return Byte
  }
  return ERROR; // Receive Buffer empty
}

int GPIO_Exp_WriteBit(int reg, int bit, int value){

  // Check for value in Range
  if(value > 1 || value < 0){
    Serial.println("Value must be binary");
    return ERROR;
  }
  else{
    Serial.println("Value is in Range");
  }
  
  // Check for Address out of Range
  if(reg<0x00 || (reg < 0x10 && reg > 0x0A) || reg > 0x1A){
    Serial.println("Register Address out of Range");
    return ERROR;
  }
  else{
    Serial.println("Register Address is in Range");
  }

  int rv = 0;
  // Read Data from Register
  int reg_value = GPIO_Exp_ReadRegister(reg);
  if(reg_value == ERROR){
    Serial.println("Register could not be read");
    return ERROR;
  }

  // Changes Bit in Register Data and write data to Buffer
  if(value == HIGH){
    rv = GPIO_Exp_WriteRegister(reg, reg_value | (1 << bit));
  }
  else if(value == LOW){
    rv = GPIO_Exp_WriteRegister(reg, reg_value & ~(1 << bit));
  }

  // Error handling
  if(rv == ERROR){
    Serial.println("Writing Failed");
    return ERROR;
  }
  
  return SUCCESS;  
}

int GPIO_Exp_ReadBit(int reg, int bit){

  /**
   * @brief Reads one single Bit from registers of the GPIO Expansion
   * 
   * @param reg: 8bit Register Address in Hex
   * @param bit: Bit position to be read
   * 
   * @return -1: Bit Value ot of Range
   * @return -1: Register Address out of range
   * @return -1: Read register failed
   * @return bit_value: Data read and processed successfully 
   */

   // Check for Bit Value out of Range
  if(bit > 7){
    Serial.println("Value out of Range");
    return ERROR;
  }

  // Check for Address out of Range
  if(reg<0x00 || (reg < 0x10 && reg > 0x0A) || reg > 0x1A){
    Serial.println("Register Address out of Range");
    return ERROR;
  }

  // Read Register from GPIO Port Expansion
  int value = GPIO_Exp_ReadRegister(reg);
  if(value == ERROR){
    Serial.println("Read Register Failed");
    return ERROR;
  }

  // mask and filter for single bit
  u_int8_t bit_value = value >> bit;
  bit_value &= 0x01;

  return bit_value;
}