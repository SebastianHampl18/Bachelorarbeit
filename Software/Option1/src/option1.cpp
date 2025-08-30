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
      perror("ChipSelect RFID failed");
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
    perror("ChipSelect RFID could not be Reset");
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
      reset_active == true;

      if(Slave == DISPLAY_LCD){
        digitalWrite(27, HIGH);//TODO: Check for High or LOW active
        return 0;
      }
      else if(Slave == DISPLAY_TOUCH){
        digitalWrite(32, HIGH); //TODO: Check for High or LOW active
        return 0;
      }
      else if(Slave == RFID){
        int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 3, HIGH); //TODO: Check for High or LOW active
        if(rv == ERROR){
          perror("Writing failed");
          Serial.println("Writing failed");
          return ERROR;
        }
        return 0;
      }
      else{  
        perror("Unknown Slave Address");
        return ERROR;
      }
    }

    // Timer has expired, stop reset
    if(now >= wait_time_SPI_reset and reset_active == true){
      reset_active = false;

      if(Slave == DISPLAY_LCD){
        digitalWrite(27, LOW);//TODO: Check for High or LOW active
        wait_time_SPI_reset = 0;
        return SUCCESS;
      }
      else if(Slave == DISPLAY_TOUCH){
        digitalWrite(32, LOW); //TODO: Check for High or LOW active
        wait_time_SPI_reset = 0;
        return SUCCESS;
      }
      else if(Slave == RFID){
        int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 3, LOW); //TODO: Check for High or LOW active
        if(rv == ERROR){
          perror("Writing failed");
          return ERROR;
        }
        wait_time_SPI_reset = 0;
        return SUCCESS;
      }
      else{  
        perror("Unknown Slave Address");
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
  int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 6, LOW);  // TODO: Check on lo or high active

  if(rv == ERROR){
    perror("Write failed");
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
  int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 6, HIGH);  // TODO: Check on low or high active

  if(rv == ERROR){
    perror("Write failed");
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
  int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 6, LOW);  // TODO: Check on lo or high active

  if(rv == ERROR){
    perror("Write failed");
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
  int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 7, HIGH);  // TODO: Check on lo or high active

  if(rv == ERROR){
    perror("Write failed");
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
    perror("Write failed");
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
    perror("Write failed");
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
  if(time_wait_Learn_RF == 0){
    time_wait_Learn_RF = millis();
  }
  
  // Write to PIN 1 on GPIO B Expansion
  int rv = 0;

  if(mode == 1){
    // Pairing Mode 1 -> Pairs RF Control, returns acknowledge
    // GND, 1x short (<1s)
    unsigned long now = millis();
    if(time_wait_Learn_RF + 100 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
      time_wait_Learn_RF = 0;
      if(rv!=2){perror("Writing failed"); return ERROR;}
      return mode;
    }
    else{
      // Set Signal to active
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    }
  }
  if(mode == 2){
    // Pairing Mode 2 -> Pairs RF Control Button, returns acknowledge
    // GND, 2x short (<1s)
    unsigned long now = millis();
    if(time_wait_Learn_RF + 300 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
      time_wait_Learn_RF = 0;
      if(rv!=4){perror("Writing failed"); return ERROR;}
      return mode;
    }
    else if(time_wait_Learn_RF + 200 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    }
    else if(time_wait_Learn_RF + 100 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    }
    else{
      // Set Signal to active
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    }
  }
  if(mode == 3){
    // Pairing Mode 3 -> Pairs RF Control, returns no acknowledge
    // GND, 3x short (<1s)
    unsigned long now = millis();
    if(time_wait_Learn_RF + 500 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
      time_wait_Learn_RF = 0;
      if(rv!=6){perror("Writing failed"); return ERROR;}
      return mode;
    }
    else if(time_wait_Learn_RF + 400 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    }
    else if(time_wait_Learn_RF + 300 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    }
    else if(time_wait_Learn_RF + 200 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    }
    else if(time_wait_Learn_RF + 100 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    }
    else{
      // Set Signal to active
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    }
  }
  if(mode == 4){
    // Pairing Mode 4 -> Pairs RF Control Button, returns no acknowledge
    // GND, 4x short (<1s)
    unsigned long now = millis();
    if(time_wait_Learn_RF + 700 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
      time_wait_Learn_RF = 0;
      if(rv!=8){perror("Writing failed"); return ERROR;}
      return mode;
    }
    else if(time_wait_Learn_RF + 600 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    }
    else if(time_wait_Learn_RF + 500 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    }
    else if(time_wait_Learn_RF + 400 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    }
    else if(time_wait_Learn_RF + 300 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    }
    else if(time_wait_Learn_RF + 200 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    }
    else if(time_wait_Learn_RF + 100 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    }
    else{
      // Set Signal to active
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    }
  }
  if(mode == 5){
    // Disconnect Mode 1 -> Removes 1 single RF Controller from list
    // GND, 1x lang (>3s)
    unsigned long now = millis();
    if(time_wait_Learn_RF + 3100 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
      time_wait_Learn_RF = 0;
      if(rv!=2){perror("Writing failed"); return ERROR;}
      return mode;
    }
    else{
      // Set Signal to active
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    }
  }
  if(mode == 6){
    // Disconnect Mode 2 -> Removes all paired RF Controller from List
    // GND, 2x lang (>3s)
    unsigned long now = millis();
    if(time_wait_Learn_RF + 7100 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
      time_wait_Learn_RF = 0;
      if(rv!=4){perror("Writing failed"); return ERROR;}
      return mode;
    }
    else if(time_wait_Learn_RF + 4000 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    }
    else if(time_wait_Learn_RF + 3100 <= now){
      // Reset Signal
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    }
    else{
      // Set Signal to active
      rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    }
  }
  return 0;
}

/***************************

I²C Read and Write Functions

****************************/

int GPIO_Exp_WriteRegister(int reg, int value){
  /**
 * @brief Writes 8bit Value to a register of the GPIO Expansion
 * 
 * @param reg: Register Adress as Hex
 * @param value: 8bit value to be writen in register
 * 
 * @return 1 at Success
 * @return -1: Writing Address Failed
 * @return -1: Value out of Range
 * @return -1: Writing Data Failed
 * @return -1: Sending Buffer Failed
 */

  // Check for Value out of Range
  if(value > 255 || value < 0){
    // Value out of Range
    perror("Value out of Range");
    Serial.println("Value out of Range");
    return ERROR;
  }

  if(reg<0x00 || (reg < 0x10 && reg > 0x0A) || reg > 0x1A){
    perror("Register Address out of Range");
    Serial.println("Register Address out of Range");
    return ERROR;
  }

  // Write Address and Write Bit to Buffer
  Wire.beginTransmission(GPIO_EXP_ADRESS);
  
  // Write register Address to Buffer
  int rv = Wire.write(reg); 
  if(rv <= 0){
    perror("Write register Adress failed");
    Serial.println("Write register Adress failed");
    return ERROR;
  }  

  // Write Data to Buffer
  rv = Wire.write(value);      // Write Data
  if(rv <= 0){
    perror("Write data failed");
    Serial.println("Write data failed");
    return ERROR;
  }  

  // Send Buffer to Communication
  rv = Wire.endTransmission();
  if(rv != 0){
    perror("Sending Bufferr to Communication failed");
    Serial.println("Sending Bufferr to Communication failed");
    return ERROR;
  }
  return SUCCESS;
}

int GPIO_Exp_ReadRegister(int reg){
  /**
   * @brief Reads Regsiter from GPIO Expansion
   * 
   * @param reg: Register adress in hex
   * 
   * @return -1: Write register Address Failed
   * @return -1: Sendig Data Buffer failed
   * @return -1: Data Could not be read
   * @return -1: Register Address out of Range
   * @return data: Data was read successfully
   */

  Serial.println("\n\nReading Register from GPIO Expansion");

  // Check for Address out of Range
  if(reg<0x00 || (reg < 0x10 && reg > 0x0A) || reg > 0x1A){
    perror("Register Address out of Range");
    Serial.println("Register Address out of Range");
    return ERROR;
  }
  else{
    Serial.println("Register Address is in Range");
  }

  // Write Address and Write Bit to Buffer
  Serial.println("Begin Transmission - Address: " + String(GPIO_EXP_ADRESS, HEX));
  Wire.beginTransmission(GPIO_EXP_ADRESS);

  // Write register address to Buffer
  Serial.println("Writing Register Address to Buffer");
  int rv = Wire.write(reg);              // Registeradresse senden
  Serial.print("Read rom Register done - Return Value: ");
  Serial.println(rv);
  if(rv <= 0){
    perror("Write Failed");
    Serial.println("Write Failed");
    return ERROR;
  }
  else{
    Serial.println("Register Address written to Buffer");
  }

  // Send Buffer to Communication
  Serial.println("Sending Buffer to Communication");
  rv = Wire.endTransmission(false);  // Stop-Bedingung vermeiden (Repeated Start)
  if(rv != 0){
    perror("sending Data failed");
    Serial.println("sending Data failed");
    return ERROR;
  }
  else{
    Serial.println("Data sent via I2C");
  }

  // Write Address and Read bit to Buffer
  Serial.println("Requesting Data from Communication");
  Wire.requestFrom(GPIO_EXP_ADRESS, 1);

  // Read Register from Communication
  if (Wire.available()) {
    Serial.println("Data available");
    Serial.println("Reading Data");
    return Wire.read(); // Byte zurückgeben
  }
  else{
    perror("Data could not be read");
    Serial.println("Data could not be read");
  }
  return ERROR; // Fehlerwert
}

int GPIO_Exp_WriteBit(int reg, int bit, int value){

  /**
   * @brief Reads the Value from addressed Register and only changes designated bit without touching rest of the register
   * 
  * @param reg: Regsiteradress in hex
  * @param bit: Bitposition to be changed at
  * @param value: Binary Value to be set at bit position
  *
  * @return -1: Register Read Error
  * @return -1: Non Binary Value
  * @return -1: Register Write Error
  * @return -1: Data Send Error
  * @return -1: Register Address out of Range
  * @return 1: Writing Successful
  */

  Serial.println("Writing Bit to GPIO Expansion");

  // Check for value in Range
  if(value > 1 || value < 0){
    perror("Value must be binary");
    Serial.println("Value must be binary");
    return ERROR;
  }
  else{
    Serial.println("Value is in Range");
  }
  
  // Check for Address out of Range
  if(reg<0x00 || (reg < 0x10 && reg > 0x0A) || reg > 0x1A){
    perror("Register Address out of Range");
    Serial.println("Register Address out of Range");
    return ERROR;
  }
  else{
    Serial.println("Register Address is in Range");
  }

  // Set Address and Write Bit
  Serial.println("Begin Transmission");
  Wire.beginTransmission(GPIO_EXP_ADRESS);

  // Write Register Adress to Buffer
  int rv = Wire.write(reg);  
  if(rv <= 0){
    perror("Writing Failed");
    Serial.println("Writing Failed");
    return ERROR;
  } 
  else{
    Serial.println("Register Address written to Buffer");
  }

  // Read Data from Register
  int reg_value = GPIO_Exp_ReadRegister(reg);
  if(reg_value == ERROR){
    perror("Register could not be read");
    Serial.println("Register could not be read");
    return ERROR;
  }
  else{
    Serial.print("Register Read, Value: ");
    Serial.println(reg_value);
  }

  // Changes Bit in Register Data and write data to Buffer
  if(value == HIGH){
    rv = Wire.write(reg_value | (1 << bit)); 
    if(rv <= 0){
      perror("Writing Failed");
      Serial.println("Writing Failed");
      return ERROR;
    }
    else{
      Serial.println("Value HIGH written to Buffer");
    }
  }
  else if(value == LOW){
    rv = Wire.write(reg_value & ~(1 << bit));
    if(rv <= 0){
      perror("Writing Failed");
      Serial.println("Writing Failed");
      return ERROR;
    }
    else{
      Serial.println("Value LOW written to Buffer");
    }
  }
  
  // Send Buffer to Communication
  Serial.println("Sending Buffer to Communication");
  rv = Wire.endTransmission();
  if(rv != 0){
    perror("Error sending data via I2C");
    Serial.println("Error sending data via I2C");
    return ERROR;
  }
  else{
    Serial.println("Data sent via I2C");
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
    perror("Value out of Range");
    Serial.println("Value out of Range");
    return ERROR;
  }

  // Check for Address out of Range
  if(reg<0x00 || (reg < 0x10 && reg > 0x0A) || reg > 0x1A){
    perror("Register Address out of Range");
    Serial.println("Register Address out of Range");
    return ERROR;
  }

  // Read Register from GPIO Port Expansion
  int value = GPIO_Exp_ReadRegister(reg);
  if(value == ERROR){
    perror("Read Register Failed");
    Serial.println("Read Register Failed");
    return ERROR;
  }

  // mask and filter for single bit
  u_int8_t bit_value = value >> bit;
  bit_value &= 0x01;

  return bit_value;
}