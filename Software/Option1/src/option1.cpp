#include <Arduino.h>
#include <option1.hpp>
#include <string.h>
#include <Wire.h>

int SPI_select(int Slave);
int SPI_deselect();
int send_RemoteDrive_Request(int send_CAN);
int send_SOC_Request(int send_CAN);
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
 * @return 1 at Success, 0 if there is no Reset for given Slave, -1 at fail
 */
  // Reset for given Slave
  if(Slave == DISPLAY_LCD){
    digitalWrite(27, HIGH);//TODO: Check for High or LOW active

    //TODO: Wait and Set back to LOW
    return SUCCESS;
  }
  if(Slave == DISPLAY_TOUCH){
    digitalWrite(32, HIGH); //TODO: Check for High or LOW active

    //TODO: Wait and Set back to LOW
    return SUCCESS;
  }
  if(Slave == RFID){
    GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 3, HIGH); //TODO: Check for High or LOW active

    //TODO: Wait and Set back to LOW
  }
  perror("Unknown Slave Address");
  return ERROR;
}

int send_RemoteDrive_Request(int send_CAN){
/**
 * @brief Depending on the System, this function sends a CAN Message to the VCU or powers the MOSFET to send the Signal for Remote Drive to VCU via wired Connection. 
 * 
 * @return 1 at Success, -1 at fail
 */
  // Send analog Signal
  // Set Pin GPIOB 4 on Port Extension to HIGH to power the MOSFET
  int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 4, HIGH);
  if(rv == ERROR){
    perror("Write failed");
    return ERROR;
  }
  delay(300); // Wait 300ms for the Signal to be be Recieved by VCU

  // Set Pin GPIOB 4 on Port Extension to LOW to reset the MOSFET
  int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 4, LOW);
  if(rv == ERROR){
    perror("Write failed");
    return ERROR;
  }

  if(send_CAN){
    //TODO: Send CAN-Message with RemoteDive Command to VCU
  }

  return SUCCESS;
}

int send_SOC_Request(int send_CAN){
/**
 * @brief Depending on the System, this function sends a CAN Message to the VCU or powers the MOSFET to send the Signal to VCU via wired Connection
 * 
 * @return 1 at Success, -1 at fail
 */
  // Send analog Signal
  // Set Pin GPIOB 5 on Port Extension to HIGH to power the MOSFET
  int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 5, HIGH);
  if(rv == ERROR){
    perror("Write failed");
    return ERROR;
  }

  // Wait and reset
  delay(300);

  // Set Pin GPIOB 5 on Port Extension to LOW to reset the MOSFET
  int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 5, LOW);
  if(rv == ERROR){
    perror("Write failed");
    return ERROR;
  }

  // Send CAN Message
  if(send_CAN){
    //TODO: Send CAN-Message with RemoteDive Command to VCU
  }

  return SUCCESS;
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

    // TODO: noo delay
  }
  
  // Write to PIN 1 on GPIO B Expansion
  int rv = 0;
  if(mode == 1){
    // Pairing Mode 1 -> Pairs RF Control, returns acknowledge
    // GND, 1x short (<1s)
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    delay(100);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    if(rv==2){perror("Writing failed"); return ERROR;}
    return mode;
  }
  if(mode == 2){
    // Pairing Mode 2 -> Pairs RF Control Button, returns acknowledge
    // GND, 2x short (<1s)
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    delay(100);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    delay(100);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    delay(100);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    if(rv==4){perror("Writing failed"); return ERROR;}
    return mode;
  }
  if(mode == 3){
    // Pairing Mode 3 -> Pairs RF Control, returns no acknowledge
    // GND, 3x short (<1s)
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    delay(100);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    delay(100);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    delay(100);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    delay(100);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    delay(100);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    if(rv==6){perror("Writing failed"); return ERROR;}
    return mode;
  }
  if(mode == 4){
    // Pairing Mode 4 -> Pairs RF Control Button, returns no acknowledge
    // GND, 4x short (<1s)
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    delay(100);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    delay(100);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    delay(100);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    delay(100);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    delay(100);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    delay(100);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    delay(100);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    if(rv==8){perror("Writing failed"); return ERROR;}
    return mode;
  }
  if(mode == 5){
    // Disconnect Mode 1 -> Removes 1 single RF Controller from list
    // GND, 1x lang (>3s)
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    delay(3500);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    if(rv==2){perror("Writing failed"); return ERROR;}
    return mode;
  }
  if(mode == 6){
    // Disconnect Mode 2 -> Removes all paired RF Controller from List
    // GND, 2x lang (>3s)
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    delay(3500);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    delay(500);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, LOW);
    delay(3500);
    rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
    if(rv==4){perror("Writing failed"); return ERROR;}
    return mode;
  }

  // unknown Mode, never met one condition
  perror("Unknown Learning Mode");
  return ERROR;
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
    return ERROR;
  }

  if(reg<0x00 || (reg < 0x10 && reg > 0x0A) || reg > 0x1A){
    perror("Register Address out of Range");
    return ERROR;
  }

  // Write Address and Write Bit to Buffer
  Wire.beginTransmission(GPIO_EXP_ADRESS);
  
  // Write register Address to Buffer
  int rv = Wire.write(reg); 
  if(rv <= 0){
    perror("Write register Adress failed");
    return ERROR;
  }  

  // Write Data to Buffer
  rv = Wire.write(value);      // Write Data
  if(rv <= 0){
    perror("Write data failed");
    return ERROR;
  }  

  // Send Buffer to Communication
  rv = Wire.endTransmission();
  if(rv != 0){
    perror("Sending Bufferr to Communication failed");
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

  // Check for Address out of Range
  if(reg<0x00 || (reg < 0x10 && reg > 0x0A) || reg > 0x1A){
    perror("Register Address out of Range");
    return ERROR;
  }

  // Write Address and Write Bit to Buffer
  Wire.beginTransmission(GPIO_EXP_ADRESS);

  // Write register address to Buffer
  int rv = Wire.write(reg);              // Registeradresse senden
  if(rv <= 0){
    perror("Write Failed");
    return ERROR;
  }

  // Send Buffer to Communication
  rv = Wire.endTransmission(false);  // Stop-Bedingung vermeiden (Repeated Start)
  if(rv != 0){
    perror("sending Data failed");
    return ERROR;
  }

  // Write Address and Read bit to Buffer
  Wire.requestFrom(GPIO_EXP_ADRESS, uint8_t(1));

  // Read Register from Communication
  if (Wire.available()) {
    return Wire.read(); // Byte zurückgeben
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

  // Check for value in Range
  if(value > 1 || value < 0){
    perror("Value must be binary");
    return ERROR;
  }

  // Check for Address out of Range
  if(reg<0x00 || (reg < 0x10 && reg > 0x0A) || reg > 0x1A){
    perror("Register Address out of Range");
    return ERROR;
  }

  // Set Address and Write Bit
  Wire.beginTransmission(GPIO_EXP_ADRESS);

  // Write Register Adress to Buffer
  int rv = Wire.write(reg);  
  if(rv <= 0){
    perror("Writing Failed");
    return ERROR;
  } 

  // Read Data from Register
  int reg_value = GPIO_Exp_ReadRegister(reg);
  if(reg_value == ERROR){
    perror("Register could not be read");
    return ERROR;
  }

  // Changes Bit in Register Data and write data to Buffer
  if(value == HIGH){
    rv = Wire.write(reg_value | (value << bit)); 
    if(rv <= 0){
      perror("Writing Failed");
      return ERROR;
    }
  }
  else if(value == LOW){
    rv = Wire.write(reg_value & (value << bit));
    if(rv <= 0){
      perror("Writing Failed");
      return ERROR;
    }
  }
  
  // Send Buffer to Communication
  rv = Wire.endTransmission();
  if(rv != 0){
    perror("Error sending data via I2C");
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
    perror("Value out of Range");
    return ERROR;
  }

  // Check for Address out of Range
  if(reg<0x00 || (reg < 0x10 && reg > 0x0A) || reg > 0x1A){
    perror("Register Address out of Range");
    return ERROR;
  }

  // Read Register from GPIO Port Expansion
  int value = GPIO_Exp_ReadRegister(reg);
  if(value == ERROR){
    perror("Read Register Failed");
    return ERROR;
  }

  // mask and filter for single bit
  u_int8_t bit_value = value >> bit;
  bit_value &= 0x01;

  return bit_value;
}