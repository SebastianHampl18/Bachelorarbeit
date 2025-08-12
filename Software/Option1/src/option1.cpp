#include <Arduino.h>
#include <option1.hpp>
#include <string.h>
#include <Wire.h>

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
  

}

int send_RemoteDrive_Request(){
/**
 * @brief Depending on the System, this function sends a CAN Message to the VCU or powers the MOSFET to send the Signal to VCU via wired Connection
 * 
 * @return 1 at Success, -1 at fail
 */
    // Code here
    // Set Pin GPIOB 4 on Port Extension to HIGH
  int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 4, HIGH);

  if(rv == ERROR){
    perror("Write failed");
    return ERROR;
  }

  //TODO: Wait and reset

  return SUCCESS;
}

int send_SOC_Request(){
/**
 * @brief Depending on the System, this function sends a CAN Message to the VCU or powers the MOSFET to send the Signal to VCU via wired Connection
 * 
 * @return 1 at Success, -1 at fail
 */
    // Code here
    // Set Pin GPIOB 5 on Port Extension to HIGH
  int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 5, HIGH);

  if(rv == ERROR){
    perror("Write failed");
    return ERROR;
  }

  // TODO: Wait and reset

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
 * @return 1 at Success, -1 at fail
 */
  // TODO: Write to PIN 2 on GPIO Expansion
}

int recieve_Learn_RF_Module(){
/**
 * @brief RF Module returnes Data after activating learn mode. Data is diffrent, when learning was succesful or not
 * 
 * @return data at Success, -1 at fail
 */
  // TODO: Process Data from RF Module recieved by GPIO Expansion
}

int recieve_Error_RF_Module(){
/**
 * @brief RF Module returnes Error Code after activating learn mode. Error is flashing LED Code witch is read via Micro Controller
 * 
 * @return Error Code at Success, -1 at fail
 */
  // TODO: Process Errors from RF Module recieved by GPIO Expansion
}

int recieve_LED_Signal(){
/**
 * @brief RF Module returnes Data after activating learn mode. Data is diffrent, when learning was succesful or not
 * 
 * @return data at Success, -1 at fail
 */
  // TODO: Process Data from VCU for Status LED
}

int recieve_RemoteDrive_Request(){
/**
 * @brief Process Data witch were recieved as User Input for Remote Drive. Data must be send to VCU -> send_remoteDrive_Request
 * 
 * @return data at Success, -1 at fail
 */
  // TODO: Process Data from RF Module and Send to VCU
}

int recieve_SOC_Request(){
  /**
 * @brief Process Data witch were recieved as User Input for SOC Request. Data must be send to VCU -> send_SOC_Request
 * 
 * @return data at Success, -1 at fail
 */
  // TODO: Process Data from RF Module and Send to VCU
}

int recieve_Identification_Request(){
/**
 * @brief Process Data witch were recieved as User Input for Identification. Status LED must flash
 * 
 * @return data at Success, -1 at fail
 */
  // TODO: Process Data from RF Module and toggle Status LED
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
  Wire.requestFrom(GPIO_EXP_ADRESS, (uint8_t)1);

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