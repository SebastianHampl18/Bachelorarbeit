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
    return 1;
  }
  else if (Slave == Flash)
  {
    digitalWrite(SPI_CS_FLASH_PIN, LOW);
    return 1;
  }
  else if (Slave == CAN2)
  {
    digitalWrite(SPI_CS_CAN2_PIN, LOW);
    return 1;
  }  
  else if (Slave == RFID)
  {
    // TODO: Set PIN GPIOB 2 on Port Extension to LOW
  }
  else
  {
    //Error in Defines
    return -1;
  }
}

void SPI_deselect(){
/**
 * @brief This functions Sets every CS to HIGH, no CS is active after this function
 */
  digitalWrite(SPI_CS_DISPLAY_PIN, HIGH);
  digitalWrite(SPI_CS_FLASH_PIN, HIGH);
  digitalWrite(SPI_CS_CAN2_PIN, HIGH);

  // TODO: Dactivate ChipSelect for RFID

}

int SPI_reset(int Slave){
/**
 * @brief Resets given SPI Slave
 * 
 * @param Slave Component to be reset
 * 
 * @return 1 at Success, 0 if there is no Reset for given Slave, -1 at fail
 */
    // TODO: Reset for given Slave
}

int send_RemoteDrive_Request(){
/**
 * @brief Depending on the System, this function sends a CAN Message to the VCU or powers the MOSFET to send the Signal to VCU via wired Connection
 * 
 * @return 1 at Success, -1 at fail
 */
    // Code here
    // TODO: Set Pin GPIOB 4 on Port Extension to HIGH
}

int send_SOC_Request(){
/**
 * @brief Depending on the System, this function sends a CAN Message to the VCU or powers the MOSFET to send the Signal to VCU via wired Connection
 * 
 * @return 1 at Success, -1 at fail
 */
    // Code here
    // TODO: Set Pin GPIOB 4 on Port Extension to HIGH
}

int CAN1_silent(){
/**
 * @brief Sets the CAN-Transciever for CAN1 to silent, Transciever con only listen not send
 * 
 * @return 1 at Success, -1 at fail
 */
    // TODO: Write To PIN 6 on GPIO Expansion
}

int CAN1_not_silent(){
/**
 * @brief Sets the CAN-Transciever for CAN1 back from silent, Transciever can now send and listen
 * 
 * @return 1 at Success, -1 at fail
 */
    // TODO: Write to Pin 6 on GPIO Expansion
}

int CAN2_silent(){
/**
 * @brief Sets the CAN-Transciever for CAN2 to silent, Transciever con only listen not send
 * 
 * @return 1 at Success, -1 at fail
 */
    // TODO: Write to PIN 7 on GPIO Expansion
}

int CAN2_not_silent(){
/**
 * @brief Sets the CAN-Transciever for CAN2 back from silent, Transciever can now send and listen
 * 
 * @return 1 at Success, -1 at fail
 */
    // TODO: Write to PIN 7 on GPIO Expansion
}

int digitalWrite_GPIOB(int port, int val){
/**
 * @brief Writes the Pins at the GPIO Expansion via I²C
 * 
 * @param port Port to be adressed
 * @param val Value to be set
 * 
 * @return 1 at Success, -1 at fail
 */
    // TODO: Digital Write for Pins on Port Extension via I²C
}

int Status_LED_ON(){
/**
 * @brief Switches the Status-LED to on
 * 
 * @return 1 at Success, -1 at fail
 */
  // TODO: Write to PIN 1 on GPIO Expansion
}

int Status_LED_OFF(){
/**
 * @brief Switches the Status-LED to off
 * 
 * @return 1 at Success, -1 at fail
 */
  // TODO: Write to PIN 1 on GPIO Expansion
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

int GPIO_Exp_WriteRegister(int reg, int value){
  Wire.beginTransmission(GPIO_EXP_ADRESS);
  Wire.write(reg);   
  Wire.write(value);      // Write Data
  Wire.endTransmission();
}

int GPIO_Exp_ReadRegister(int reg){
  Wire.beginTransmission(GPIO_EXP_ADRESS);
  Wire.write(reg);      // Registeradresse senden
  Wire.endTransmission(false); // Stop-Bedingung vermeiden (Repeated Start)

  Wire.requestFrom(GPIO_EXP_ADRESS, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read(); // Byte zurückgeben
  }
  return ERROR; // Fehlerwert
}


int GPIO_Exp_WriteBit(int reg, int bit, int value){

  Wire.beginTransmission(GPIO_EXP_ADRESS);
  Wire.write(reg);   
  int reg_value = GPIO_Exp_ReadRegister(reg);
  if(reg_value == ERROR){
    Wire.endTransmission();
    return ERROR;
  }
  if(value == HIGH){
    Wire.write(reg_value | (value << bit)); 
    Wire.endTransmission();
    return SUCCESS;
  }
  else if(value == LOW){
    Wire.write(reg_value & (value << bit));
    Wire.endTransmission();
    return SUCCESS;
  }
  else
    Wire.endTransmission();
    return ERROR;
}