#include <Arduino.h>
#include <option1.hpp>

int SPI_select(int Slave){
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
  digitalWrite(SPI_CS_DISPLAY_PIN, HIGH);
  digitalWrite(SPI_CS_FLASH_PIN, HIGH);
  digitalWrite(SPI_CS_CAN2_PIN, HIGH);

  // TODO: Dactivate ChipSelect for RFID

}

int SPI_reset(int Slave){
    // TODO: Reset for given Slave
}

int send_RemoteDrive_Request(){
    // Code here
    // TODO: Set Pin GPIOB 4 on Port Extension to HIGH
}

int send_SOC_Request(){
    // Code here
    // TODO: Set Pin GPIOB 4 on Port Extension to HIGH
}

int CAN1_silent(){
    // TODO: Write To PIN 6 on GPIO Expansion
}

int CAN1_not_silent(){
    // TODO: Write to Pin 6 on GPIO Expansion
}

int CAN2_silent(){
    // TODO: Write to PIN 7 on GPIO Expansion
}

int CAN2_not_silent(){
    // TODO: Write to PIN 7 on GPIO Expansion
}

int digitalWrite_GPIOB(int Port, int val){
    // TODO: Digital Write for Pins on Port Extension via I²C
}