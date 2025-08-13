#include <Arduino.h>
#include "option1.hpp"
#include <HardwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <ESP32CAN.h>
#include <CAN_config.h>

CAN_device_t CAN1;
HardwareSerial MicroUSB(1);
SPIClass SPI(VSPI);

// put function declarations here:
void init_ports();
void init_GPIO_Exp_Ports(); 

void setup() {
  init_ports();
  init_GPIO_Exp_Ports();
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:

// Init Ports
void init_ports(){
  // Output
  pinMode(SPI_CS_CAN2_PIN, OUTPUT);
  pinMode(SPI_CS_DISPLAY_PIN, OUTPUT);
  pinMode(SPI_CS_FLASH_PIN, OUTPUT);
  pinMode(SPI_RST_LCD_PIN, OUTPUT);
  pinMode(SPI_RST_TP_PIN, OUTPUT);

  // Input
  pinMode(SPI_INT_CAN2_PIN, INPUT);
  pinMode(SPI_INT_TP_PIN, INPUT);
  pinMode(INT_PE_PIN, INPUT);

  // UART
  MicroUSB.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // SPI
  SPI.begin(SPI_CLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

  // PWM For Display Backlight
  ledcSetup(DISPLAY_PWM_CH, DISPLAY_PWM_FREQ, DISPLAY_PWM_RES); // Configure Channel
  ledcAttachPin(LCD_BL_PIN, DISPLAY_PWM_CH);                    // Define Pin for PWM
  ledcWrite(DISPLAY_PWM_CH, 2^DISPLAY_PWM_RES*DISPLAY_PWM_DC);  // 50% Duty Cycle

  // CAN
  CAN1.rx_pin_id = gpio_num_t(CAN1_RX_PIN);
  CAN1.tx_pin_id = gpio_num_t(CAN1_TX_PIN);
  CAN1.speed = CAN_SPEED_500KBPS;
  CAN1.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
}

void init_GPIO_Exp_Ports(){
  // set Pin Direction
  GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 0, INPUT); // LED Learn Mode
  GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 1, INPUT); // LED Signal from VCU
  GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 2, INPUT); // RC Errors
  GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 3, INPUT); // RC_Recieve_CH2
  GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 5, INPUT); // RC Receive CH3
  GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 7, INPUT); // RC Receive CH4

  GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 0, OUTPUT); // LED Pin
  GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 1, OUTPUT); // Activate RC Learn Mode
  GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 2, OUTPUT); // RFID SPI Chip Select
  GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 3, OUTPUT); // RFID SPI Reset 
  GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 4, OUTPUT); // RC Transmit CH2
  GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 5, OUTPUT); // RC Transmit CH3
  GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 6, OUTPUT); // CAN1 Silent Mode
  GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 7, OUTPUT); // CAN2 Silent Mode

  // set PullUps where needed
  GPIO_Exp_WriteRegister(GPIO_EXP_GPPUB, 0x00); // No PullUps needed
  GPIO_Exp_WriteRegister(GPIO_EXP_GPPUB, 0x00); // no PullUps needed

  // set Interrupt Settings
  // TODO:
}

