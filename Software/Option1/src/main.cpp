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

static int ISR_Pairing_LED_CTR = FALSE;
static int ISR_LED_Signal_Flag = FALSE;
static int ISR_RF_Error_CTR = FALSE;
static int ISR_RX_2_Flag = FALSE;
static int ISR_RX_3_Flag = FALSE;
static int ISR_RX_4_Flag = FALSE;
static int LED_cur_state = OFF;
static int ID_cur_state = OFF;

// put function declarations here:
void init_ports();
int init_GPIO_Exp_Ports(); 
void init_Interrupts();
void ISR_GPIO_Expansion();
void ISR_CAN2();
void ISR_TouchController();

void setup() {
  init_ports();
  init_GPIO_Exp_Ports();
  init_Interrupts();
}

void loop() {
  // put your main code here, to run repeatedly:

  // Polling for Interrupt Flags set by GPIO Expansion ***************************************************************************
  if(ISR_RX_2_Flag == TRUE){
    // The Microcontroller has recieved a Signal, which indicates, that a remote drive Request was sent by the RF Control
    // The Signal must now be transmitted to the VCU
    send_RemoteDrive_Request(FALSE);
  }
  if(ISR_RX_3_Flag == TRUE){
    // The Microcontroller has recieved a Signal, which indicates, that a SOC Request was sent by the RF Control
    // The Signal must now be transmitted to the VCU
    send_SOC_Request(FALSE);
  }
  if(ISR_RX_4_Flag == TRUE && ID_cur_state == OFF){
    // The Microcontroller has recieved a Signal, which indicates, that a Identification Request was sent by the RF Control
    // The Status LED must now be switched on as long as the button is pressed
    ID_cur_state = ON;
    LED_cur_state = ON;
    Status_LED_ON();
  }
  if(ISR_RX_4_Flag == FALSE && ID_cur_state == ON){
    // / The Microcontroller has recieved a Signal, which indicates, that a Identification Request has ended
    // The Status LED must now be switched off as the button is no longer pressed
    ID_cur_state = OFF;
    LED_cur_state = OFF;
    Status_LED_OFF();
  }
  if(ISR_LED_Signal_Flag == TRUE && LED_cur_state == OFF && ID_cur_state == OFF){
  // The Microcontroller has recieved a Signal from VCU, it has to activate Status LED
  // The Status LED must now be switched on as long as the signal is active
  // This Signal low priority compared to Identification via RF Control
    LED_cur_state = ON;
    Status_LED_ON();
  }
  if(ISR_LED_Signal_Flag == FALSE && LED_cur_state == ON && ID_cur_state == OFF){
  // The Microcontroller has recieved a Signal from VCU, Status LED activation has ended
  // The Status LED must now be switched off as the signal is no longer acive
  // This Signal low priority compared to Identification via RF Control
    LED_cur_state = OFF;
    Status_LED_OFF();
  }

  // End of polling for Interrupts by GPIO expansion *****************************************************************************

}

// put function definitions here *************************************************************************************************

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

int init_GPIO_Exp_Ports(){
  // SET Bank for addressing
  int rv = GPIO_Exp_WriteBit(0x05, 7, HIGH);
  if(rv == ERROR){
    perror("Setting Bank for address failed");
    return ERROR;
  }
  
  rv = 0;
  // set Pin Direction
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 0, INPUT); // LED Learn Mode
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 1, INPUT); // LED Signal from VCU
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 2, INPUT); // RC Errors
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 3, INPUT); // RC_Recieve_CH2
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 5, INPUT); // RC Receive CH3
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 7, INPUT); // RC Receive CH4
  if(rv != 6){perror("Error in init Ports Bank A"); return ERROR;}

  rv = 0;
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 0, OUTPUT); // LED Pin
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 1, OUTPUT); // Activate RC Learn Mode
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 2, OUTPUT); // RFID SPI Chip Select
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 3, OUTPUT); // RFID SPI Reset 
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 4, OUTPUT); // RC Transmit CH2
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 5, OUTPUT); // RC Transmit CH3
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 6, OUTPUT); // CAN1 Silent Mode
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 7, OUTPUT); // CAN2 Silent Mode
  if(rv != 8){perror("Error in init Ports Bank B"); return ERROR;}

  // set PullUps where needed
  rv = 0;
  rv += GPIO_Exp_WriteRegister(GPIO_EXP_GPPUB, 0x00); // No PullUps needed
  rv += GPIO_Exp_WriteRegister(GPIO_EXP_GPPUB, 0x00); // no PullUps needed
  if(rv != 2){perror("Error in set PullUp Resistors"); return ERROR;}

  // set Interrupt Settings 
  rv = 0;
  // Enable Interrupts for Pins 7, 5, 3, 2, 1, 0
  rv += GPIO_Exp_WriteRegister(GPIO_EXP_GPINTENA, 0xAF);

  // Define Comparison Values for Pins to throw Interrupts
  // Interrupt is set, if opposite value occurd
  rv += GPIO_Exp_WriteBit(GPIO_EXP_DEFVALA, 1, HIGH); // low-active Signal
  rv += GPIO_Exp_WriteBit(GPIO_EXP_DEFVALA, 3, HIGH); // low-active Signal

  rv += GPIO_Exp_WriteBit(GPIO_EXP_DEFVALA, 3, LOW);
  rv += GPIO_Exp_WriteBit(GPIO_EXP_DEFVALA, 5, LOW);

  // Set Interrupt Control Regsiter to comparison against previous Values or DEFVAL Regsister
  // Pins 5, 3, 2 and 0 are compared against defVal (set to 1)
  // Pins 7 and 1 are compared against previous Value (set to 0)
  // -> 0b 0010 1101
  rv += GPIO_Exp_WriteRegister(GPIO_EXP_INTCONA, 0x2D);

  // Set Interrupt polarity to High_active
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IOCONA, 1, HIGH);

  if(rv != 7){perror("Error in init Interrupts for GPIO Expansion"); return ERROR;}

  // Initial Value
  rv = 0;
  rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
  if(rv != 1){perror("Error in init Interrupts for GPIO Expansion"); return ERROR;}

  return SUCCESS;
}

void init_Interrupts(){
  attachInterrupt(digitalPinToInterrupt(INT_PE_PIN), ISR_GPIO_Expansion, RISING);
  attachInterrupt(digitalPinToInterrupt(SPI_INT_CAN2_PIN), ISR_CAN2, RISING);
  attachInterrupt(digitalPinToInterrupt(SPI_INT_TP_PIN), ISR_TouchController, RISING);
}

// Interrupts
void ISR_GPIO_Expansion(){
  // Interrupt Service Routine for GPIO Expansion Interrupts
  // Read Interrupt Pending Register
  // Set Flags for Interrupts

  int flags = GPIO_Exp_ReadRegister(GPIO_EXP_INTFA);
  int rv = GPIO_Exp_ReadRegister(GPIO_EXP_INTCAPA);

  if(flags != ERROR){
    
    // Check for every single bit
    if((flags >> 0) & 0x01){
      // Interrupt ocured for Bit 0
      // RF Controller Pairing Mode Acknowledge
      ISR_Pairing_LED_CTR += 1; 
    }
    if((flags >> 1) & 0x01){
      // Interrupt ocured for Bit 1
      // LED Signal recieved from VCU
      ISR_LED_Signal_Flag = TRUE;
    }
    if((flags >> 2) & 0x01){
      // Interrupt ocured for Bit 2
      // RF Controller Error received
      ISR_RF_Error_CTR += 1;
    }
    if((flags >> 3) & 0x01){
      // Interrupt ocured for Bit 3
      // Remote Drive Command received from RF Module 
      ISR_RX_2_Flag = TRUE;
    }
    if((flags >> 4) & 0x01){
      // Interrupt ocured for Bit 4
      // Not used for interrupt
    }
    if((flags >> 5) & 0x01){
      // Interrupt ocured for Bit 5
      // SOC Command received from RF Module 
      ISR_RX_3_Flag = TRUE;
    }
    if((flags >> 6) & 0x01){
      // Interrupt ocured for Bit 6
      // not used for interrupt
    }
    if((flags >> 7) & 0x01){
      // Interrupt ocured for Bit 7
      // Status LED Command received from RF Module 
      if((rv >> 7) & 0x01){
        // Value during interrupt is HIGH -> Button is pressed
        // Status LED Switch on
        ISR_RX_4_Flag = TRUE;
      }
      else{
        // Status LED Switch off
        ISR_RX_4_Flag = FALSE;
      }
    }

  }
}

void ISR_CAN2(){
  // Interrupt Service Routine for CAN2 Messages
  // Set Flags for Interrupts
}

void ISR_TouchController(){
  // Interrupt Service Routine activated by TouchController
  // Set Flags for Interrupts
}


