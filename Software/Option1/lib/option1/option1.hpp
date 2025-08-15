#ifndef OPTION1_HPP
#define OPTION1_HPP

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
int recieve_Learn_RF_Module();
int recieve_Error_RF_Module();
int recieve_LED_Signal();
int recieve_RemoteDrive_Request();
int recieve_SOC_Request();
int recieve_Identification_Request();
int GPIO_Exp_WriteRegister(int reg, int value);
int GPIO_Exp_ReadRegister(int reg);
int GPIO_Exp_WriteBit(int reg, int bit, int value);
int GPIO_Exp_ReadBit(int reg, int bit);


/* GPIO define

GPIO1   -> SPI_INT_CAN2               -> Input                [Interrupt from CAN 2 Controller via SPI]
GPIO3   -> SPI_CLK                    -> Alternate Function   [SPI Clock]
GPIO4   -> CAN1_RX                    -> Alternate Function   [Receive for internal CAN-Controller]
GPIO13  -> SPI_CS_Display             -> Output               [SPI ChipSelect for Display]
GPIO14  -> LCD_DC                     ->                      [... Pin for LCD Display]
GPIO16  -> CAN1_TX                    -> Alternate Function   [Transmit for internal CAN-Controller]
GPIO17  -> SPI_CS_Flash               -> Output               [SPI ChipSelect for Flash Chip]
GPIO18  -> SPI_CS_CAN2                -> Output               [SPI ChipSelect for CAN 2 Controller]
GPIO19  -> SPI_MISO                   -> Alternate Function   [SPI Master In/Slave Out Signal]
GPIO21  -> SPI_MOSI                   -> Alternate Function   [SPI Master Out/Slave In Signal]
GPIO22  -> U0TXD                      -> Alternate Function   [Transmit for UART]
GPIO23  -> U0RXD                      -> Alternate Function   [Receive for UART]
GPIO25  -> I2C_SDA                    -> Alternate Function   [I2C Data]
GPIO26  -> LCD_BL                     -> Alternate Function   [PWM for Display BackLight]        
GPIO27  -> LCD_RST                    -> Output               [SPI Reset for LCD Display]
GPIO32  -> TP_RST                     -> Output               [SPI Reset for Display Touch Controller]
GPIO33  -> I2C_SCL                    -> Alternate Function   [I2C Clock]
GPIO34  -> TP_INT                     -> Input                [Interrupt from Display Touch Controller via SPI]
GPIO35  -> Interrupt Port-Expander    -> Input                [Interrupt from GPIO Port Expander]

*/

#define I2C_SDA_PIN 25
#define I2C_SCL_PIN 33 

#define UART_RX_PIN 23
#define UART_TX_PIN 22

#define CAN1_RX_PIN 4
#define CAN1_TX_PIN 16

#define SPI_CLK_PIN 3
#define SPI_MISO_PIN 19
#define SPI_MOSI_PIN 21

#define SPI_CS_DISPLAY_PIN 13
#define SPI_CS_FLASH_PIN 17
#define SPI_CS_CAN2_PIN 18

#define SPI_RST_TP_PIN 32
#define SPI_RST_LCD_PIN 27

#define SPI_INT_CAN2_PIN 1
#define SPI_INT_TP_PIN 34

#define LCD_DC_PIN 14
#define LCD_BL_PIN 26

#define INT_PE_PIN 35

// Display Setting
#define DISPLAY_PWM_CH 0        // [ESP32 PWM Channel 0 ... 15]
#define DISPLAY_PWM_FREQ 1000   // [Hz]
#define DISPLAY_PWM_RES 8       // [Resolution for PWM Signal [bits] ]
#define DISPLAY_PWM_DC 50       // [Duty Cycle for PWM Signal [%]]

#define Display 0
#define Flash 1
#define CAN2 2
#define RFID 3

// SPI Reset
#define DISPLAY_LCD 1
#define DISPLAY_TOUCH 2

// GPIO Expander Register Adresses
#define GPIO_EXP_ADRESS 0b0100000
#define GPIO_EXP_IODIRA 0x00
#define GPIO_EXP_IPOLA 0x01
#define GPIO_EXP_GPINTENA 0x02
#define GPIO_EXP_DEFVALA 0x03
#define GPIO_EXP_INTCONA 0x04
#define GPIO_EXP_IOCONA 0x05
#define GPIO_EXP_GPPUA 0x06
#define GPIO_EXP_INTFA 0x07
#define GPIO_EXP_INTCAPA 0x08
#define GPIO_EXP_GPIOA 0x09
#define GPIO_EXP_OLATA 0x0A
#define GPIO_EXP_IODIRB 0x10
#define GPIO_EXP_IPOLB 0x11
#define GPIO_EXP_GPINTENB 0x12
#define GPIO_EXP_DEFVALB 0x13
#define GPIO_EXP_INTCONB 0x14
#define GPIO_EXP_IOCONB 0x15
#define GPIO_EXP_GPPUB 0x16
#define GPIO_EXP_INTFB 0x17
#define GPIO_EXP_INTCAPB 0x18
#define GPIO_EXP_GPIOB 0x19
#define GPIO_EXP_OLATB 0x1A

// Logical Values
#define OUTPUT 0
#define INPUT 1

#define INVERT 1
#define NONINVERT 0

#define IOC_EN 1
#define IOC_DIS 0

#define COMP_TO_DEFVAL 1
#define COMP_TO_PREVIOUS 0

// general
#define ERROR -1
#define SUCCESS 1

#define HIGH 1
#define LOW 0

#define TRUE 1
#define FALSE 0

#define ON 1
#define OFF 0



#endif