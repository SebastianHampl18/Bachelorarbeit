#include <Arduino.h>
#include "option1.hpp"
#include <HardwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Preferences.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>

#include "LCD_Driver.h"
#include "Touch_Driver.h"
#include "GUI_Paint.h"
#include "image.h"



CAN_device_t CAN_cfg;
//CAN_config_t CAN_cfg;
HardwareSerial MicroUSB(1);
SPIClass my_SPI(VSPI);
hw_timer_t * TIM_RF_Learn_Active = NULL;
hw_timer_t * TIM_LED_Flashing = NULL;
Preferences ESP_storage;

// Set IP-Address for WIFI
IPAddress local_IP(192,168,4,1);     // ESP32 Access Point IP
IPAddress gateway(192,168,4,1);      // Gateway 
IPAddress subnet(255,255,255,0);     // Subnetmask

// Webserver
WebServer kart_server(80);

/****************            Global Variables            ****************************/
// CAN 1
CAN_Message Battery_Temperature;
CAN_Message Battery_Voltage;
CAN_Message Power_Data;
CAN_Message Option1_Commands;
CAN_Message VCU_Commands;

CAN_Signal Avg_Cell_Voltage;
CAN_Signal Max_Cell_Voltage;
CAN_Signal Min_Cell_Voltage;
CAN_Signal Overall_Voltage;
CAN_Signal Avg_Temp;
CAN_Signal Max_Temp;
CAN_Signal Min_Temp;
CAN_Signal Speed;
CAN_Signal RemoteDrive_Request;
CAN_Signal SOC_Request;
CAN_Signal LED_Signal;
CAN_Signal LED_Flashing;
CAN_Signal M_Engine;
CAN_Signal n_Engine;
CAN_Signal SOC;

// CAN 2
CAN_Message RFID_Data;

CAN_Signal Customer_ID;
CAN_Signal Power_Mode;

// Error Logging
File logFile;

volatile int ISR_Learn_LED_CTR = 0;
volatile bool ISR_LED_Signal_Flag = false;
volatile int ISR_RF_Error_CTR = 0;
volatile bool ISR_RX_2_Flag = false;
volatile bool ISR_RX_3_Flag = false;
volatile bool ISR_RX_4_Flag = false;
volatile bool LED_cur_state = OFF;
volatile bool ID_cur_state = OFF;
volatile bool ISR_Learn_RF_Flag = false;
volatile int ISR_Learn_RF_Mode = 0;
volatile bool Learn_RF_Active_Flag = false;
volatile bool Learn_RF_ACK_Flag = false;
volatile bool Learn_RF_ACK_Waiting = false;
volatile bool RF_Error_Active = false;
volatile unsigned long time_wait_Error = 0;
volatile bool CAN1_active = false;
volatile float CAN_speed = 0.0;
volatile float CAN_soc = 0.0;
volatile float CAN_battery_voltage = 0.0;
volatile float CAN_battery_temp = 0.0;
String RF_Connect_Return = "...";
volatile bool LED_Flashing_Active = false;
volatile uint8_t LED_Flashing_CTR = 0;
volatile bool Master_Mode_active = false;
volatile float Avg_Temp_value = 0.0;
volatile float Overall_Voltage_value = 0.00;
volatile bool CAN2_Message_Flag = false;
volatile bool ISR_GPIO_Exp_Flag = false;
volatile bool Touch_ISR_Flag = false;


// put function declarations here:
void init_ports();
int init_GPIO_Exp_Ports(); 
void init_Interrupts();
void ISR_GPIO_Expansion();
void ISR_CAN2();
void ISR_TouchController();
void init_Timer();
void init_storage();
void init_can1();
void init_wifi();
int process_CAN1();
int check_RF_Error();
int check_RF_Acknowledge(int mode);
void IRAM_ATTR TIM_RF_Learn_Active_overflow();
void handleRoot();
void handleLivedaten();
void handleValues();
void handleEinstellungen();
void handleNotFound();
void handleHersteller();
int send_RemoteDrive_Request();
int send_SOC_Request();
void handleRFConReturn();
void define_CAN1_Messages();
void process_CAN_PowerData(CAN_frame_t* frame);
void process_CAN_BatteryTemperature(CAN_frame_t* frame);
void process_CAN_BatteryVoltage(CAN_frame_t* frame);
void process_CAN_Option1Commands(CAN_frame_t* frame);
void init_CAN2();
void define_CAN2_Messages();
int decodeSignal(CAN_Signal signal, CAN_frame_t* frame);
void print_NVS();
int send_CAN1_Message(int id, uint8_t* data, int len);
void write_SPI_Register(uint8_t reg, uint8_t value);
uint8_t read_SPI_Register(uint8_t reg);
void IRAM_ATTR TIM_LED_Flashing_overflow();
void process_CAN2_Message();
void handleSettingsToggle();
void process_ISR_GPIO_Expansion();
void handleDownloadLog();
void init_files();
void init_display();
UBYTE DEV_I2C_Read_Byte(UBYTE DevAddr, UBYTE RegAddr);

void setup() {
  //init_files();
  Serial.begin(115200);
  Serial.println("Setup Start");
  //init_ports();
  init_display();
  //init_GPIO_Exp_Ports();
  //init_Interrupts();
  //init_Timer();
  //init_storage();
  //init_can1();
  //init_CAN2();
  //init_wifi();
  

  Master_Mode_active = true;
  
}

void loop() {

  
  // put your main code here, to run repeatedly:
  static int loop_ctr = 0;
  if(loop_ctr == 0){
    Serial.println("Loop Start");
    loop_ctr++;
  }
  loop_ctr++;

  // Test!!!!! TODO: Remove
  Wire.requestFrom(0x15, 1); // SCL aktiv
  delay(100);

  // Dummy Werte statt CAN
  if(loop_ctr % 900 == 0){
    CAN_speed += 5.3;
    CAN_battery_temp += 0.1;
    CAN_soc -= 0.2;
    if(CAN_speed > 120.0){CAN_speed = 0.0;}
    if(CAN_battery_temp > 45.0){CAN_battery_temp = 20.0;}
    if(CAN_soc < 0.0){CAN_soc = 100.0;}
  }
  if(loop_ctr % 5000 == 0){
    //print_NVS();
  }

  // Handle Interrupt by GPIO Expansion
  if(ISR_GPIO_Exp_Flag == true){
    process_ISR_GPIO_Expansion();
    ISR_GPIO_Exp_Flag = false;
  }

  // Polling for Interrupt Flags set by GPIO Expansion ***************************************************************************
  if(ESP_storage.getInt("RF_enable", FALSE) == TRUE){
    if(ISR_RX_2_Flag == true){
      // The Microcontroller has recieved a Signal, which indicates, that a remote drive Request was sent by the RF Control
      // The Signal must now be transmitted to the VCU
      send_RemoteDrive_Request();
    }
    if(ISR_RX_3_Flag == true){
      // The Microcontroller has recieved a Signal, which indicates, that a SOC Request was sent by the RF Control
      // The Signal must now be transmitted to the VCU
      send_SOC_Request();
    }
    if(ISR_RX_4_Flag == true && ID_cur_state == OFF){
      // The Microcontroller has recieved a Signal, which indicates, that a Identification Request was sent by the RF Control
      // The Status LED must now be switched on as long as the button is pressed
      ID_cur_state = ON;
      LED_cur_state = ON;
      Status_LED_ON();
    }
    if(ISR_RX_4_Flag == false && ID_cur_state == ON){
      // / The Microcontroller has recieved a Signal, which indicates, that a Identification Request has ended
      // The Status LED must now be switched off as the button is no longer pressed
      ID_cur_state = OFF;
      LED_cur_state = OFF;
      Status_LED_OFF();
    }
    if(ISR_RF_Error_CTR > 0){
      check_RF_Error();
    }
  }
  
  if(ESP_storage.getInt("LED_enable", FALSE) == TRUE){
    if(ISR_LED_Signal_Flag == true && LED_cur_state == OFF && ID_cur_state == OFF){
    // The Microcontroller has recieved a Signal from VCU, it has to activate Status LED
    // The Status LED must now be switched on as long as the signal is active
    // This Signal low priority compared to Identification via RF Control
      LED_cur_state = ON;
      Status_LED_ON();
    }
    if(ISR_LED_Signal_Flag == false && LED_cur_state == ON && ID_cur_state == OFF){
    // The Microcontroller has recieved a Signal from VCU, Status LED activation has ended
    // The Status LED must now be switched off as the signal is no longer acive
    // This Signal low priority compared to Identification via RF Control
      LED_cur_state = OFF;
      Status_LED_OFF();
    }
  }
  

  // End of polling for Interrupts by GPIO expansion *****************************************************************************

  // Polling for Interrupts set by Touch Display Controller **********************************************************************
  if(ESP_storage.getInt("RF_enable", FALSE) == TRUE && ESP_storage.getInt("Display_enable", FALSE) == TRUE){
    // RF Control is enabled and CAN Transmission of RF Control Signals is enabled
    // Check if Learn Mode for RF Control was activated by Touch Display
    if(ISR_Learn_RF_Flag == true){
      // 
      if(ISR_Learn_RF_Mode > 0 && Learn_RF_Active_Flag == false){
        int rv = learn_RFControl(ISR_Learn_RF_Mode);

        if(rv==ERROR){Serial.println("activating Pairing Mode failed");}

        // repeat until Signal was transmitted to RF Controller successfully
        if(rv == ISR_Learn_RF_Mode){
          Learn_RF_Active_Flag = true;
        }

        timerWrite(TIM_RF_Learn_Active, 0);                 // Reset Timer Counter
        timerAlarmEnable(TIM_RF_Learn_Active);              // Activate Timer
      }
      // Check ACK and reset Learning_Flag if ACK is received
      // Learning Mode may remain active
      int rv = check_RF_Acknowledge(ISR_Learn_RF_Mode);
    }
  }

  // End of polling for Interrupts set by Touch Controller Display *********************************************************
  
  
  // Processing incoming CAN1 Messages *****************************************************************************************
  if(ESP_storage.getInt("Display_enable", FALSE) == TRUE || ESP_storage.getInt("Wifi_enable", FALSE) == TRUE){
    // CAN1 is only processed if Display or Wifi Communication is enabled
    if(CAN1_active == true){
      process_CAN1();
    }
  }
  // End of processing incoming CAN1 Messages ***********************************************************************

  // Polling for incoming CAN2 Messages *****************************************************************************************
  if(CAN2_Message_Flag == true){
    // A CAN2 Message was received, process it
    process_CAN2_Message();
  }

  // End of polling for incoming CAN2 Messages ***********************************************************************

  // Polling for HTTP Requests *************************************************************************************************
  kart_server.handleClient();
  
}

// put function definitions here *************************************************************************************************

// Init Ports
void init_ports(){

  Serial.println("Init Ports");
  // Output
  pinMode(SPI_CS_CAN2_PIN, PIN_OUTPUT);
  pinMode(SPI_CS_DISPLAY_PIN, PIN_OUTPUT);
  pinMode(SPI_CS_FLASH_PIN, PIN_OUTPUT);
  pinMode(SPI_RST_LCD_PIN, PIN_OUTPUT);
  pinMode(SPI_RST_TP_PIN, PIN_OUTPUT);

  // Input
  pinMode(SPI_INT_CAN2_PIN, PIN_INPUT);
  pinMode(SPI_INT_TP_PIN, PIN_INPUT);
  pinMode(INT_PE_PIN, PIN_INPUT);

  // UART
  MicroUSB.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // SPI
  my_SPI.begin(SPI_CLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

  //PWM For Display Backlight
  analogWrite(LCD_BL_PIN, 140);
}

int init_GPIO_Exp_Ports(){

  Serial.println("Init GPIO Expansion");
  // Initialize GPIO Expansion

  // SET Bank for addressing
  int rv = GPIO_Exp_WriteBit(0x05, 7, HIGH);
  if(rv == ERROR){
    Serial.println("Setting Bank for address failed");
    return ERROR;
  }
  else{
    Serial.println("Setting Bank for address successful");
  }
  
  rv = 0;
  // set Pin Direction
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 0, PIN_INPUT); // LED Learn Mode
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 1, PIN_INPUT); // LED Signal from VCU
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 2, PIN_INPUT); // RC Errors
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 3, PIN_INPUT); // RC_Recieve_CH2
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 5, PIN_INPUT); // RC Receive CH3
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRA, 7, PIN_INPUT); // RC Receive CH4
  if(rv != 6){Serial.println("Error in init Ports Bank A"); return ERROR;}
  else{Serial.println("Init Ports Bank A successful");}

  rv = 0;
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 0, PIN_OUTPUT); // LED Pin
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 1, PIN_OUTPUT); // Activate RC Learn Mode
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 2, PIN_OUTPUT); // RFID SPI Chip Select
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 3, PIN_OUTPUT); // RFID SPI Reset 
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 4, PIN_OUTPUT); // RC Transmit CH2
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 5, PIN_OUTPUT); // RC Transmit CH3
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 6, PIN_OUTPUT); // CAN1 Silent Mode
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IODIRB, 7, PIN_OUTPUT); // CAN2 Silent Mode
  if(rv != 8){Serial.println("Error in init Ports Bank B"); return ERROR;}
  else{Serial.println("Init Ports Bank B successful");}

  // set PullUps where needed
  rv = 0;
  rv += GPIO_Exp_WriteRegister(GPIO_EXP_GPPUA, 0x00); // No PullUps needed
  rv += GPIO_Exp_WriteRegister(GPIO_EXP_GPPUB, 0x00); // no PullUps needed
  if(rv != 2){Serial.println("Error in set PullUp Resistors"); return ERROR;}else{Serial.println("Set PullUp Resistors successful");}

  // set Interrupt Settings 
  rv = 0;
  // Enable Interrupts for Pins 7, 5, 3, 2, 1, 0
  rv += GPIO_Exp_WriteRegister(GPIO_EXP_GPINTENA, 0xAF);

  // Define Comparison Values for Pins to throw Interrupts
  // Interrupt is set, if opposite value occurd
  rv += GPIO_Exp_WriteBit(GPIO_EXP_DEFVALA, 0, HIGH); // low-active Signal
  rv += GPIO_Exp_WriteBit(GPIO_EXP_DEFVALA, 2, HIGH); // low-active Signal

  rv += GPIO_Exp_WriteBit(GPIO_EXP_DEFVALA, 3, LOW);
  rv += GPIO_Exp_WriteBit(GPIO_EXP_DEFVALA, 5, LOW);

  // Set Interrupt Control Regsiter to comparison against previous Values or DEFVAL Regsister
  // Pins 5, 3, 2 and 0 are compared against defVal (set to 1)
  // Pins 7 and 1 are compared against previous Value (set to 0)
  // -> 0b 0010 1101
  rv += GPIO_Exp_WriteRegister(GPIO_EXP_INTCONA, 0x2D);

  // Set Interrupt polarity to High_active
  rv += GPIO_Exp_WriteBit(GPIO_EXP_IOCONA, 1, HIGH);

  if(rv != 7){Serial.println("Error in init Interrupts for GPIO Expansion"); return ERROR;}else{Serial.println("Init Interrupts for GPIO Expansion successful");}

  // Initial Value
  rv = 0;
  rv += GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 1, HIGH);
  if(rv != 1){Serial.println("Error in init Interrupts for GPIO Expansion"); return ERROR;}else{Serial.println("Init Interrupts for GPIO Expansion successful");}

  return SUCCESS;
}

void init_Interrupts(){

  Serial.println("Init Interrupts");
  // Init Interrupts

  attachInterrupt(digitalPinToInterrupt(INT_PE_PIN), ISR_GPIO_Expansion, RISING);
  attachInterrupt(digitalPinToInterrupt(SPI_INT_CAN2_PIN), ISR_CAN2, RISING);
  attachInterrupt(digitalPinToInterrupt(SPI_INT_TP_PIN), ISR_TouchController, RISING);
}

void init_Timer(){

  Serial.println("Init Timer");

  // initardwaretimer

  /***************  Timer 0 - RF Learn Active detection ********************************************/
  // Timer 0, Prescaler 8000 -> 1 tick = 0.1 ms (bei 80 MHz APB), countUp
  TIM_RF_Learn_Active = timerBegin(0, 8000, true); 

  // init Interrupt for Timer overflow
  timerAttachInterrupt(TIM_RF_Learn_Active, TIM_RF_Learn_Active_overflow, true); // react on rising Edge

  // set Timer-Alarm: 3.000 ms = 3 Sekunden
  timerAlarmWrite(TIM_RF_Learn_Active, 3000, false); // false = one-shot, true = auto-reload
  timerAlarmDisable(TIM_RF_Learn_Active);            // Timer will be activated by ISR
  /*****************************************************************************************************/

  /*************** Timer 1 - LED Flashing via CAN ******************************************************/
  // Timer 1, Prescaler 8000 -> 1 tick = 0.1 ms (bei 80 MHz APB), countUp
  TIM_LED_Flashing = timerBegin(1, 8000, true);

  // init Interrupt for Timer overflow
  timerAttachInterrupt(TIM_LED_Flashing, TIM_LED_Flashing_overflow, true); // react on rising Edge

  // set Timer-Alarm: 500 ms = 0.5 Sekunden -> Period = 1 sec
  timerAlarmWrite(TIM_LED_Flashing, 500, true); // true = auto-reload
  timerAlarmDisable(TIM_LED_Flashing);            // Timer will be activated by CAN-receive
  /*****************************************************************************************************/
}

void init_storage(){

  Serial.println("Init Storage");

  // save settings
  ESP_storage.begin("settings", false);

  if(ESP_storage.getInt("RF_enable", -1) == -1){
    // Variable is not stored at the moment
    // Init Variable
    ESP_storage.putInt("RF_enable", true);
  }
  if(ESP_storage.getInt("Display_enable", -1) == -1){
    // Variable is not stored at the moment
    // Init Variable
    ESP_storage.putInt("Display_enable", true);
  }
  if(ESP_storage.getInt("WiFi_enable", -1) == -1){
    // Variable is not stored at the moment
    // Init Variable
    ESP_storage.putInt("WiFi_enable", true);
  }
  if(ESP_storage.getInt("RFID_enable", -1) == -1){
    // Variable is not stored at the moment
    // Init Variable
    ESP_storage.putInt("RFID_enable", true);
  }
  if(ESP_storage.getInt("LED_enable", -1) == -1){
    // Variable is not stored at the moment
    // Init Variable
    ESP_storage.putInt("LED_enable", true);
  }
  if(ESP_storage.getInt("RF_CAN_en", -1) == -1){
    // Variable is not stored at the moment
    // Init Variable
    ESP_storage.putInt("RF_CAN_en", false);
  }
  if(ESP_storage.getInt("Display_off", -1) == -1){
    // Variable is not stored at the moment
    // Init Variable
    ESP_storage.putInt("Display_off", true);
  }
  if(ESP_storage.getString("WIFI_Name", "unknown") == "unknown"){
    // Variable is not stored at the moment
    // Init Variable
    ESP_storage.putString("WIFI_Name", "SMS REVO SL");
  }
  if(ESP_storage.getString("WIFI_Password", "unknown") == "unknown"){
    // Variable is not stored at the moment
    // Init Variable
    ESP_storage.putString("WIFI_Password", "SMS REVO SL");
  }
  String wifi_pass = ESP_storage.getString("WIFI_Password");
  if (wifi_pass.length() < 8) {
    Serial.println("Passwort ist zu kurz!");
    Serial.println("Setze Standard Passwort.");
    // Passwort ist to short, set Standard Password

    ESP_storage.putString("WIFI_Password", "SMS REVO SL");
  } 
  
}

void init_can1(){

  Serial.println("Init CAN");

  CAN_cfg.rx_pin_id = gpio_num_t(CAN1_RX_PIN);
  CAN_cfg.tx_pin_id = gpio_num_t(CAN1_TX_PIN);
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.rx_queue = xQueueCreate(40, sizeof(CAN_frame_t));
  if(ESP32Can.CANInit() == ESP_OK){
    CAN1_active = true;
    Serial.println("CAN1 started");
  }
  else{
    CAN1_active = false;
    Serial.println("Error starting CAN1");
  }

  define_CAN1_Messages();
}

void init_wifi(){

  Serial.println("Init WIFI");

  // Start Wifi as Accesspoint
  // set static IP-Address
  if(!WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("Fehler beim Setzen der AP-Konfiguration");
  }

  // Access Point starten
  if(WiFi.softAP(ESP_storage.getString("WIFI_Name", "SMS REVO SL"), ESP_storage.getString("WIFI_Password"))) {
    // Debugging Infos
    Serial.println("Access Point gestartet");
    Serial.print("IP-Adresse des Access Points: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Fehler beim Start des Access Points");
  }

  // set URLs
  kart_server.on("/", handleRoot);
  kart_server.on("/livedaten", handleLivedaten);
  kart_server.on("/einstellungen", handleEinstellungen);
  kart_server.on("/values", handleValues);
  kart_server.on("/RF_Connect_Return", handleRFConReturn);
  kart_server.on("/hersteller", HTTP_GET, handleSettingsToggle);   // Für das Laden der Seite
  kart_server.on("/hersteller", HTTP_POST, handleSettingsToggle);  // Für die AJAX-POSTs
  kart_server.on("/downloadLog", HTTP_GET, handleDownloadLog);
  kart_server.onNotFound(handleNotFound);

  // Start Webserver
  kart_server.begin();
}

void init_files(){
  Serial.println("Init Filesystem");

  if(!SPIFFS.begin(false)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    if (SPIFFS.format()) {
      // Format SPIFFS on first setup
      Serial.println("Format successful!");
      SPIFFS.begin(true);  
  } else {
    Serial.println("Format failed!");
  }
  }
  else{
    Serial.println("SPIFFS mounted successfully");
  }

  // Create or open a file
  logFile = SPIFFS.open("/Error.log", FILE_WRITE);
  if (!logFile) {
    Serial.println("File Generation failed");
    return;
  }
  else{
    Serial.println("File opened successfully");
  }

  // Etwas Text hineinschreiben
  logFile.println("=== Neue Logdatei gestartet ===");
  logFile.println("Fehlercode: 42");
  logFile.println("System neu gestartet.");
  logFile.close();
}

void init_display(){

  /*
  Touch_1IN28_XY XY;
    XY.mode = 0;
    XY.x_point = 0;
    XY.y_point = 0;
    //Config_Init();
    LCD_Init();
    if(Touch_1IN28_init(XY.mode) == true)
        Serial.println("OK!");
    else
        Serial.println("NO!");

    LCD_SetBacklight(1000);
    Paint_NewImage(LCD_WIDTH, LCD_HEIGHT, 0, BLUE);
    Paint_Clear(WHITE);
    attachInterrupt(1,ISR_TouchController,LOW);
    pinMode(TP_INT_PIN, INPUT_PULLUP);
    Paint_DrawString_EN(35, 90, "Gesture test", &Font20, BLACK, WHITE);
    Paint_DrawString_EN(10, 120, "Complete as prompted", &Font16, BLACK, WHITE);
    DEV_Delay_ms(500);
    */
    Wire.begin(27, 33, 100000);

    delay(1000);

    DEV_I2C_Read_Byte(0x15, 0x01);
}

int process_CAN1(){
  // Process incoming CAN1 Messages
  CAN_frame_t rx_frame;

  // Check for IDs and process Data based on 

  // Versuchen, eine Nachricht aus der Queue zu holen
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 0) == pdTRUE) {
    
    // Frame-Infos
    Serial.print("Empfangen: ID=0x");
    Serial.print(rx_frame.MsgID, HEX);
    Serial.print(" DLC=");
    Serial.print(rx_frame.FIR.B.DLC);
    Serial.print(" Daten=");

    // Datenbytes ausgeben
    for (int i = 0; i < rx_frame.FIR.B.DLC; i++) {
      Serial.print(rx_frame.data.u8[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    int identifier = rx_frame.MsgID;

    if(Power_Data.id == identifier){
      // Power Data Message
      process_CAN_PowerData(&rx_frame);
    }
    else if(Battery_Temperature.id == identifier){
      // Battery Temperature Message
      process_CAN_BatteryTemperature(&rx_frame);
    }
    else if(Battery_Voltage.id == identifier){
      // Battery Voltage Message
      process_CAN_BatteryVoltage(&rx_frame);
    }
    else if(Option1_Commands.id == identifier){
      // VCU Commands Message
      process_CAN_Option1Commands(&rx_frame);
    }
    else{
      // Unknown Message
      Serial.println("Unknown CAN Message");
      return 0;
    }
  }

  return 0;
}

void init_CAN2(){

  int ctr = 0;
  int mode = 0;

  Serial.println("Init CAN2");
  bool connected = false;

  // Initialize CAN2 Module via SPI
  // Set CAN2 Module to Config Mode
  write_SPI_Register(CAN2_CANCTRL,0x80); // Set Config Mode
  delay(50);

  // Check for Controller in Config Mode
  // Read CANSTAT Register [7:5] -> Mode
  while(ctr<100 && connected == false){
    mode = read_SPI_Register(CAN2_CANSTAT) >> 5;
    if(mode == 0x04){
      Serial.println("CAN2 in Config Mode");
      connected = true;
    }
    else{
      // Error Handling - Try Multiple Times
      ctr++;
      Serial.println("Error: CAN2 not in Config Mode");
    }
  }

  if(connected == true){
    // Set Bitrate to 500 kbps
    // For 8MHz Clock
    write_SPI_Register(CAN2_CNF1, 0x00); // SJW=1, BRP=0 -> 500 kbps
    write_SPI_Register(CAN2_CNF2, 0x89); // BTLMODE=1, SAM=0, PHSEG1=2, PRSEG=2
    write_SPI_Register(CAN2_CNF3, 0x02); // PHSEG2=3

    // Enable Interrupts
    write_SPI_Register(CAN2_CANINTE, 0xFF); // Enable all interrupts

    // Set Normal Mode
    write_SPI_Register(CAN2_CANCTRL, 0x00); // Set Normal Mode
    delay(50);

    // Check for Controller in Normal Mode
    ctr = 0;
    while(ctr<100 && CAN1_active == false){
      mode = read_SPI_Register(CAN2_CANSTAT) >> 5;
      if(mode == 0x00){
        Serial.println("CAN2 in Normal Mode");
        CAN1_active = true;
      }
      else{
        //Error Handling - Try Multiple Times
        ctr++;
        Serial.println("Error: CAN2 not in Normal Mode");
      }
    }
  }

  define_CAN2_Messages();
}

// Interrupts
void ISR_GPIO_Expansion(){
  // Interrupt Service Routine for GPIO Expansion Interrupts
  ISR_GPIO_Exp_Flag = true;
}

void process_ISR_GPIO_Expansion(){
  // Read Interrupt Pending Register
  // Set Flags for Interrupts


  int flags = GPIO_Exp_ReadRegister(GPIO_EXP_INTFA);
  int rv = GPIO_Exp_ReadRegister(GPIO_EXP_INTCAPA);

  if(flags != ERROR){
    
    // Check for every single bit
    if((flags >> 0) & 0x01){
      // Interrupt ocured for Bit 0
      // RF Controller Pairing Mode Acknowledge
      ISR_Learn_LED_CTR += 1; 
      timerWrite(TIM_RF_Learn_Active, 0);
    }
    if((flags >> 1) & 0x01){
      // Interrupt ocured for Bit 1
      // LED Signal recieved from VCU
      ISR_LED_Signal_Flag = true;
    }
    if((flags >> 2) & 0x01){
      // Interrupt ocured for Bit 2
      // RF Controller Error received
      ISR_RF_Error_CTR += 1;
    }
    if((flags >> 3) & 0x01){
      // Interrupt ocured for Bit 3
      // Remote Drive Command received from RF Module 
      ISR_RX_2_Flag = true;
    }
    if((flags >> 4) & 0x01){
      // Interrupt ocured for Bit 4
      // Not used for interrupt
    }
    if((flags >> 5) & 0x01){
      // Interrupt ocured for Bit 5
      // SOC Command received from RF Module 
      ISR_RX_3_Flag = true;
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
        ISR_RX_4_Flag = true;
      }
      else{
        // Status LED Switch off
        ISR_RX_4_Flag = false;
      }
    }

  }
}

void IRAM_ATTR ISR_CAN2(){
  // Interrupt Service Routine for CAN2 Messages
  // Set Flags for Interrupts
  CAN2_Message_Flag = true;
}

void process_CAN2_Message(){
  // Check for Message in Buffer 0
  // Check RX Buffer 0 Full Flag [bit:0]
  if(read_SPI_Register(CAN2_CANINTF) & 0x01){
    // Clear Interrupt Flag
    write_SPI_Register(CAN2_CANINTF, 0x00);

    // Message in Buffer 0
    // Read Identifier from Controller
    int identifier = (read_SPI_Register(CAN2_RXB0SIDH) << 3) | (read_SPI_Register(CAN2_RXB0SIDL) >> 5);
    if(identifier == RFID_Data.id){
      // RFID Message recieved
      // Read Data Length Code
      int dlc = read_SPI_Register(CAN2_RXB0DLC) & 0x0F;

      // Read Data Bytes
      uint64_t data = 0;
      for(int i=0; i<dlc; i++){
        data = (data << (i * 8)) | read_SPI_Register(CAN2_RXB0D0 + i);
      }
      // Process Data
      // generate mask for data
      uint64_t mask = ~(0xFFFFFFFFFFFFFFFF << Customer_ID.length);

      // Extract Signals
      uint64_t Customer_identifier = (data >> Customer_ID.start_bit) & mask;

      // Handle data
      if(Customer_identifier == MASTER_ID){
        // Master ID detected
        // Activate Master Mode
        Master_Mode_active = true;
      }
      else{
        Master_Mode_active = false;
      }
    }
    else{
      // Unknown Message
      Serial.println("Unknown CAN2 Message");
    }

  }
}

void IRAM_ATTR ISR_TouchController(){
  // Interrupt Service Routine activated by TouchController
  // Set Flags for Interrupts
  Touch_ISR_Flag = true;
}

// Check on RF Controll for Errors or ACK and process recieved Data
int check_RF_Error(){
  // Error Data
  // Function reads Signals with are send by RF Module to toggle a Error LED on the PCB
  // Error Data is defined by Frequency of the Signal
  // Every Peak trigger the ISR, in with a counter is running
  // Error will be shown on Display

  /* Error Data Overview
  Lern- oder Löschmodus: 
  Eintrag konnte aus Liste der eingelernten Sender nicht entfernt werden: Blinkt 2x
  Liste der eingelernten Sender ist voll: Blinkt 3x
  Sender wurde bereits eingelernt: Blinkt 2x
  */

  if(RF_Error_Active == false){
    RF_Error_Active = true;
    time_wait_Error = millis() + 500;
  }
  unsigned long now = millis();
  if(now >= time_wait_Error){
    // Read Error Code
    if(Learn_RF_Active_Flag == true){
      if(ISR_RF_Error_CTR == 2 && ISR_Learn_RF_Mode >= 5){
        // Entry could not be Errased from list
        // TODO: Write to Display
      }
      if(ISR_RF_Error_CTR == 2 && ISR_Learn_RF_Mode >= 4){
        // Entry already found in list
        // TODO: Write to Display
      }
      if(ISR_RF_Error_CTR == 3){
        // Entry could not be added - maximum reached
        // TODO: Write to Display
      }
    }

    RF_Error_Active = false;
    ISR_RF_Error_CTR = 0;
    return SUCCESS;
  }
  else{
    // waiting for Error
    return 0;
  }

}

int check_RF_Acknowledge(int mode){
  /** 
   *  @brief  Acknowledge Data
   *          Function reads Signals with are send by RF Module to toggle a Acknowledge LED on the PCB
   *          Acknowledge Data is defined by Frequency of the Signal
   *          Every Peak trigger the ISR, in with a counter is running
   * 
   *  @return 1 if valid ACK was captured, 0, if no valid ACK was captured
   */

  /* 
  Acknowledge Data Overview
  Selection learn mode I: Light interrupts 1x every 2s
  Selection learn mode II: Light interrupts 2x every 2s
  Selection learn mode III: Light interrupts 3x every 2s
  Selection learn mode IV: Light interrupts 4x every 2s
  Selection erase mode I: Flashes permanently
  */

  static unsigned long time_wait_ACK = 0;

  if(Learn_RF_ACK_Waiting == false){
    // Start waiting for ACK, only operate once on first call
    ISR_Learn_LED_CTR = 0;
    time_wait_ACK = millis() + 2000;  // set timestamp for waiting time
    Learn_RF_ACK_Waiting = true;
  }
  unsigned long now = millis();
  if(now >= time_wait_ACK){
    // Timer is over, count captured Signals
    if(mode == ISR_Learn_LED_CTR || ISR_Learn_LED_CTR >= 5){
      // Captured valid ACK
      time_wait_ACK = false;
      ISR_Learn_RF_Flag = false;
      Learn_RF_ACK_Waiting = false;
      return SUCCESS;
    }
    else{
      // wait for valid ACK
      time_wait_ACK = false;
      return 0;
    }
  }
  else{
    // Waiting for Error
    return 0;
  }
}

void IRAM_ATTR TIM_RF_Learn_Active_overflow() {
  Learn_RF_Active_Flag = false;   // Reset Learning Mode Active on Timer overflow
  timerAlarmDisable(TIM_RF_Learn_Active);       // stop Timer after first overflow
}

// Webserver Handles
void handleRoot() {
  // Return Main Page
  // HTML-Seite im Flash-Speicher ablegen
  String MAIN_page = "";

  if(ESP_storage.getInt("WiFi_enable", FALSE) == FALSE){
    MAIN_page =
      "<!DOCTYPE html><html lang=\"de\"><head>"
      "<meta charset=\"UTF-8\">"
      "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
      "<title>SMS REVO SL</title>"
      "<style>"
      "body { margin:0; display:flex; justify-content:center; align-items:center; height:100vh; background:#f2f2f2; font-family:Arial,sans-serif; }"
      ".card { background:#fff; padding:20px 30px; border-radius:12px; text-align:center; box-shadow:0 4px 12px rgba(0,0,0,0.15); max-width:400px; width:100%; }"
      "h1 { color:red; margin-bottom:20px; }"
      "</style></head><body><div class=\"card\">"
      "<h1>Hier gibt es nix zu sehen</h1>"
      "</div></body></html>";
  }
  else{
    MAIN_page =
    "<!DOCTYPE html>"
    "<html lang=\"de\">"
    "<head>"
      "<meta charset=\"UTF-8\">"
      "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
      "<title>SMS REVO SL</title>"
      "<style>"
        "body { margin:0; display:flex; justify-content:center; align-items:center; height:100vh; background:#f2f2f2; font-family:Arial,sans-serif; }"
        ".card { background:#fff; padding:20px 30px; border-radius:12px; text-align:center; box-shadow:0 4px 12px rgba(0,0,0,0.15); }"
        "h1 { color:red; margin-bottom:20px; }"
        "a.btn { display:inline-block; margin:8px; padding:12px 18px; border-radius:8px; text-decoration:none; color:white; font-weight:bold; }"
        "a.green { background:green; }"
        "a.gray { background:gray; }"
        "a.blue { background:blue; }"
      "</style>"
    "</head>"
    "<body>"
      "<div class=\"card\">"
        "<h1>SMS REVO SL</h1>"
        "<a class=\"btn green\" href=\"/livedaten\">Livedaten</a>"
        "<a class=\"btn blue\"  href=\"/einstellungen\">Einstellungen</a>"
      "</div>"
    "</body>"
    "</html>";
  }

  kart_server.send(200, "text/html", MAIN_page);
}

void handleNotFound() {
  kart_server.send(404, "text/plain", "404: Not Found");
}

void handleLivedaten() {

  String LIVEDATEN_page = "";

  if(ESP_storage.getInt("WiFi_enable", FALSE) == FALSE){
    LIVEDATEN_page =
      "<!DOCTYPE html><html lang=\"de\"><head>"
      "<meta charset=\"UTF-8\">"
      "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
      "<title>SMS REVO SL - Livedaten</title>"
      "<style>"
      "body { margin:0; display:flex; justify-content:center; align-items:center; height:100vh; background:#f2f2f2; font-family:Arial,sans-serif; }"
      ".card { background:#fff; padding:20px 30px; border-radius:12px; text-align:center; box-shadow:0 4px 12px rgba(0,0,0,0.15); max-width:400px; width:100%; }"
      "h1 { color:red; margin-bottom:20px; }"
      "a.btn { display:inline-block; margin:8px; padding:12px 18px; border-radius:8px; text-decoration:none; color:white; font-weight:bold; }"
      "a.gray { background:gray; }"
      "</style></head><body><div class=\"card\">"
      "<h1>Hier gibt es nix zu sehen</h1>"


      "<br><a href=\"/\" class=\"btn gray\">Zurück</a>"
      "</div></body></html>";
  }
  else{
    LIVEDATEN_page = 
    "<!DOCTYPE html><html lang='de'><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<title>Livedaten - SMS REVO SL</title>"
    "<style>"
    "body{margin:0;display:flex;justify-content:center;align-items:center;height:100vh;background:#f2f2f2;font-family:Arial,sans-serif;}"
    ".card{background:#fff;padding:20px 30px;border-radius:12px;text-align:center;box-shadow:0 4px 12px rgba(0,0,0,0.15);max-width:400px;width:100%;}"
    "h1{color:red;margin-bottom:20px;}"
    ".databox{background:#f9f9f9;border:1px solid #ddd;border-radius:8px;padding:15px;margin:10px 0;text-align:left;font-size:1.4em;}"
    ".label{font-weight:bold;display:block;margin-bottom:6px;}"
    ".value{color:#333;}"
    "a.btn{display:inline-block;margin:12px;padding:12px 18px;border-radius:8px;text-decoration:none;color:white;font-weight:bold;}"
    "a.gray{background:gray;}"
    "</style>"
    "<script>"
    "async function updateValues(){"
    "  let response = await fetch('/values');"
    "  let data = await response.json();"
    "  document.getElementById('speed').innerText = data.speed + ' km/h';"
    "  document.getElementById('soc').innerText = data.soc + ' %';"
    "  document.getElementById('temp').innerText = data.temp + ' °C';"
    "}"
    "setInterval(updateValues,1000);"
    "</script>"
    "</head><body>"
    "<div class='card'>"
    "<h1>Livedaten</h1>"
    "<div class='databox'><span class='label'>Geschwindigkeit</span><span class='value' id='speed'>--</span></div>"
    "<div class='databox'><span class='label'>State of Charge</span><span class='value' id='soc'>--</span></div>"
    "<div class='databox'><span class='label'>Akkutemperatur</span><span class='value' id='temp'>--</span></div>"
    "<a class='btn gray' href='/'>Zurück</a>"
    "</div>"
    "</body></html>";
  }
  kart_server.send(200, "text/html", LIVEDATEN_page);
}

void handleValues() {
  float speed, soc, bat_temp;
  noInterrupts();
  speed = CAN_speed;
  soc = CAN_soc;
  bat_temp = CAN_battery_temp;
  interrupts();

  String json = "{";
  json += "\"speed\":" + String(speed, 1) + ",";
  json += "\"soc\":" + String(soc, 1) + ",";
  json += "\"temp\":" + String(bat_temp, 1);
  json += "}";

  kart_server.send(200, "application/json", json);
}

void handleEinstellungen() {

  String EINSTELLUNGEN_page = "";

  if(ESP_storage.getInt("WiFi_enable", FALSE) == FALSE){
    EINSTELLUNGEN_page =
      "<!DOCTYPE html><html lang=\"de\"><head>"
      "<meta charset=\"UTF-8\">"
      "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
      "<title>SMS REVO SL - Einstellungen</title>"
      "<style>"
      "body { margin:0; display:flex; justify-content:center; align-items:center; height:100vh; background:#f2f2f2; font-family:Arial,sans-serif; }"
      ".card { background:#fff; padding:20px 30px; border-radius:12px; text-align:center; box-shadow:0 4px 12px rgba(0,0,0,0.15); max-width:400px; width:100%; }"
      "h1 { color:red; margin-bottom:20px; }"
      "a.btn { display:inline-block; margin:8px; padding:12px 18px; border-radius:8px; text-decoration:none; color:white; font-weight:bold; }"
      "a.gray { background:gray; }"
      "</style></head><body><div class=\"card\">"
      "<h1>Hier gibt es nix zu sehen</h1>"


      "<br><a href=\"/\" class=\"btn gray\">Zurück</a>"
      "</div></body></html>";
  }
  else{
    if (kart_server.method() == HTTP_POST) {
      // Werte aus Formular speichern
      if (kart_server.hasArg("wifi_name")) {
        ESP_storage.putString("WIFI_Name", kart_server.arg("wifi_name"));
      }
      if (kart_server.hasArg("wifi_password")) {
        ESP_storage.putString("WIFI_Password", kart_server.arg("wifi_password"));
      }

      if (kart_server.hasArg("display_off")) {
        ESP_storage.putInt("Display_off", true);
      } else {
        ESP_storage.putInt("Display_off", false);
      }

      // Spezialaktionen über Buttons
      if (kart_server.hasArg("action")) {
        String act = kart_server.arg("action");
        if (act == "remote_bind") {
          // Start learning mode for RF Controller
          Serial.println("bind Remote Control");
          int rv = learn_RFControl(1); // Start learning mode for RF Controller
          if(rv==ERROR){
            Serial.println("Pairing Controller failed");
            // Write to Website 
            RF_Connect_Return = "Pairing Controller failed";
          }
          else{
            Serial.println("Pairing Controller successful");
            // Write to Website 
            RF_Connect_Return = "Pairing Controller successful";
          }
        }
        else if (act == "remote_unbind") {
          // Delete last binded RF Controller
          Serial.println("Funkfernbedienung löschen");
          int rv = learn_RFControl(5); // Delete last binded RF Controller
          if(rv==ERROR){
            Serial.println("deleting last RF Controller failed");
            // Write to Website 
            RF_Connect_Return = "deleting last RF Controller failed";
          }
          else{
            Serial.println("deleting last RF Controller successful");
            // Write to Website 
            RF_Connect_Return = "deleting last RF Controller successful";
          }
        }      
        else if (act == "remote_unbind_all") {
          // Delete all binded RF Controllers
          Serial.println("Alle Funkfernbedienungen löschen");
          int rv = learn_RFControl(6); // Delete every binded RF Controller
          if(rv==ERROR){
            Serial.println("deleting all RF Controllers failed");
            // Write to Website 
            RF_Connect_Return = "deleting all RF Controllers failed";
          }
          else{
            Serial.println("deleting all RF Controllers successful");
            // Write to Website
            RF_Connect_Return = "deleting all RF Controllers successful";
          }
        }
      }
      
      kart_server.sendHeader("Location", "/einstellungen");
      kart_server.send(303, "text/plain", "Werte gespeichert, weiterleiten...");
      return;
      
    }

    // Aktuellen Status von Display_off holen
    bool displayOff = ESP_storage.getInt("Display_off", false);

    EINSTELLUNGEN_page =
      "<!DOCTYPE html><html lang=\"de\"><head>"
      "<meta charset=\"UTF-8\">"
      "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
      "<title>Einstellungen - SMS REVO SL</title>"
      "<style>"
      "body { margin:0; display:flex; justify-content:center; align-items:center; height:100vh; background:#f2f2f2; font-family:Arial,sans-serif; }"
      ".card { background:#fff; padding:20px 30px; border-radius:12px; text-align:center; box-shadow:0 4px 12px rgba(0,0,0,0.15); max-width:400px; width:100%; }"
      "h1 { color:red; margin-bottom:20px; }"
      "a.btn { display:inline-block; margin:8px; padding:12px 18px; border-radius:8px; text-decoration:none; color:white; font-weight:bold; }"
      "a.green { background:green; }"
      "a.gray { background:gray; }"
      "a.download { background:blue; }"
      "form { margin-top:20px; }"
      "input[type=text], input[type=password] { padding:8px; margin:8px 0; border-radius:6px; border:1px solid #ccc; width:80%; }"
      "input[type=submit], button { padding:10px 20px; border-radius:8px; border:none; background:green; color:white; font-weight:bold; cursor:pointer; margin:6px; }"
      "input[type=submit]:hover, button:hover { background:darkgreen; }"
      ".switch { position:relative; display:inline-block; width:50px; height:24px; }"
      ".switch input { opacity:0; width:0; height:0; }"
      ".slider { position:absolute; cursor:pointer; top:0; left:0; right:0; bottom:0; background-color:#ccc; transition:.4s; border-radius:24px; }"
      ".slider:before { position:absolute; content:\"\"; height:18px; width:18px; left:3px; bottom:3px; background:white; transition:.4s; border-radius:50%; }"
      "input:checked + .slider { background-color:green; }"
      "input:checked + .slider:before { transform:translateX(26px); }"
      "#RF_Connect_Return { margin-top:15px; font-weight:bold; }"
      "</style></head><body><div class=\"card\">"
      "<h1>Einstellungen</h1>"
      "<form method=\"POST\" action=\"/einstellungen\">"
      "<label for=\"wifi_name\">WIFI Name:</label><br>"
      "<input type=\"text\" id=\"wifi_name\" name=\"wifi_name\" value=\"" + ESP_storage.getString("WIFI_Name") + "\"><br>"
      "<label for=\"wifi_password\">WIFI Passwort:</label><br>"
      "<input type=\"password\" id=\"wifi_password\" name=\"wifi_password\" minlength=\"8\" required value=\"" + ESP_storage.getString("WIFI_Password") + "\"><br>"

      "<label for=\"display_off\">Display Off:</label><br>"
      "<label class=\"switch\">"
      "<input type=\"checkbox\" name=\"display_off\" " + String(displayOff ? "checked" : "") + ">"
      "<span class=\"slider\"></span>"
      "</label><br><br>"

      "<input type=\"submit\" value=\"Speichern\">"
      "<p>Netzwerk Änderungen werden beim nächsten Neustart wirksam!</p>"
      "</form>"

      "<hr>"
      "<form method=\"POST\" action=\"/einstellungen\">"
      "<button type=\"submit\" name=\"action\" value=\"remote_bind\">Funkfernbedienung verbinden</button><br>"
      "<button type=\"submit\" name=\"action\" value=\"remote_unbind\">Funkfernbedienung löschen</button><br>"
      "<button type=\"submit\" name=\"action\" value=\"remote_unbind_all\">Alle Funkfernbedienungen löschen</button><br>"

      "<div id=\"RF_Connect_Return\">warte auf Verbindungsanfrage...</div>"

      "</form>"

      "<hr>"
      "<a class=\"btn download\" href=\"/downloadLog\">📥 Logdatei herunterladen</a>"
      "<a class=\"btn gray\" href=\"/\">Zurück</a>"

      "<script>"
      "function updateRFConReturn(){"
      " fetch('/RF_Connect_Return')"
      "   .then(response => response.text())"
      "   .then(data => { document.getElementById('RF_Connect_Return').innerText = 'Verbindungsstatus: ' + data; });"
      "}"
      "setInterval(updateRFConReturn, 2000);"
      "updateRFConReturn();"
      "</script>"
      "</div></body></html>";
  }
  kart_server.send(200, "text/html", EINSTELLUNGEN_page);
}

void handleRFConReturn() {
  kart_server.send(200, "text/plain", RF_Connect_Return);
}

int send_RemoteDrive_Request(){
/**
 * @brief Depending on the System, this function sends a CAN Message to the VCU or powers the MOSFET to send the Signal for Remote Drive to VCU via wired Connection. 
 * 
 * @return 1 at Success, -1 at fail
 */

  static bool Signal_active = false;
  static int timer_end = 0;
  int rv = 0;

  if(ESP_storage.getInt("RF_CAN_en", FALSE) == FALSE){
    // Send analog Signal
    // Set Pin GPIOB 4 on Port Extension to HIGH to power the MOSFET
    if(Signal_active == false){
      Signal_active = true;
      timer_end = millis() + 300; // Set Timer for 300ms

      int rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 4, HIGH);
      if(rv == ERROR){
        Serial.println("Write failed");
        ISR_RX_2_Flag = false; // Reset Flag
        Signal_active = false;

        return ERROR;
      }
    }
    if(millis() < timer_end){
      return 0; // Wait until 300ms are over
    }

    // Reset Flag
    ISR_RX_2_Flag = false;
    Signal_active = false;

    // Set Pin GPIOB 4 on Port Extension to LOW to reset the MOSFET
    rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 4, LOW);
    if(rv == ERROR){
      Serial.println("Write failed");
      return ERROR;
    }
  }

  if(ESP_storage.getInt("RF_CAN_en", FALSE) == TRUE){
    //Send CAN-Message with RemoteDive Command to VCU
    uint8_t data[0] = {};
    data[0] = 1 << RemoteDrive_Request.start_bit; // Set SOC Bit to HIGH
    int rv = send_CAN1_Message(VCU_Commands.id, data, 1); // Send CAN Message

    if(rv == ERROR){
      Serial.println("Send CAN Message failed");
      return ERROR;
    }

    
  }

  return SUCCESS;
}

int send_SOC_Request(){
/**
 * @brief Depending on the System, this function sends a CAN Message to the VCU or powers the MOSFET to send the Signal to VCU via wired Connection
 * 
 * @return 1 at Success, -1 at fail
 */

  static bool Signal_active = false;
  static int timer_end = 0;
  int rv = 0;

  // Send analog Signal
  // Set Pin GPIOB 5 on Port Extension to HIGH to power the MOSFET
  if(Signal_active == false){
    Signal_active = true;
    timer_end = millis() + 300; // Set Timer for 300ms

    rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 5, HIGH);
    if(rv == ERROR){
      Serial.println("Write failed");
      ISR_RX_2_Flag = false; // Reset Flag
      Signal_active = false;

      return ERROR;
    }
  }

  // Wait and reset
  if(millis() < timer_end){
    return 0; // Wait until 300ms are over
  }

  // Reset Flag
  ISR_RX_2_Flag = false;
  Signal_active = false;

  // Set Pin GPIOB 5 on Port Extension to LOW to reset the MOSFET
  rv = GPIO_Exp_WriteBit(GPIO_EXP_GPIOB, 5, LOW);
  if(rv == ERROR){
    Serial.println("Write failed");
    return ERROR;
  }

  // Send CAN Message
  if(ESP_storage.getInt("RF_CAN_en", FALSE) == TRUE){
    //Send CAN-Message with SOC Request Command to VCU
    uint8_t data[0] = {};
    data[0] = 1 << SOC_Request.start_bit; // Set SOC Bit to HIGH
    int rv = send_CAN1_Message(VCU_Commands.id, data, 1); // Send CAN Message

    if(rv == ERROR){
      Serial.println("Send CAN Message failed");
      return ERROR;
    }
  }

  return SUCCESS;
}

int send_CAN1_Message(int id, uint8_t* data, int len){
/**
 * @brief Send CAN1 Message with given ID and Data
 * 
 * @param id CAN ID
 * @param data Pointer to Data Array
 * @param len Length of Data Array (max 8)
 * @return 1 at Success, -1 at fail, 0 if CAN1 not active
 */
  if(len > 8){
    Serial.println("Data Length too long");
    return ERROR;
  }

  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = id;
  tx_frame.FIR.B.DLC = len;

  for(int i=0; i<len; i++){
    tx_frame.data.u8[i] = data[i];
  }

  // Send Message
  if(CAN1_active == true){
    if(ESP32Can.CANWriteFrame(&tx_frame) == ESP_OK){
      return SUCCESS;
    }
    else{
      Serial.println("Error sending CAN1 Message");
      return ERROR;
    }
  }
  else{
    Serial.println("CAN1 not active");
    return 0;
  }

}

void define_CAN1_Messages(){
  // Define CAN Messages and Signals for Cummunication

  Avg_Cell_Voltage.length = 9;
  Avg_Cell_Voltage.factor = 0.01;
  Avg_Cell_Voltage.offset = 0;
  Avg_Cell_Voltage.polarity = UNSIGNED;
  Avg_Cell_Voltage.start_bit = 0;

  Max_Cell_Voltage.length = 9;
  Max_Cell_Voltage.factor = 0.01;
  Max_Cell_Voltage.offset = 0;
  Max_Cell_Voltage.polarity = UNSIGNED;
  Max_Cell_Voltage.start_bit = 16;

  Min_Cell_Voltage.length = 9;
  Min_Cell_Voltage.factor = 0.01;
  Min_Cell_Voltage.offset = 0;
  Min_Cell_Voltage.polarity = UNSIGNED;
  Min_Cell_Voltage.start_bit = 32;

  Overall_Voltage.length = 8;
  Overall_Voltage.factor = 0.1;
  Overall_Voltage.offset = 40;
  Overall_Voltage.polarity = UNSIGNED;
  Overall_Voltage.start_bit = 48;

  Battery_Voltage.id = 0x20;
  Battery_Voltage.n_signals = 4;
  Battery_Voltage.signals[0] = &Avg_Cell_Voltage;
  Battery_Voltage.signals[1] = &Max_Cell_Voltage;
  Battery_Voltage.signals[2] = &Min_Cell_Voltage;
  Battery_Voltage.signals[3] = &Overall_Voltage;

  Avg_Temp.length = 9;
  Avg_Temp.factor = 0.1;
  Avg_Temp.offset = 0;
  Avg_Temp.polarity = UNSIGNED;
  Avg_Temp.start_bit = 0;

  Max_Temp.length = 9;
  Max_Temp.factor = 0.1;
  Max_Temp.offset = 0;
  Max_Temp.polarity = UNSIGNED;
  Max_Temp.start_bit = 16;

  Min_Temp.length = 9;
  Min_Temp.factor = 0.1;
  Min_Temp.offset = 0;
  Min_Temp.polarity = UNSIGNED;
  Min_Temp.start_bit = 32;

  Battery_Temperature.id = 0x21;
  Battery_Temperature.n_signals = 3;
  Battery_Temperature.signals[0] = &Avg_Temp;
  Battery_Temperature.signals[1] = &Max_Temp;
  Battery_Temperature.signals[2] = &Min_Temp;

  M_Engine.length = 8;
  M_Engine.factor = 0.1;
  M_Engine.offset = 0;
  M_Engine.polarity = UNSIGNED;
  M_Engine.start_bit = 0;

  n_Engine.length = 10;
  n_Engine.factor = 1;
  n_Engine.offset = 0;
  n_Engine.polarity = UNSIGNED;
  n_Engine.start_bit = 8;

  Speed.length = 11;
  Speed.factor = 0.1;
  Speed.offset = 0;
  Speed.polarity = UNSIGNED;
  Speed.start_bit = 24;

  SOC.length = 10;
  SOC.factor = 0.1;
  SOC.offset = 0;
  SOC.polarity = UNSIGNED;
  SOC.start_bit = 40;

  Power_Data.id = 0x010;
  Power_Data.n_signals = 4;
  Power_Data.signals[0] = &M_Engine;
  Power_Data.signals[1] = &n_Engine;
  Power_Data.signals[2] = &Speed;
  Power_Data.signals[3] = &SOC;

  LED_Flashing.length = 1;
  LED_Flashing.factor = 1;
  LED_Flashing.offset = 0;
  LED_Flashing.polarity = UNSIGNED;
  LED_Flashing.start_bit = 0;

  LED_Signal.length = 3;
  LED_Signal.factor = 1;
  LED_Signal.offset = 0;
  LED_Signal.polarity = UNSIGNED;
  LED_Signal.start_bit = 1;

  Option1_Commands.id = 0x031;
  Option1_Commands.n_signals = 2;
  Option1_Commands.signals[0] = &LED_Flashing;
  Option1_Commands.signals[1] = &LED_Signal;

  RemoteDrive_Request.length = 1;
  RemoteDrive_Request.factor = 1;
  RemoteDrive_Request.offset = 0;
  RemoteDrive_Request.polarity = UNSIGNED;
  RemoteDrive_Request.start_bit = 0;

  SOC_Request.length = 1;
  SOC_Request.factor = 1;
  SOC_Request.offset = 0;
  SOC_Request.polarity = UNSIGNED;
  SOC_Request.start_bit = 1;

  VCU_Commands.id = 0x030;
  VCU_Commands.n_signals = 2;
  VCU_Commands.signals[0] = &RemoteDrive_Request;
  VCU_Commands.signals[1] = &SOC_Request;
}

void define_CAN2_Messages(){
  // Define CAN2 Messages and Signals for Cummunication
  Customer_ID.length = 16;
  Customer_ID.factor = 1;
  Customer_ID.offset = 0;
  Customer_ID.polarity = UNSIGNED;
  Customer_ID.start_bit = 0;

  Power_Mode.length = 8;
  Power_Mode.factor = 1;
  Power_Mode.offset = 0;
  Power_Mode.polarity = UNSIGNED;
  Power_Mode.start_bit = 16;

  RFID_Data.id = 0x010;
  RFID_Data.n_signals = 2;
  RFID_Data.signals[0] = &Customer_ID;
  RFID_Data.signals[1] = &Power_Mode;
}

void process_CAN_PowerData(CAN_frame_t* frame){
  
  // Speed
  float value = decodeSignal(Speed, frame);
  CAN_speed = value;  // Save Value to global Variable to be shown on Display and Webserver

  value = decodeSignal(SOC, frame);
  CAN_soc = value;    // Save Value to global Variable to be shown on Display and Webserver
}

void process_CAN_Option1Commands(CAN_frame_t* frame){
  float value = decodeSignal(LED_Flashing, frame);
  // Enable or disable LED Flash Timer
  if(value = TRUE){
    LED_Flashing_Active = TRUE;
  }
  else{
    LED_Flashing_Active = FALSE;
  }

  value = decodeSignal(LED_Signal, frame);
  // Activate LED Signal as often as defined in the CAN Message
  if(value > 0){
    LED_Flashing_CTR = value; // every cycle of the timer toggles the LED, so double the count
    timerAlarmEnable(TIM_LED_Flashing); // start Timer
  }

}

void process_CAN_BatteryTemperature(CAN_frame_t* frame){
  float value = decodeSignal(Avg_Temp, frame);

  // Save Value to global Variable to be shown on Display and Webserver
  CAN_battery_temp = value;
}

void process_CAN_BatteryVoltage(CAN_frame_t* frame){
  // Process incoming Battery Voltage CAN Message

  float value = decodeSignal(Overall_Voltage, frame);
  // Save Value to global Variable to be shown on Display and Webserver
  Overall_Voltage_value = value;
  
}

int decodeSignal(CAN_Signal signal, CAN_frame_t* frame){
  int data = 0;

  // Combine all data arrays to one value
  for(int i = 0; i < frame->FIR.B.DLC; i++){
    data |= frame->data.u8[i] << (i * 8);
  }

  // Read Speed from Data
  // generate mask
  int mask = 0;
  for(int i=0; i<signal.length; i++){
    mask = (mask << 1) | 0x01;
  }

  // Process all signals
  int result = data >> signal.start_bit & mask;

  // Apply factor and offset
  float value = result * signal.factor + signal.offset;

  return value;
}

void IRAM_ATTR TIM_LED_Flashing_overflow(){
  static bool led_state = OFF;
  if(LED_Flashing_CTR > 0){
    // Flash LED
    if(led_state == OFF){
      // Set LED on
      Status_LED_ON();
      led_state = ON;
      LED_Flashing_CTR -= 1;
    }
    else{
      // Set LED off
      Status_LED_OFF();
      led_state = OFF;
    }
  }
  else{
    // Stop Flashing
    Status_LED_OFF();
    timerAlarmDisable(TIM_LED_Flashing);       // stop Timer
  }
  
  // Constant Flashing
  if(LED_Flashing_Active == true){
    // Flash LED permanently
    if(led_state == OFF){
      // Set LED on
      Status_LED_ON();
      led_state = ON;
    }
    else{
      // Set LED off
      Status_LED_OFF();
      led_state = OFF;
    }
  }
  
}

void write_SPI_Register(uint8_t reg, uint8_t value){
  // Write single Register via SPI
  SPI_select(CAN2);
  my_SPI.transfer(0x03);      // Send Write Command
  my_SPI.transfer(reg);       // Send Register Address
  my_SPI.transfer(value);     // Send Value
  SPI_deselect();
  delay(10);
}

uint8_t read_SPI_Register(int chip, uint8_t reg){
  // Read single Register via SPI
  SPI_select(chip);
  my_SPI.transfer(0x02);      // Send Read Command
  my_SPI.transfer(reg);       // Send Register Address
  uint8_t value = my_SPI.transfer(0x00); // Send Dummy Byte to receive Value
  SPI_deselect();
  delay(10);
  return value;
}

void print_NVS(){
  static int Display_off_old = true, RF_CAN_en_old = true, RF_enable_old= true, Display_enable_old= true, WiFi_enable_old= true, RFID_enable_old= true, LED_enable_old= true;

  Serial.println("NVS Storage Content:");
  
  Serial.print("Display Off: \t");
  Serial.println(ESP_storage.getInt("Display_off"));
  Serial.print("RF Enable: \t");
  Serial.println(ESP_storage.getInt("RF_enable"));
  Serial.print("Display Enable: ");
  Serial.println(ESP_storage.getInt("Display_enable"));
  Serial.print("WiFi Enable: \t");
  Serial.println(ESP_storage.getInt("WiFi_enable"));
  Serial.print("RFID Enable: \t");
  Serial.println(ESP_storage.getInt("RFID_enable"));
  Serial.print("LED Enable: \t");
  Serial.println(ESP_storage.getInt("LED_enable"));
  Serial.print("RF via CAN: \t");
  Serial.println(ESP_storage.getInt("RF_CAN_en"));
}

void handleSettingsToggle() {

  String SETTINGS_page = "";

  if(Master_Mode_active == true){
    // AJAX-POST-Handler: nur speichern, keine HTML-Seite
    if (kart_server.method() == HTTP_POST) {
      if (kart_server.hasArg("key") && kart_server.hasArg("state")) {
        String key = kart_server.arg("key");
        String state = kart_server.arg("state");
        bool value = (state == "true");

        if (key == "Display_enable") {
          ESP_storage.putInt("Display_enable", value);
          Serial.printf("Display_enable -> %s\n", value ? "true" : "false");
        }
        if (key == "RF_enable") {
          ESP_storage.putInt("RF_enable", value);
          Serial.printf("RF_enable -> %s\n", value ? "true" : "false");
        }
        if (key == "WiFi_enable") {
          ESP_storage.putInt("WiFi_enable", value);
          Serial.printf("WiFi_enable -> %s\n", value ? "true" : "false");
        }
        if (key == "RFID_enable") {
          ESP_storage.putInt("RFID_enable", value);
          Serial.printf("RFID_enable -> %s\n", value ? "true" : "false");
        }
        if (key == "LED_enable") {
          ESP_storage.putInt("LED_enable", value);
          Serial.printf("LED_enable -> %s\n", value ? "true" : "false");
        }
        if (key == "RF_CAN_en") {
          ESP_storage.putInt("RF_CAN_en", value);
          Serial.printf("RF_CAN_en -> %s\n", value ? "true" : "false");
        }

        kart_server.send(200, "text/plain", "OK");
        return;
      }
    }

    // Aktuelle Werte aus NVS
    bool displayEnable = ESP_storage.getInt("Display_enable", true);
    bool rfEnable = ESP_storage.getInt("RF_enable", true);
    bool wifiEnable = ESP_storage.getInt("WiFi_enable", true);
    bool rfidEnable = ESP_storage.getInt("RFID_enable", true);
    bool ledEnable = ESP_storage.getInt("LED_enable", true);
    bool rfCanEnable = ESP_storage.getInt("RF_CAN_en", true);

    // HTML-Seite
    SETTINGS_page =
      "<!DOCTYPE html><html lang='de'><head>"
      "<meta charset='UTF-8'>"
      "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
      "<title>Settings</title>"
      "<style>"
      "body{margin:0;display:flex;justify-content:center;align-items:flex-start;min-height:100vh;"
      "background:#f2f2f2;font-family:Arial,sans-serif;}"
      ".card{margin-top:40px;background:#fff;padding:20px 30px;border-radius:12px;text-align:center;"
      "box-shadow:0 4px 12px rgba(0,0,0,0.15);max-width:350px;width:100%;}"
      "h1 { color:red; margin-bottom:20px; }"
      "a.btn { display:inline-block; margin:8px; padding:12px 18px; border-radius:8px; text-decoration:none; color:white; font-weight:bold; }"
      "a.gray { background:gray; }"
      ".switch{position:relative;display:inline-block;width:60px;height:34px;margin:10px;}"
      ".switch input{opacity:0;width:0;height:0;}"
      ".slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;"
      "background-color:#ccc;transition:.4s;border-radius:34px;}"
      ".slider:before{position:absolute;content:'';height:26px;width:26px;left:4px;"
      "bottom:4px;background:white;transition:.4s;border-radius:50%;}"
      "input:checked+.slider{background-color:green;}"
      "input:checked+.slider:before{transform:translateX(26px);}"
      ".label{font-weight:bold;display:block;margin-top:10px;}"
      "</style></head><body>"
      "<div class='card'>"
      "<h1>Hersteller Einstellungen</h1>"

      // Display_enable Switch
      "<div class='label'>Display</div>"
      "<label class='switch'>"
      "<input type='checkbox' id='displayToggle' " + String(displayEnable ? "checked" : "") + ">"
      "<span class='slider'></span>"
      "</label>"

      // RF_enable Switch
      "<div class='label'>Funkempfänger</div>"
      "<label class='switch'>"
      "<input type='checkbox' id='rfToggle' " + String(rfEnable ? "checked" : "") + ">"
      "<span class='slider'></span>"
      "</label>"

      // WiFi_enable Switch
      "<div class='label'>WiFi</div>"
      "<label class='switch'>"
      "<input type='checkbox' id='wifiToggle' " + String(wifiEnable ? "checked" : "") + ">"
      "<span class='slider'></span>"
      "</label>"

      // RFID_enable Switch
      "<div class='label'>RFID</div>"
      "<label class='switch'>"
      "<input type='checkbox' id='rfidToggle' " + String(rfidEnable ? "checked" : "") + ">"
      "<span class='slider'></span>"
      "</label>"

      // LED_enable Switch
      "<div class='label'>LED</div>"
      "<label class='switch'>"
      "<input type='checkbox' id='ledToggle' " + String(ledEnable ? "checked" : "") + ">"
      "<span class='slider'></span>"
      "</label>"

      // RF_CAN_enable Switch
      "<div class='label'>Funksignale via CAN</div>"
      "<label class='switch'>"
      "<input type='checkbox' id='rfCanToggle' " + String(rfCanEnable ? "checked" : "") + ">"
      "<span class='slider'></span>"
      "</label>"

      "<br><a href=\"/\" class=\"btn gray\">Zurück</a>"

      "</div>"
      "<script>"
      "function sendToggle(key, state){"
      "  fetch('/hersteller',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},"
      "  body:'key='+key+'&state='+(state?'true':'false')});"
      "}"
      "document.getElementById('displayToggle').addEventListener('change',function(){"
      "  sendToggle('Display_enable',this.checked);"
      "});"
      "document.getElementById('rfToggle').addEventListener('change',function(){"
      "  sendToggle('RF_enable',this.checked);"
      "});"
      "document.getElementById('wifiToggle').addEventListener('change',function(){"
      "  sendToggle('WiFi_enable',this.checked);"
      "});"
      "document.getElementById('rfidToggle').addEventListener('change',function(){"
      "  sendToggle('RFID_enable',this.checked);"
      "});"
      "document.getElementById('ledToggle').addEventListener('change',function(){"
      "  sendToggle('LED_enable',this.checked);"
      "});"
      "document.getElementById('rfCanToggle').addEventListener('change',function(){"
      "  sendToggle('RF_CAN_en',this.checked);"
      "});"
      "</script>"
      "</body></html>";
  }
  else{
    SETTINGS_page =
      "<!DOCTYPE html><html lang=\"de\"><head>"
      "<meta charset=\"UTF-8\">"
      "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
      "<title>SMS REVO SL - Hersteller</title>"
      "<style>"
      "body { margin:0; display:flex; justify-content:center; align-items:center; height:100vh; background:#f2f2f2; font-family:Arial,sans-serif; }"
      ".card { background:#fff; padding:20px 30px; border-radius:12px; text-align:center; box-shadow:0 4px 12px rgba(0,0,0,0.15); max-width:400px; width:100%; }"
      "h1 { color:red; margin-bottom:20px; }"
      "a.btn { display:inline-block; margin:8px; padding:12px 18px; border-radius:8px; text-decoration:none; color:white; font-weight:bold; }"
      "a.gray { background:gray; }"
      "</style></head><body><div class=\"card\">"
      "<h1>Hier gibt es nix zu sehen</h1>"


      "<br><a href=\"/\" class=\"btn gray\">Zurück</a>"
      "</div></body></html>";
  }
  kart_server.send(200, "text/html", SETTINGS_page);
}

void handleDownloadLog() {
  if (SPIFFS.exists("/Error.log")) {
    File file = SPIFFS.open("/Error.log", "r");
    if (file) {
      // Header setzen, damit der Browser einen Download startet
      kart_server.sendHeader("Content-Type", "text/plain");
      kart_server.sendHeader("Content-Disposition", "attachment; filename=Error.log");
      kart_server.sendHeader("Connection", "close");
      kart_server.streamFile(file, "text/plain");
      file.close();
      return;
    }
  }
  else{
    String LOGDATEN_page =
      "<!DOCTYPE html><html lang=\"de\"><head>"
      "<meta charset=\"UTF-8\">"
      "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
      "<title>SMS REVO SL - Hersteller</title>"
      "<style>"
      "body { margin:0; display:flex; justify-content:center; align-items:center; height:100vh; background:#f2f2f2; font-family:Arial,sans-serif; }"
      ".card { background:#fff; padding:20px 30px; border-radius:12px; text-align:center; box-shadow:0 4px 12px rgba(0,0,0,0.15); max-width:400px; width:100%; }"
      "h1 { color:red; margin-bottom:20px; }"
      "a.btn { display:inline-block; margin:8px; padding:12px 18px; border-radius:8px; text-decoration:none; color:white; font-weight:bold; }"
      "a.gray { background:gray; }"
      "</style></head><body><div class=\"card\">"
      "<h1>Logdatei nicht gefunden</h1>"


      "<br><a href=\"/\" class=\"btn gray\">Zurück</a>"
      "</div></body></html>";
  
    kart_server.send(200, "text/html", LOGDATEN_page);
  }

}

UBYTE DEV_I2C_Read_Byte(UBYTE DevAddr, UBYTE RegAddr) {
  UBYTE value = 0xFF; // Default-Wert bei Fehler

  Serial.print("DEV_I2C_Read_Byte Reg: 0x");
  Serial.println(RegAddr, HEX);

  // Schritt 1: Register schreiben
  Wire.beginTransmission(DevAddr);
  Wire.write(RegAddr);
  int rv = Wire.endTransmission();
  Serial.print("EndTransmission RV: ");
  Serial.println(rv);
  if (rv != 0) {
    Serial.print("I2C Write Error! Code: ");
    Serial.println(rv);
    return value;
  }

  // Schritt 2: Byte vom Gerät anfordern
  int n = Wire.requestFrom(DevAddr, (byte)1);
  if (n == 1 && Wire.available()) {
    value = Wire.read();
    Serial.print("Value: ");
    Serial.println(value, HEX);
  } else {
    Serial.println("I2C Read Error!");
  }

  return value;
}