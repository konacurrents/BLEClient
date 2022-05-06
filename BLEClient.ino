/*
Name:		NimBLE_PetTutor_Client.ino
Created : 8 / 31 / 2021 2 : 40 : 01 PM
Author : wes
  * added M5StickCPlus button and LCD
* /


/** NimBLE_Server Demo:
 *
 *  Demonstrates many of the available features of the NimBLE client library.
 *
 *  Created: on March 24 2020
 *      Author: H2zero
 *
*/

#include <NimBLEDevice.h>
#include <M5StickCPlus.h>
//#include "Speaker.h"

#define PIR_PIN 36 //passive IR Hat

int FeedCount = 0;

double vbat = 0.0; //for battery voltage

/* init M5StickCPlus IMU variables */
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;
float diffX   = 0.0F;
float diffY   = 0.0F;
float diffZ   = 0.0F;
float diffXYZ = 0.0F;
float prevX = 0.0F;
float prevY = 0.0F;
float prevZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

float TILT_SENSITIVITY = 0.5;

unsigned long PrevSampleTime    = 0; 
unsigned long PrevTriggerTime    = 0;
unsigned long InactivityTimeOut = 0;

#define Elapsed3secs  3000
#define Elapsed3mins  180000 // 3 minutes in milliseconds


//#define originalApproach;  //different approaches to register service and characteristic UUID
/*** define which ESP32 board is target for firmware***/
//#define ESP32DEVKIT
#define M5STICKCPLUS

#define nimBLEApproach  //  uses  NimBLEUUID


#ifdef nimBLEApproach
static NimBLEUUID serviceUUID("B0E6A4BF-CCCC-FFFF-330C-0000000000F0"); //??
static NimBLEUUID    charUUID("b0e6a4bf-cccc-ffff-330c-0000000000f1");
static NimBLERemoteCharacteristic* pChrFeed;
//static NimBLERemoteCharacteristic* pChrFeed1,pChrFeed2,pChrFeed3;
#endif

void scanEndedCB(NimBLEScanResults results);

static NimBLEAdvertisedDevice* advDevice;

static bool doConnect = false;
static uint32_t scanTime = 0; /** 0 = scan forever */

bool FeedFlag = false;
bool ResetFeedCount = false;
bool ButtonPressedB = false;
bool TiltMode = false;
bool InactivityMode = false;
 
#ifdef ESP32DEVKIT
  #define LED 2  
#endif
#ifdef M5STICKCPLUS
  #define LED 10  
#endif

const uint32_t SLEEP_DURATION = 1 * 1000000; // �s

void Led_ON(){
#ifdef ESP32DEVKIT
  digitalWrite(LED, LOW);
#endif
#ifdef M5STICKCPLUS
  digitalWrite(LED, LOW);
#endif
}

void Led_OFF(){
#ifdef ESP32DEVKIT
  digitalWrite(LED, HIGH);
#endif
#ifdef M5STICKCPLUS
  digitalWrite(LED, HIGH);
#endif
}

void M5Message(){
  M5.Lcd.setTextSize(3);  //Set font size.
  M5.Lcd.setRotation(3);  //Rotate the screen.
//  M5.Lcd.setTextColor(BLUE, WHITE);
//  M5.Lcd.println("Pet Tutor0");
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.println("MINI Clicker");
}

void Menu(){
      M5.Lcd.setCursor(210, 110, 1); 
}
void CheckButtonA(){  //big button on front of M5StickC Plus
 // M5.update(); //Read the press state of the key. 
  if (M5.BtnA.wasReleased() && !FeedFlag) { 
    FeedFlag = true;   // FEED=true
                Serial.println("Button pressed******"); //?????
  }
  else if (M5.BtnA.wasReleasefor(2000) && !ResetFeedCount) { 
    ResetFeedCount = true;
  }
}
void CheckButtonB(){  //small button on right side of M5StickC Plus
  if (M5.BtnB.wasReleased() ){ //&& !TiltMode) { 
    TiltMode = !TiltMode;  //toggle TILT
    M5.Lcd.setCursor(210, 110, 1); 
    if (TiltMode) {
 //     M5.Lcd.setTextColor(BLUE, YELLOW);           
      M5.Lcd.printf("T\r\n");          //display tile active
 //     M5.Lcd.setTextColor(WHITE, BLACK);
//    InactivityMode = true;  //by default timeout is turned on when entering TILT
    }
    else {
      M5.Lcd.printf(" \r\n");//blank over "T"   display tilt inactive
    }
  }
  else if (M5.BtnB.wasReleasefor(2000) ) {  //toggle timer enable or disable
    InactivityMode = !InactivityMode;//toggle enable/disable
    M5.Lcd.setCursor(210, 90, 1);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.printf("%d\r\n", InactivityMode);//
    } 
//  else if (M5.BtnB.wasReleased() && TiltMode) { 
//    M5.Lcd.setCursor(210, 110, 1);
//    M5.Lcd.setTextColor(WHITE, BLACK);
//    M5.Lcd.printf(" \r\n");//blank over "T"
//    TiltMode = false;  // toggle TILT off 
//  }
}

bool CheckMotion()
{
    if ( (millis() - PrevSampleTime) >100) // 100ms sample interval
    {
      PrevSampleTime = millis(); //update for next sample point
      M5.IMU.getAccelData(&accX,&accY,&accZ);
      diffX =   abs(prevX - accX);
      diffY =   abs(prevY - accY);
      diffZ =   abs(prevZ - accZ);
      diffXYZ = diffX + diffY + diffZ;

    
      prevX = accX; //save x,y,z from this cycle for next cycle
      prevY = accY;
      prevZ = accZ;
    }   
    if ( (diffXYZ > TILT_SENSITIVITY) && ((millis()-PrevTriggerTime)>Elapsed3secs) )  //if the movement is above threshold sensitivity then broadcast and start IGNORE time
      {
        PrevTriggerTime = millis(); 
        Serial.printf("diff:  %.2f  \r\n", diffXYZ);
        return true;// TILT has exceed threshold
      }
    else
    {
        return false; //movement does not exceed threshold
    }
}  // end of CheckMotion

#if 0 //remove code for ESP32 Dev Kit v1 button
/***************** boot button **************/
/*   ESP32 position    top view USB plug facing down:   left=EN-----USB plug------right=BOOT button*/
#define ESP_INTR_FLAG_DEFAULT 0
// #define GPIO_NUM_0 0


// GPIO_NUM_0(0) // WJL
SemaphoreHandle_t semaphore = nullptr;
/* boot button */
/* boot button functions */
void IRAM_ATTR handler(void* arg) {
    xSemaphoreGiveFromISR(semaphore, NULL);
}

/*** action taken on boot button press ***/
void button_task(void* arg) {
    for (;;) {
        if (xSemaphoreTake(semaphore, portMAX_DELAY) == pdTRUE) {
            Serial.println("Oh, button pushed!\n");
            ButtonPressed = true;
        }
    }
}
#endif

bool ServerConnectedFlag;

/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) {
        Serial.println("Connected");
        /** After connection we should change the parameters if we don't need fast response times.
         *  These settings are 150ms interval, 0 latency, 450ms timout.
         *  Timeout should be a multiple of the interval, minimum is 100ms.
         *  I find a multiple of 3-5 * the interval works best for quick response/reconnect.
         *  Min interval: 120 * 1.25ms = 150, Max interval: 120 * 1.25ms = 150, 0 latency, 60 * 10ms = 600ms timeout
         */
        pClient->updateConnParams(120, 120, 0, 60);
//new        pClient->updateConnParams(120, 120, 10, 60); //change latency to 10 from 0
        ServerConnectedFlag = true;
//        M5.Lcd.setCursor(210, 50, 1);
//        M5.Lcd.setTextColor(BLUE, YELLOW);
//        M5.Lcd.printf("C\r\n");// C = connected BLE
//        M5.Lcd.setTextColor(WHITE, BLACK);
    };

    void onDisconnect(NimBLEClient* pClient) {
        Serial.print(pClient->getPeerAddress().toString().c_str());
        Serial.println(" Disconnected - Starting scan");
        NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
        ServerConnectedFlag = false;
//        M5.Lcd.setCursor(210, 50, 1);
//        M5.Lcd.setTextColor(WHITE, BLACK);
//        M5.Lcd.printf("nc\r\n");// C = connected BLE
//        M5.Lcd.setTextColor(WHITE, BLACK);
    };

    /** Called when the peripheral requests a change to the connection parameters.
     *  Return true to accept and apply them or false to reject and keep
     *  the currently used parameters. Default will return true.
     */
    bool onConnParamsUpdateRequest(NimBLEClient* pClient, const ble_gap_upd_params* params) {
        if (params->itvl_min < 24) { /** 1.25ms units */
            return false;
        }
        else if (params->itvl_max > 40) { /** 1.25ms units */
            return false;
        }
        else if (params->latency > 2) { /** Number of intervals allowed to skip */
            return false;
        }
        else if (params->supervision_timeout > 100) { /** 10ms units */
            return false;
        }

        return true;
    };

    /********************* Security handled here **********************
    ****** Note: these are the same return values as defaults ********/
    uint32_t onPassKeyRequest() {
        Serial.println("Client Passkey Request");
        /** return the passkey to send to the server */
        return 123456;
    };

    bool onConfirmPIN(uint32_t pass_key) {
        Serial.print("The passkey YES/NO number: ");
        Serial.println(pass_key);
        /** Return false if passkeys don't match. */
        return true;
    };

    /** Pairing process complete, we can check the results in ble_gap_conn_desc */
    void onAuthenticationComplete(ble_gap_conn_desc* desc) {
        if (!desc->sec_state.encrypted) {
            Serial.println("Encrypt connection failed - disconnecting");
            /** Find the client with the connection handle provided in desc */
            NimBLEDevice::getClientByID(desc->conn_handle)->disconnect();
            return;
        }
    };
};


/** Define a class to handle the callbacks when advertisments are received */
class AdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {

    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        //Serial.print("Advertised Device found: ");
        //Serial.println(advertisedDevice->toString().c_str());
        String deviceInfo = advertisedDevice->toString().c_str();
        if (advertisedDevice->isAdvertisingService(NimBLEUUID(serviceUUID))  || (deviceInfo.indexOf("b0e6a4bf-cccc-ffff-330c-0000000000f0") > 0)) //this was the original code service called "DEAD"
       {
            Serial.println("Found Our Service");
            Serial.println(advertisedDevice->toString().c_str());

            /** stop scan before connecting */
            NimBLEDevice::getScan()->stop();
            /** Save the device reference in a global for the client to use*/
            advDevice = advertisedDevice;
            /** Ready to connect now */
            doConnect = true;
        }
    };
};


/** Notification / Indication receiving handler callback */
void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    std::string str = (isNotify == true) ? "Notification" : "Indication";
    str += " from ";
    /** NimBLEAddress and NimBLEUUID have std::string operators */
    str += std::string(pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress());
    str += ": Service = " + std::string(pRemoteCharacteristic->getRemoteService()->getUUID());
    str += ", Characteristic = " + std::string(pRemoteCharacteristic->getUUID());
    str += ", Value = " + std::string((char*)pData, length);
    Serial.println(str.c_str());
}

/** Callback to process the results of the last scan or restart it */
void scanEndedCB(NimBLEScanResults results) {
    Serial.println("Scan Ended");
}


/** Create a single global instance of the callback class to be used by all clients */
static ClientCallbacks clientCB;


/** Handles the provisioning of clients and connects / interfaces with the server */
bool connectToServer() {
    NimBLEClient* pClient = nullptr; //creates a new instance of a pointer to a client(1 for each server it connects to)
    /** Check if we have a client we should reuse first **/
    if (NimBLEDevice::getClientListSize()) {  //this runs only if #clients>= 1...ie no client instance start yet
        /** Special case when we already know this device, we send false as the
         *  second argument in connect() to prevent refreshing the service database.
         *  This saves considerable time and power.
         */
        pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress()); //if the server was connected to before there should be an address stored for it -wha
        if (pClient) {
            if (!pClient->connect(advDevice, false)) {
                Serial.println("Reconnect failed");
                return false;
            }
            Serial.println("Reconnected client");
        }
        /** We don't already have a client that knows this device,
         *  we will check for a client that is disconnected that we can use.
         */
        else {
            pClient = NimBLEDevice::getDisconnectedClient(); //reusing a client that was created but is disconnected  -wha
        }
    }

    /** No client to reuse? Create a new one. */
    if (!pClient) { //if a client instance is created from above this will not be executed -wha
        if (NimBLEDevice::getClientListSize() >= 1 ) {   //wha -original example code used the max configuration of 3 --> NIMBLE_MAX_CONNECTIONS) {
            Serial.println("Max clients reached - no more connections available");
            return false;
        }
         
        pClient = NimBLEDevice::createClient();

        Serial.println("New client created");

        pClient->setClientCallbacks(&clientCB, false);
        /** Set initial connection parameters: These settings are 15ms interval, 0 latency, 120ms timout.
         *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
         *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
         *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 51 * 10ms = 510ms timeout
         */
        pClient->setConnectionParams(12, 12, 0, 51);
//        pClient->setConnectionParams(12, 12, 5, 51);
        /** Set how long we are willing to wait for the connection to complete (seconds), default is 30. */
        pClient->setConnectTimeout(5);


        if (!pClient->connect(advDevice)) {
            /** Created a client but failed to connect, don't need to keep it as it has no data */
            NimBLEDevice::deleteClient(pClient);
            Serial.println("Failed to connect, deleted client");
            return false;
        }
    }

    if (!pClient->isConnected()) {
        if (!pClient->connect(advDevice)) {
            Serial.println("Failed to connect");
            return false;
        }
    }
    Serial.printf("number clients=   %d \n\r", NimBLEDevice::getClientListSize()); //
    Serial.print("Connected to: ");
    Serial.println(pClient->getPeerAddress().toString().c_str());
    Serial.print("RSSI: ");
    Serial.println(pClient->getRssi());

    /** Now we can read/write/subscribe the charateristics of the services we are interested in */
    NimBLERemoteService* pSvc = nullptr;
    NimBLERemoteCharacteristic* pChr = nullptr;
    NimBLERemoteDescriptor* pDsc = nullptr;
   // the original was setup for service/characteristic : DEAD-BEEF   ..but we are revising for BAAD/F00D
    pSvc = pClient->getService(serviceUUID);  // char* serviceUUID   36 characters
    if (pSvc) {     /** make sure it's not null */
        pChr = pSvc->getCharacteristic(charUUID);  //this was the original example code characteristic called  "BEEF"
        pChrFeed = pChr;
        if (pChr) {     /** make sure it's not null */
            if (pChr->canRead()) {
                Serial.print(pChr->getUUID().toString().c_str());
                Serial.print(" Value: ");
                Serial.println(pChr->readValue().c_str());
            }

            if (pChr->canWrite()) {
                if (pChr->writeValue("_ESP_32")) { //was Tasty
                    Serial.print("Wrote new value to: ");
                    Serial.println(pChr->getUUID().toString().c_str());
                }
                else {
                    /** Disconnect if write failed */
                    pClient->disconnect();
                    return false;
                }

                if (pChr->canRead()) {
                    Serial.print("The value of: ");
                    Serial.print(pChr->getUUID().toString().c_str());
                    Serial.print(" is now: ");
                    Serial.println(pChr->readValue().c_str());

                    if (String("_ESP_32").compareTo(pChr->readValue().c_str()) == 0)
                    {
                      Serial.println("*** This is an ESP Based Feeder (not GEN3)***\n");
                    }
                    else
                    {
                      Serial.println("*** This is an GEN3 Feeder (not ESP32)***");

                    }
                }
            }

            /** registerForNotify() has been deprecated and replaced with subscribe() / unsubscribe().
             *  Subscribe parameter defaults are: notifications=true, notifyCallback=nullptr, response=false.
             *  Unsubscribe parameter defaults are: response=false.
             */
            if (pChr->canNotify()) {
                //if(!pChr->registerForNotify(notifyCB)) {
                if (!pChr->subscribe(true, notifyCB)) {
                    /** Disconnect if subscribe failed */
                    pClient->disconnect();
                    return false;
                }
            }
            else if (pChr->canIndicate()) {
                /** Send false as first argument to subscribe to indications instead of notifications */
                //if(!pChr->registerForNotify(notifyCB, false)) {
                if (!pChr->subscribe(false, notifyCB)) {
                    /** Disconnect if subscribe failed */
                    pClient->disconnect();
                    return false;
                }
            }
        }

    }
    else {
        Serial.println("DEAD service not found.");
    }
    // original example  pSvc = pClient->getService("BAAD"); // reference only---remove after testing
    if (pSvc) {     /** make sure it's not null */
        // origional example:  pChr = pSvc->getCharacteristic("F00D"); // reference only---remove after testing
        if (pChr) {     /** make sure it's not null */
            if (pChr->canRead()) {
                Serial.print(pChr->getUUID().toString().c_str());
                Serial.print(" Value: ");
                Serial.println(pChr->readValue().c_str());
            }

            pDsc = pChr->getDescriptor(NimBLEUUID("C01D"));
            if (pDsc) {   /** make sure it's not null */
                Serial.print("Descriptor: ");
                Serial.print(pDsc->getUUID().toString().c_str());
                Serial.print(" Value: ");
                Serial.println(pDsc->readValue().c_str());
            }

            if (pChr->canWrite()) {
                if (pChr->writeValue("No tip!")) {
                    Serial.print("Wrote new value to: ");
                    Serial.println(pChr->getUUID().toString().c_str());
                }
                else {
                    /** Disconnect if write failed */
                    pClient->disconnect();
                    return false;
                }

                if (pChr->canRead()) {
                    Serial.print("The value of: ");
                    Serial.print(pChr->getUUID().toString().c_str());
                    Serial.print(" is now: ");
                    Serial.println(pChr->readValue().c_str());
                }
            }

            /** registerForNotify() has been deprecated and replaced with subscribe() / unsubscribe().
             *  Subscribe parameter defaults are: notifications=true, notifyCallback=nullptr, response=false.
             *  Unsubscribe parameter defaults are: response=false.
             */
            if (pChr->canNotify()) {
                //if(!pChr->registerForNotify(notifyCB)) {
                if (!pChr->subscribe(true, notifyCB)) {
                    /** Disconnect if subscribe failed */
                    pClient->disconnect();
                    return false;
                }
            }
            else if (pChr->canIndicate()) {
                /** Send false as first argument to subscribe to indications instead of notifications */
                //if(!pChr->registerForNotify(notifyCB, false)) {
                if (!pChr->subscribe(false, notifyCB)) {
                    /** Disconnect if subscribe failed */
                    pClient->disconnect();
                    return false;
                }
            }
        }

    }
    else {
        Serial.println("BAAD service not found.");
    }

    Serial.println("Done with this device!");
    return true;
}

void setup() {
    Serial.begin(115200);
    pinMode(LED, OUTPUT);
    pinMode(PIR_PIN,INPUT_PULLUP);
    ServerConnectedFlag = false;
    Led_OFF();
    M5.begin(); // Initialize M5StickC Plus.
    M5.Imu.Init();  //Init IMU.
    M5.Axp.begin();
    M5Message();
    FeedCount=15;
    TiltMode = false;
    InactivityTimeOut = Elapsed3mins;
    InactivityMode = false;
    M5.Lcd.setCursor(210, 90, 1);
    M5.Lcd.printf("%d\r\n", InactivityMode);//

  
#if 0 // remove code for ESP32 Dev Kit v1 button
    /******* setup for changing boot button to general input *********/
      // Create a binary semaphore
    semaphore = xSemaphoreCreateBinary();
    // Setup the button GPIO pin
    gpio_pad_select_gpio(GPIO_NUM_0);
    // Quite obvious, a button is a input
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
    // Trigger the interrupt when going from HIGH -> LOW ( == pushing button)
    gpio_set_intr_type(GPIO_NUM_0, GPIO_INTR_NEGEDGE);
    // Associate button_task method as a callback
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, NULL);
    // Install default ISR service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // Add our custom button handler to ISR
    gpio_isr_handler_add(GPIO_NUM_0, handler, NULL);
    /* end boot button setup ****/
#endif

    Serial.println("Starting NimBLE Client");
    /** Initialize NimBLE, no device name spcified as we are not advertising */
    NimBLEDevice::init("");

    /** Set the IO capabilities of the device, each option will trigger a different pairing method.
     *  BLE_HS_IO_KEYBOARD_ONLY    - Passkey pairing
     *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
     *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
     */
     //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_KEYBOARD_ONLY); // use passkey
     //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison

     /** 2 different ways to set security - both calls achieve the same result.
      *  no bonding, no man in the middle protection, secure connections.
      *
      *  These are the default values, only shown here for demonstration.
      */
      //NimBLEDevice::setSecurityAuth(false, false, true);
    NimBLEDevice::setSecurityAuth(/*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);

    /** Optional: set the transmit power, default is 3db */
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */

    /** Optional: set any devices you don't want to get advertisments from */
    // NimBLEDevice::addIgnored(NimBLEAddress ("aa:bb:cc:dd:ee:ff"));

    /** create new scan */
    NimBLEScan* pScan = NimBLEDevice::getScan();

    /** create a callback that gets called when advertisers are found */
    pScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());

    /** Set scan interval (how often) and window (how long) in milliseconds */
    pScan->setInterval(45);
    pScan->setWindow(15);

    /** Active scan will gather scan response data from advertisers
     *  but will use more energy from both devices
     */
    pScan->setActiveScan(true);
    /** Start scanning for advertisers for the scan time specified (in seconds) 0 = forever
     *  Optional callback for when scanning stops.
     */
    pScan->start(scanTime, scanEndedCB);
} /****************   end Setup  *************************/


void lightSleep() {
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION);
    esp_light_sleep_start();
}

void loop(){
  M5.update(); //Read the press state of the key. ONLY call once per loop or the status of B button is lost
  //check for inactivity if InactivityMode=true is enabled
//  if ( InactivityMode && ( (millis() - PrevTriggerTime) > InactivityTimeOut) ){              
//    M5.Axp.PowerOff(); // shutoff after no activity
//  }
 
  CheckButtonA(); //check button A on M5StickCPlus
  CheckButtonB(); //check button B on M5StickCPlus

    M5.Lcd.setCursor(0, 110, 1);
    M5.Lcd.printf("bat:%.2fV\r\n", M5.Axp.GetBatVoltage());
    
 //  M5.Lcd.printf("TEMP:%.2fV\r\n", M5.Axp.GetTempInAXP192() );

#ifdef originalApproach
    /** Loop here until we find a device we want to connect to */
    while (!doConnect) {
        delay(1);
    }
    doConnect = false;
    /** Found a device we want to connect to, do it now */
    if (connectToServer()) {
        Serial.println("Success! we should now be getting notifications, scanning for more!");
    }
    else {
        Serial.println("Failed to connect, starting scan");
    }
    NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
#endif

#if 0 // tried to add code to signal led is connected or not....size increments but does not decrement only wha
    if (NimBLEDevice::getClientListSize()>0)//if (NimBLEClient::isConnected() )
    {
                //   Led_OFF();// indicates scanning stopped  wha 
    }
    else{
               //   Led_ON();// 
    }
#endif

    if (doConnect == true) {   //doConnect is a command to try and connect using connectToServer()  wha 8-11-21
        if (connectToServer()) {
            Serial.println("-We are now connected to the BLE Server.");
           // Led_OFF();// indicates scanning stopped  wha 
                M5.Lcd.setCursor(210, 50, 1);
                M5.Lcd.setTextColor(BLUE, YELLOW);
                M5.Lcd.printf("C\r\n");// C = connected BLE
                M5.Lcd.setTextColor(WHITE, BLACK);
        }
        else {
            Serial.println("We have failed to connect to the server; there is nothin more we will do.");
        }
        doConnect = false;
         //  0=stop scanning after first device found
        NimBLEDevice::getScan()->start(scanTime, scanEndedCB); //resume scanning for more BLE servers
    }
//    if ( NimBLEDevice::isConnected() ){
  //  if ( NimBLEClient::isConnected() ){
//    if ( pClient->isConnected() ){     
//    }
    
    //reset feed count
    if (ResetFeedCount == true) {
      FeedCount = 15; //reset after user refills 
      M5.Lcd.setCursor(0, 80); 
      M5.Lcd.printf("Treats= %d \n\r",FeedCount); 
      ResetFeedCount = false; 
    }
    if (TiltMode){
      if(CheckMotion()){
        FeedFlag = true;
      }    
    }
    if (digitalRead(PIR_PIN) && !FeedFlag ) {
      if ((millis()-PrevTriggerTime)>Elapsed3secs){ //only allow triggers > 3 seconds each
        FeedFlag = true;
        PrevTriggerTime = millis();
      }
    }
//    if ( (millis()-PrevTriggerTime) >5000) {  // auto test only remove ?????
//      FeedFlag = true;
//    }
//    if (!ServerConnectedFlag){
//      FeedFlag = false; //do not attempt to feed if not connected to server(feeder)
//    }

    if (FeedFlag == true) { //flag is set in  button_task, PIR , TILT
    // Set the characteristic's value to be the array of bytes that is actually a string.
    std::string newValue = "s"; // this sets a value to the GATT which is the trigger value for the BLE server feeder
    const uint8_t newValueFeed = { 0x00 };
        if (pChrFeed->writeValue(newValueFeed, 1)) {  //force the length to 1.  newValue.length() may return 0 if newValue=null
            Serial.print(newValue.c_str());
            Serial.println("sent FEED");
            PrevTriggerTime = millis(); 
            FeedCount--;  
//FeedCount++;  //??? remove
            if (FeedCount <= 0){
              FeedCount = 0;//avoid display of negative values
            }
            M5.Lcd.setCursor(0, 80);
            M5.Lcd.printf("Treats= %d \n\r",FeedCount);
        }
        else {
            Serial.print(newValue.c_str());
            Serial.println("FAILED GATT write");
        }
        FeedFlag = false;
      //  delay(100);
        /* check for the acknowledge from the server which is 0x01   */
        if (pChrFeed->canRead()) {
            std::string value = pChrFeed->readValue();
            Serial.print("-server response: ");
            Serial.println(value.c_str());
            if (value[0] == 0x01) {  // server ack is 0x01....so flash client led for the ack from the server  wha 9-28-21
              if (!InactivityMode){ //false=battery saver mode
                Led_ON();
                delay(100);
                Led_OFF();//  
                delay(100);                
              }
            }
        }
    }
   // lightSleep();  // not ready to test yet
   //FeedFlag = true; //????????? temporay test
    delay(10);
}
