#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>

#define FIRMWARE_VERSION "V3.0"

#define PI_VALUE 3.1415926535897932384626433832795

#define RED    0xFF0000
#define GREEN  0x00FF00
#define BLUE   0x0000FF
#define YELLOW 0xFFFF00
#define PINK   0xFF1088
#define ORANGE 0xE05800
#define WHITE  0xFFFFFF

#define RSSI_HIGH 61
#define RSSI_LOW 62
#define RSSI_AVERAGE 60



// CONSTANTS
const long LORA_FREQUENCY =  433E6;                   // LORA FREQUENCY set to 433MHz
const int SYNC_ID = 0xF3;                             // SYNC-ID for Lora receiver
const int NEOPIXEL = 14;   

const int LED_COUNT = 5;                              // Number of LEDs in PCB ( pcb has 5)
const int LED_BRIGHTNESS = 30;                        // variable for LED Brightness

const long SAMPLING_RATE_TELEMETRY = 600000;          // time between heartbeat pulses

const unsigned eeprom_limit = 7;

// PINS
const int SIREN_CONTROL = 23;                          // Pin for SIREN MOSFET
const int RELAY_CONTROL = 22;                          // Pin for Relay MOSFET

const int LORA_SLAVE_SELECT = 10;                     // Pin for slave Select for LORA module
const int LORA_RESET = 9;                             // Pin for LORA_LORA_LORA_LORA_LORA_RESET PIN for LORA module
const int LORA_INTERRUPT = 8;                         // Pin for Interupt PIN for LORA module

// VARIABLES
bool tension_alarm = true;
bool length_alarm = true;
int diameter = 0;
int low_threshold_length = 0;
int high_threshold_length = 0;
int low_threshold_tension = 0;
int high_threshold_tension = 0;
unsigned long duration = 0;
bool init_flag = false;
bool Searching_for_nodes = false;
String node_names[6];
int node_rssi[6];
int last_node_index = 0;
String paired_node = "";

void writeString(char add,String data);
String read_String(char add);

DMAMEM int displayMemory[LED_COUNT*1];
int drawingMemory[LED_COUNT*1];


Adafruit_NeoPixel strip(LED_COUNT, NEOPIXEL, NEO_GRB + NEO_KHZ800);

union {
  char charByte[4];
  long valLong;
} value;


String message_from_Display = "";
String incomming_data = "";

bool waiting_for_reset_response = false;

boolean checkTelemetry()
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= SAMPLING_RATE_TELEMETRY)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

void serialEvent5() {
    while (Serial5.available()) {
        char inChar = (char)Serial5.read();
        message_from_Display += inChar;     
    }
}

void writeValue(String variableName, String stringData) {

    String ToBeSent_value = ""; //Declare and initialise the string we will send
    ToBeSent_value = variableName; //Build the part of the string that we know 
    ToBeSent_value.concat(".val=");
    ToBeSent_value.concat(stringData);
    
    for (unsigned int i = 0; i < ToBeSent_value.length(); i++)
    {
        Serial5.write(ToBeSent_value[i]);
    }
    Serial5.write(0xff);
    Serial5.write(0xff);
    Serial5.write(0xff);
}

void writeString(String variableName, String stringData) {

    String ToBeSent_string = "";
    ToBeSent_string = variableName;
    ToBeSent_string.concat(".txt=\"");
    ToBeSent_string.concat(stringData);
    ToBeSent_string.concat("\"");
    
    for (unsigned int i = 0; i < ToBeSent_string.length(); i++)
    {
        Serial5.write(ToBeSent_string[i]);
    }

    Serial5.write(0xff);
    Serial5.write(0xff);
    Serial5.write(0xff);
}

void makeVissible(String variableName) {

    String ToBeSent_string = "vis ";
    ToBeSent_string.concat(variableName);
    ToBeSent_string.concat(",1");
    Serial.print(ToBeSent_string);
    
    for (unsigned int i = 0; i < ToBeSent_string.length(); i++)
    {
        Serial5.write(ToBeSent_string[i]);
    }

    Serial5.write(0xff);
    Serial5.write(0xff);
    Serial5.write(0xff);
}

void changeImage(String variableName, int newImageId) {

    String ToBeSent_string = "";
    ToBeSent_string = variableName;
    ToBeSent_string.concat(".pic=");
    ToBeSent_string.concat(String(newImageId));
    
    for (unsigned int i = 0; i < ToBeSent_string.length(); i++)
    {
        Serial5.write(ToBeSent_string[i]);
    }

    Serial5.write(0xff);
    Serial5.write(0xff);
    Serial5.write(0xff);
}

void changePage(String pageName) {

    String ToBeSent_string = "page ";
    ToBeSent_string.concat(pageName);
  
    for (unsigned int i = 0; i < ToBeSent_string.length(); i++)
    {
        Serial5.write(ToBeSent_string[i]);
    }
    
    Serial5.write(0xff);
    Serial5.write(0xff);
    Serial5.write(0xff);
}

void TransmissionComplete() {

    String ToBeSent_string_1 = "pwm7=50";
   
    for (unsigned int i = 0; i < ToBeSent_string_1.length(); i++)
    {
        Serial5.write(ToBeSent_string_1[i]);
    }
    
    Serial5.write(0xff);
    Serial5.write(0xff);
    Serial5.write(0xff);
  
    delay(50);
  
    String ToBeSent_string_3 = "pwm7=0";
   
    for (unsigned int i = 0; i < ToBeSent_string_3.length(); i++)
    {
        Serial5.write(ToBeSent_string_3[i]);
    }    

    Serial5.write(0xff);
    Serial5.write(0xff);
    Serial5.write(0xff);
}

void setColour (int32_t color){
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
        strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
        strip.show();                          //  Update strip to match
    }
}

void clearColour (){
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
        strip.setPixelColor(i, 0);         //  Set pixel's color (in RAM)
        strip.show();                          //  Update strip to match
    }
}

void LoRa_rxMode(){
    LoRa.disableInvertIQ();               // normal mode
    LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
    LoRa.idle();                          // set standby mode
    LoRa.enableInvertIQ();                // active invert I and Q signals
}

void onTxDone() {
    Serial.println("\nTransmission to Node Complete");
    LoRa_rxMode();
}

void onReceive(int packetSize) {   

    // read packet
    for (int i = 0; i < packetSize; i++) {
    incomming_data += (char)LoRa.read();
    }
}

String readStringEEPROM(){
    String data = "";
    for(unsigned i = 0; i < eeprom_limit; i++)
    {
        int r = EEPROM.read(i);
        data.concat(char(r));
    }
    return data;
}

void writeStringEEPROM(String data){
    for(unsigned i = 0; i < eeprom_limit; i++)
    {
      EEPROM.write(i, data[i]);
    }
}


void setup() {
    // read last paired node from EEPROM
    paired_node = readStringEEPROM();
    Serial.println(paired_node);

    // POWER ON indicator
    strip.begin();                      // INITIALIZE NeoPixel strip object (REQUIRED)
    strip.show();                       // Update Neopixel to current data
    strip.clear();                      // Clear old/garbage data
    strip.show();                       // Update Neopixel to current data
    strip.setBrightness(50);            // Set brightness to 50/255 (8 bit value)
    setColour(GREEN);

    // UART port initialization
    Serial.begin(115200); // native Serial used for USB 
    Serial5.begin(921600); // hardware serial for Display

    // LORA initialization
    LoRa.setPins(LORA_SLAVE_SELECT, LORA_RESET, LORA_INTERRUPT);  
    if (!LoRa.begin(433E6)) {
        Serial.println("Starting LoRa failed!");
        setColour(RED);
        changePage("no_com_page");
        while (true){
            TransmissionComplete();
            delay(1000);
        };  
    }
    else{
        Serial.println("Starting LoRa RFM96W Successful!");
    } 

    LoRa.setSyncWord(SYNC_ID); 

    LoRa.onReceive(onReceive);
    LoRa.onTxDone(onTxDone);
    LoRa_rxMode();

    pinMode(RELAY_CONTROL, OUTPUT);
    pinMode(SIREN_CONTROL, OUTPUT);

    setColour(YELLOW);

    changePage("init_page");
    writeString("home_page.node_id", paired_node);    
    writeString("data_page.firmware", FIRMWARE_VERSION);  

}

void loop() { 

    /*     COMMANDS FROM DISPLAY       
 
    A + <4 byte long> = LOW THRESHOLD OF LENGTH
    B + <4 byte long> = HIGH THRESHOLD OF LENGTH
    C + <4 byte long> = LOW THRESHOLD OF TENSION
    D + <4 byte long> = NEW DIAMETER OF PULLEY
    E + <4 byte long> = HIGH THRESHOLD OF TENSION
    F + <4 byte long> = LENGTH ALARM STATUS ( 0 = ON & 1 = OFF)
    G + <4 byte long> = TENSION ALARM STATUS ( 0 = ON & 1 = OFF)
    

    R = RESET LENGTH COMMAND
    N = DO NOT RESET LENGTH COMMAND 
    H = STOP ALARM
    I = DISENGAGE EMERGENCY BRAKE
    J = REQUEST PING BACK WITH IDENTITY
    */    

    if ((message_from_Display.length()>0) && (init_flag)){
        setColour(BLUE);
        if(message_from_Display.substring(0,1)=="R"){
            Serial.println("Reset Length command received");
            message_from_Display = "";

            StaticJsonDocument<200> Reset;

            Reset["message"] = "reset counter";

            String response = "";
            LoRa_txMode(); 
            LoRa.beginPacket();
            LoRa.print("R");
            serializeJson(Reset, LoRa);
            LoRa.endPacket(true);

        }
        else if(message_from_Display.substring(0,1)=="J"){
            
            message_from_Display = "";

            if (Searching_for_nodes){
                Serial.println("Scan for Nodes Completed");
                Serial.println("updating display with "+String(last_node_index)+" node responses");
                
                for(int i = 0; i<last_node_index; i++){                    
                    Serial.println("SET scan_page.t"+String(i)+" as "+String(node_names[i])+" - ("+String(node_rssi[i])+"dBm)");
                    int component_number = 12+i;
                    writeString("scan_page.t"+String(i), "Node: "+String(node_names[i])+" ~ ( "+String(node_rssi[i])+"dBm )");
                    makeVissible("t0");
                    if (node_rssi[i]<-80){
                        changeImage("p"+String(component_number), RSSI_LOW);
                    }
                    else if (node_rssi[i]<-50){
                        changeImage("p"+String(component_number), RSSI_AVERAGE);
                    }
                    else{
                        changeImage("p"+String(component_number), RSSI_HIGH);
                    }
                    makeVissible("p"+String(component_number));                    
                }
                last_node_index=0;
                Searching_for_nodes=false;
            }
            else{
                Serial.println("Scan for Nodes...");
                StaticJsonDocument<200> Scan;

                Scan["message"] = "Scanning for Nodes";

                LoRa_txMode(); 
                LoRa.beginPacket();
                LoRa.print("J");
                serializeJson(Scan, LoRa);
                serializeJson(Scan, Serial);
                LoRa.endPacket(true);

                Searching_for_nodes = true;
                last_node_index=0;
            }
            

        }
        else if(message_from_Display.substring(0,1)=="N"){
            Serial.println("Don't Reset command received");
            message_from_Display = "";                     
        }
        else if(message_from_Display.substring(0,1)=="H"){
            Serial.println("Stop Alarm command received");
            message_from_Display = "";   
            analogWriteFrequency(SIREN_CONTROL, 0);
            analogWrite(SIREN_CONTROL, 0);   
        }
        else if(message_from_Display.substring(0,1)=="I"){
            Serial.println("Disengage emergency brake command received");
            message_from_Display = "";
            digitalWrite(RELAY_CONTROL,LOW);
            analogWriteFrequency(SIREN_CONTROL, 0);
            analogWrite(SIREN_CONTROL, 0);  

        }
        else if (message_from_Display.length()==5){

            value.charByte[0]=char(message_from_Display[1]);
            value.charByte[1]=char(message_from_Display[2]);
            value.charByte[2]=char(message_from_Display[3]);
            value.charByte[3]=char(message_from_Display[4]);

            switch (char(message_from_Display[0])) {
                case 'A':
                    Serial.println("New Minimum Length Threshold : "+String(value.valLong));
                    low_threshold_length = int(value.valLong);
                    break;
                case 'B':
                    Serial.println("New Maximum Length Threshold : "+String(value.valLong));
                    high_threshold_length = int(value.valLong);
                    break;
                case 'C':
                    Serial.println("New Minimum Tension Threshold : "+String(value.valLong));
                    low_threshold_tension = int(value.valLong);
                    break;
                case 'D':
                    Serial.println("New Diameter : "+String(value.valLong));
                    diameter = int(value.valLong);
                    break;
                case 'E':
                    Serial.println("New Maximum Tension Threshold : "+String(value.valLong));
                    high_threshold_tension = int(value.valLong);
                    break;
                case 'F':
                    Serial.println("New Length Alarm Status : "+String(value.valLong));
                    if (int(value.valLong) == 0) length_alarm = true;
                    else length_alarm = false;
                    break;
                case 'G':
                    Serial.println("New Tension Alarm Status : "+String(value.valLong));
                    if (int(value.valLong) == 0) tension_alarm = true;
                    else tension_alarm = false;
                    break;
                case 'L':
                    Serial.println("Got New Node Choice : "+String(value.valLong)+"\t --> "+String(node_names[int(value.valLong)]));
                    writeStringEEPROM(String(node_names[int(value.valLong)]));
                    paired_node = String(node_names[int(value.valLong)]);
                    writeString("home_page.node_id", paired_node);                    
                    break;
                default:
                    break;                
            }
            message_from_Display = "";
        }
        else if(message_from_Display.length()>5){
            message_from_Display = "";
        }
        clearColour();
    }
    else if((!init_flag) && (message_from_Display.length()==35)){
        
        Serial.println("Got configuration data ...");
        
        for (int i=0; i < 35; i=i+5) {
            value.charByte[0]=char(message_from_Display[i+1]);
            value.charByte[1]=char(message_from_Display[i+2]);
            value.charByte[2]=char(message_from_Display[i+3]);
            value.charByte[3]=char(message_from_Display[i+4]);
            Serial.println(String(message_from_Display[i])+" : "+String(value.valLong));

            switch (char(message_from_Display[i])) {
                case 'A':
                    low_threshold_length = int(value.valLong);
                    break;
                case 'B':
                    high_threshold_length = int(value.valLong);
                    break;
                case 'C':
                    low_threshold_tension = int(value.valLong);
                    break;
                case 'D':
                    diameter = int(value.valLong);
                    break;
                case 'E':
                    high_threshold_tension = int(value.valLong);
                    break;
                case 'F':
                    if (int(value.valLong) == 0) length_alarm = true;
                    else length_alarm = false;
                    break;
                case 'G':
                    if (int(value.valLong) == 0) tension_alarm = true;
                    else tension_alarm = false;
                    break;
                default:
                    break;
            }
        }                
        init_flag=true;
        message_from_Display="";
        changePage("init_load_page");
        clearColour();
    }

    if (incomming_data.length()) {
        
        // received a packet
        Serial.print("Received packet : ");

        char data_type = incomming_data.charAt(7);        

        String NodeID = incomming_data.substring(0, 7);

        incomming_data.remove(0,8);  
        
        DynamicJsonDocument doc(200);
        DeserializationError error = deserializeJson(doc, incomming_data);
        
        double rssi_value = LoRa.packetRssi();
        double snr_value = LoRa.packetSnr();
        double fe_value = LoRa.packetFrequencyError();
        
        if (!error){

            Serial.println(incomming_data);
            Serial.println("rssi : "+String(rssi_value)+"dBm");
            Serial.println("snr : "+String(snr_value)+"dB");
            Serial.println("fe : "+String(fe_value)+"Hz");
            Serial.println("data prefix : "+String(data_type));
            Serial.println("Node is : "+NodeID);
            
            //    DIAGNOSTICS PANEL           
            writeString("home_page.rssi_txt", String(int(rssi_value))+"dBm"); 
            writeString("home_page.snr_txt", String(int(snr_value))+"dB"); 
            writeString("home_page.fe_txt", String(fe_value)+"Hz");  
             
            writeString("data_page.rssi_txt", String(int(rssi_value))+"dBm"); 
            writeString("data_page.snr_txt", String(int(snr_value))+"dB"); 
            writeString("data_page.fe_txt", String(fe_value)+"Hz");   
             
            writeValue("home_page.rsi_bar", String(int(map(rssi_value, -120, -30, 0, 100)))); 
            writeValue("home_page.snr_bar", String(int(map(snr_value,  -20, 10, 0, 100))));  
            

            double soc_val = doc["SOC"];
            double voltage_val = doc["Voltage"];            
            int latest_counter = doc["C"];
            double temperature = doc["T"];
            double humidity = doc["RH"];  

            float soc_bar = constrain(soc_val, 0.0, 100.0);    
            double length_value = 0.5*latest_counter*PI_VALUE; 
            int time_value = (int)(millis() - duration);
            double speed_value = (500.0*PI)/time_value;
            double tension_value = doc["LC"];    
            String last_message = "{C:"+String(latest_counter)+",LC:"+String(tension_value)+",SOC:"+String(soc_val)+",T:"+String(temperature)+",RH:"+String(humidity)+"}";   

            String Node_ID = doc["Node"];

            if (data_type=='K'){
                Serial.print("Node Identity Data Recieved : ");
                node_names[last_node_index]=String(Node_ID);
                node_rssi[last_node_index]=int(rssi_value);
                last_node_index++;
            }   

            else if (paired_node=NodeID){

                switch(data_type){
                    case 'A': {
                            Serial.println("Transmitter reset");                         
                    }break;                    
                    case 'B': {
                            Serial.println("charging");  
                            changePage("charging_page");                          
                                
                            writeString("charging_page.soc_txt", String(soc_bar)+"%"); 
                            writeValue("charging_page.soc_bar", String(int(soc_bar)));
                            writeString("charging_page.voltage_txt", String(voltage_val)+"V");
                            int max_value = (soc_val/100)*5;
                            setColour(GREEN);
                            clearColour();
                            for(int i=0; i<max_value; i++) {
                                strip.setPixelColor(i, GREEN);                      
                                strip.show();
                                delay(200);
                            }

                    }break;                    
                    case 'C': {
                        Serial.println("Done Charging");
                        changePage("charged_page");
                        
                    }break;                   
                    case 'D': {

                        if (waiting_for_reset_response){
                            Serial.println("got reset confirmation");
                        }
                        // update timer for speed calculations
                        duration = millis();

                        Serial.println("HE data");  
                        
                        //    DATA PAGE                   
                        writeString("data_page.clicks", String(latest_counter)+" clicks"); 
                        writeString("data_page.actual_length", String(length_value)+"m"); 
                        writeString("data_page.Temperature", String(temperature)+" 'C"); 
                        writeString("data_page.Humidity", String(humidity)+" %"); 
                        writeString("data_page.StateOfCharge", String(soc_val)+" %");
                        writeString("data_page.last_message", last_message);                     
                                            
                        //    BATTERY PANEL                      
                        writeString("home_page.bat_txt", String(soc_val)+"%"); 
                        writeValue("home_page.bat_bar", String(int(soc_val)));         
                        
                        //    LENGTH PANEL                   
                        writeString("home_page.len_txt", String(length_value));

                        //    SPEED PANEL                 
                        writeString("home_page.speed_txt", String(speed_value));

                        // alarm if threshhold is triggered
                        Serial.print(length_value);
                        Serial.print("\t");
                        Serial.print(low_threshold_length);
                        Serial.print("\t");
                        Serial.println(high_threshold_length);                    
                        if ( length_value < (double)low_threshold_length){
                            setColour(RED);
                            if(length_alarm){
                                digitalWrite(RELAY_CONTROL,HIGH);                            
                                analogWriteFrequency(SIREN_CONTROL, 2730);
                                analogWrite(SIREN_CONTROL, 128);
                                changePage("emergency_page");
                            }                        
                        }
                        else if ( length_value > (double)high_threshold_length){
                            setColour(RED);
                            if(length_alarm){
                                digitalWrite(RELAY_CONTROL,HIGH);                            
                                analogWriteFrequency(SIREN_CONTROL, 2730);
                                analogWrite(SIREN_CONTROL, 128);
                                changePage("emergency_page");
                            }   
                        }  
                        else{
                            clearColour();
                            analogWrite(SIREN_CONTROL, 0);
                            digitalWrite(RELAY_CONTROL,LOW);
                        }        
                    }break;
                    case 'E': {
                        Serial.println("Charger Plugged-In");
                        changePage("charger_page");
                        writeString("charger_page.status_txt", String("Charger Plugged-In"));                    
                    }break;
                    case 'F': {
                        Serial.println("Charger Un-Plugged");
                        changePage("charger_page");
                        writeString("charger_page.status_txt", String("Charger Un-Plugged"));
                        delay(1000);
                        changePage("home_page");                    
                    }break;
                    case 'G': {
                        Serial.println("LC data");     
                        //    TENSION PANEL                             
                        writeString("home_page.ten_txt", String(tension_value,4));

                        //    DATA PAGE    
                        writeString("data_page.lc_txt", String(tension_value,4)+" T");                
                        writeString("data_page.Temperature", "T: "+String(temperature)+" Celcius"); 
                        writeString("data_page.Humidity", "rH: "+String(humidity)+" %"); 
                        writeString("data_page.StateOfCharge", String(soc_val)+" %");
                        writeString("data_page.last_message", last_message);  

                        // alarm if threshhold is triggered
                        Serial.print(tension_value);
                        Serial.print("\t");
                        Serial.print(low_threshold_tension);
                        Serial.print("\t");
                        Serial.println(high_threshold_tension);
                        if (tension_value < (double)low_threshold_tension){
                            setColour(RED);
                            if(tension_alarm){
                                digitalWrite(RELAY_CONTROL,HIGH);                            
                                analogWriteFrequency(SIREN_CONTROL, 2730);
                                analogWrite(SIREN_CONTROL, 128);
                            }   
                        }
                        else if ( tension_value > (double)high_threshold_tension){
                            setColour(RED);
                            if(tension_alarm){
                                digitalWrite(RELAY_CONTROL,HIGH);                            
                                analogWriteFrequency(SIREN_CONTROL, 2730);
                                analogWrite(SIREN_CONTROL, 128);
                            } 
                        }
                        else{
                            clearColour();
                            analogWrite(SIREN_CONTROL, 0);
                            digitalWrite(RELAY_CONTROL,LOW);
                        }                   
                    }break;                
                    case 'T': {
                        Serial.println("Telemetry Data Recieved");
                    }break;
                    default: {
                        Serial.println("Unknown Command");
                        Serial.println();
                    }break;                 
                }   
            }               
        }
        // catch if data cannot be deserialized!!
        else {             
            Serial.println(incomming_data);
            Serial.println("rssi : "+String(rssi_value)+"dBm");
            Serial.println("snr : "+String(snr_value)+"dB");
            Serial.println("fe : "+String(fe_value)+"Hz");
            Serial.print("deserializeJson() failed: ");
            Serial.println(error.c_str());
        }
        Serial.println("Done Processing command");    
        incomming_data = "";       
    }

    if (checkTelemetry()){

        Serial.print("Requesting for telemetry data");

        StaticJsonDocument<200> Telemetry;

        Telemetry["message"] = "retrive telemetry";

        String response = "";
        LoRa_txMode(); 
        LoRa.beginPacket();
        LoRa.print("T");
        serializeJson(Telemetry, LoRa);
        LoRa.endPacket(true);

    }
}
