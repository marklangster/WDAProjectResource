#include <SPI.h>
#include <LoRa.h>
#include <assert.h>
#include <FreeRTOS_ARM.h>
#include <IPAddress.h>
#include <PowerDueWiFi.h>
//#include <String.h>


#define WIFI_SSID "PowerDue"
#define WIFI_PASS "powerdue"


#define SERVER_PORT 9999
#define SERVER_IP "172.29.93.49" 

/*------------------------------------------------------------*/

// parameters
#define FREQUENCY         915E6   // 915MHz or 920MHz
#define BANDWIDTH         125000  // 125kHz bandwidth

// vary these parameters
#define TX_POWER          20   // valid values are from 6 to 20
#define SPREADING_FACTOR  7    // valid values are 7, 9 

#define DATALEN 30

// passcode and response code 
#define PASS1 81
#define PASS2 87
#define RESPONSE 200


/*------------------------------------------------------------*/
char buf[DATALEN]; 

//<------ write your function here -------->
void getAnswer(){
  String answer = "team Green: ";
  uint8_t data;
  uint8_t message[6];
  int i;
  bool correct = false;

  SerialUSB.println("Ready to receive!");

  // loop until a message from the correct sender is received
  while(!correct){
    i = 0;
    int packetSize = 0;
    // wait for a packet of the correct size (6 bytes)
    while(packetSize != 6){
      packetSize = LoRa.parsePacket();
      delay(1);
    }

    // received a packet
    //SerialUSB.println("Received Packet");

    // read packet and put message into array
    while (LoRa.available()) {
      data = (uint8_t)LoRa.read();
      if (i < 6){
        message[i] = data;
        i++;
      }
    }
    // check if the message is from the correct sender
    if(message[0] == PASS2 && message[1] == PASS1){
      correct = true;
      //SerialUSB.println("password correct");
    }

  }

  // extrace the frequencies from the message and add to answer
  uint16_t freq1 = (uint16_t)(message[2] << 8 | message[3]);
  uint16_t freq2 = (uint16_t)(message[4] << 8 | message[5]);
  answer += String(freq1) + ", " + String(freq2);

  // write answer to buffer to be sent over TCP
  answer.toCharArray(buf, DATALEN);

  // send response to transmitter
  LoRa.beginPacket();
  LoRa.print((char)RESPONSE);
  LoRa.endPacket();

  //SerialUSB.println(answer);

}



void tcpClient(void * argument)
{  
  // the receiver runs continuously and doesn't need to restart 
  // after each round
  while(true){   

    struct sockaddr_in serverAddr;  
    socklen_t socklen;
    memset(&serverAddr, 0, sizeof(serverAddr));

    serverAddr.sin_len = sizeof(serverAddr);
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT); 
    inet_pton(AF_INET, SERVER_IP, &(serverAddr.sin_addr)); //another powerdue running as server


    int s = lwip_socket(AF_INET, SOCK_STREAM, 0);

    while(lwip_connect(s, (struct sockaddr *)&serverAddr, sizeof(serverAddr))){
      lwip_close(s);
      SerialUSB.println("Failed to connect to server. Retrying...");
      s = lwip_socket(AF_INET, SOCK_STREAM, 0);
      delay(1000);
      
    }
    SerialUSB.println("Connected to server");
    
    //<------ Call your function here -------->
    getAnswer();  // function call

    // send data  
    if (lwip_write(s, buf, DATALEN)){
      SerialUSB.println("sent");
    }else{
      SerialUSB.println("failed to send");
    }
    
    SerialUSB.println(buf);

    // close socket after everything is done
    lwip_close(s);
    SerialUSB.println("socket closed");

  }

}

/*------------------------------------------------------------*/

void initLEDs(){
  // turn off LEDs
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  turnOffLEDs();
}

void turnOffLEDs(){
  digitalWrite(6,LOW);
  digitalWrite(7,LOW);
  digitalWrite(8,LOW);
}

void turnOnLEDs(){
  digitalWrite(6,HIGH);
  digitalWrite(7,HIGH);
  digitalWrite(8,HIGH);
}

void onError(int errorCode){
  SerialUSB.print("Error received: ");
  SerialUSB.println(errorCode);
}

void onReady(){
  SerialUSB.println("Device ready");  
  SerialUSB.print("Device IP: ");
  SerialUSB.println(IPAddress(PowerDueWiFi.getDeviceIP()));  
  
  xTaskCreate(tcpClient, "tcpClient", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

void setup() {

  LoRa.setPins(22, 59, 51);
    if(!LoRa.begin(FREQUENCY)) {
    SerialUSB.println("Starting LoRa failed!");
    while(1);
  }

  // set the LoRa parameters
  LoRa.setTxPower(TX_POWER);
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.setSignalBandwidth(BANDWIDTH);
  LoRa.enableCrc();
  
  initLEDs();
  
  SerialUSB.begin(9600);
  while(!SerialUSB);
  SerialUSB.println("Program Begin");
  turnOnLEDs();
  
  PowerDueWiFi.init(WIFI_SSID, WIFI_PASS);
  PowerDueWiFi.setCallbacks(onReady, onError);
   
  vTaskStartScheduler();
  SerialUSB.println("Insufficient RAM");
  while(1);
}

void loop() {
  // not used in freertos
}
