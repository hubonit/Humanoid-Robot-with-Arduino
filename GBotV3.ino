/*
Humanoid Robot with Arduino

Created By Hub on IT
22/5/2015

Dynamixel AX-12 Pin Connections
-------------------------------
GND -> GND (From DC Power Supply) and Arduino GND Pin
VDD -> +9.6 - +12V (From DC Power Supply)
Data -> Arduino Digital Pin 1 (D1)

Arduino Pin Connections
-----------------------
D0  - Serial Receive Pin
D1  - Serial Transmit Pin
D2  - Control Pin (Half Duplex Serial Communication: Transmit -> High, Receive -> Low)
D12 - Push Button 1 
D13 - Push Button 2

Documentation:
to http://hubonit.com/ideas/?p=126


This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
  
  int current_buttonState[] = {0, 0, 0};         // pushbutton status
  int last_buttonState[] = {0, 0, 0};
  
  long lastDebounceTime = 0;  // previous output pin was toggled
  long debounceDelay = 500;    // switch debounce
  
  byte ServoID = 0;
  byte bLength = 0;
  byte checksum = 0;
  
  byte Tx_msg1[] = {0xFF, 0xFF, ServoID, bLength, 0x03, 0x19, 0x01, checksum}; //LED ON
  byte Tx_msg2[] = {0xFF, 0xFF, ServoID, bLength, 0x03, 0x19, 0x00, checksum}; //LED OFF
  byte Tx_msg3[] = {0xFF, 0xFF, ServoID, bLength, 0x02, 0x00, 0x03, checksum}; //Model Number
  byte Tx_msg4[] = {0xFF, 0xFF, ServoID, bLength, 0x00, 0x74, 0x00, 0x08, checksum}; //Firmware
  byte Tx_msg5[] = {0xFF, 0xFF, ServoID, bLength, 0x01, checksum}; //Status
  byte Tx_msg6[] = {0xFF, 0xFF, ServoID, bLength, 0x03, 0x1E, 0x00, 0x02, 0x00, 0x02, checksum}; // Position the output from 0 to 180 degree at velocity of 57RPM (0x200)
  
  char inputBuffer[32];   // Incoming AX12 data
  
  #define buttonPin1 12
  #define buttonPin2 13
  
  #define TxRx_Pin 2  //Half Duplex Serial Control Digital Pin = 2
  
  void setup() {

  #if defined (ARDUINO_AVR_YUN)
  Serial.begin(57600);
  Serial1.begin(1000000);
  Serial1.setTimeout(1);
  #elif defined(__AVR_ATmega328P__)
  Serial.begin(1000000);
  Serial.setTimeout(1);
  #endif
  
  pinMode(TxRx_Pin, OUTPUT);
  
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  
  }
  
  void loop() {
     current_buttonState[0] = digitalRead(buttonPin1);
     current_buttonState[1] = digitalRead(buttonPin2);
  
    if (current_buttonState[0] == HIGH && last_buttonState[0] == LOW && (millis() - lastDebounceTime) > debounceDelay) {
      transmit(0x11, Tx_msg1, sizeof(Tx_msg1), "LED ON");
      transmit(0x11, Tx_msg3, sizeof(Tx_msg3), "Model Number"); 
      //transmit(0x11, Tx_msg5, sizeof(Tx_msg5), "Status");  
      lastDebounceTime = millis();
    }
    else if (current_buttonState[1] == HIGH && last_buttonState[1] == LOW && (millis() - lastDebounceTime) > debounceDelay) {
    transmit(0x11, Tx_msg2, sizeof(Tx_msg2), "LED OFF"); 
    transmit(0x11, Tx_msg4, sizeof(Tx_msg4), "Firmware");  
    //transmit(0x11, Tx_msg6, sizeof(Tx_msg6), "Set angular and Speed"); 
    lastDebounceTime = millis();
    }
    
    last_buttonState[0] = current_buttonState[0];
    last_buttonState[1] = current_buttonState[1];
  }
  
  void transmit(byte ID, byte x[], int n, char *s)
  {
  ServoID = ID;
  x[2] = ServoID;
  
  bLength = n - 4;
  x[3] = bLength;
  
  checksum = 0;
  for (int i = 0; i < n - 1; i++)
  {
   if (i > 1) checksum += x[i];
  }
  
  checksum = ~checksum;
  x[n - 1] = checksum;
  
  #if defined (ARDUINO_AVR_YUN)
  digitalWrite(TxRx_Pin, HIGH); //Transmit 
  Serial1.write(x, n);
  Serial.println(s);
  digitalWrite(TxRx_Pin, LOW); //Receive
  if (Serial1.available())
  {
    Serial1.readBytes(inputBuffer,Serial1.available());
    digitalWrite(TxRx_Pin, HIGH); //Transmit 
    Serial.println(inputBuffer);
  }
  Serial1.flush();
  #elif defined(__AVR_ATmega328P__)
  digitalWrite(TxRx_Pin, HIGH); //Transmit 
  Serial.write(x, n);
  digitalWrite(TxRx_Pin, LOW); //Receive 
  if(Serial.available())
  {
    Serial.readBytes(inputBuffer,Serial.available());
    digitalWrite(TxRx_Pin, HIGH); //Transmit 
    Serial.println(inputBuffer);
  }
  Serial.flush();
  #endif
 }
