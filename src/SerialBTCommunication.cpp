
#include "ICommunication.h"
#include<String.h>
#include"BluetoothSerial.h"
#include<config.h>

class BTSerialCommunication : public ICommunication {
  private:
    bool m_isOpen;
    BluetoothSerial m_SerialBT;
    
  public:
    BTSerialCommunication() {
      m_isOpen = false;
    }

    bool isOpen(){
      return m_isOpen;
    }

    void start(){
      m_SerialBT.begin(BTSERIAL_DEVICE_NAME);
      #if BT_ECHO
      Serial.begin(SERIAL_BAUD_RATE);
      Serial.println("The device started, now you can pair it with bluetooth!");
      #endif
      m_isOpen = true;
    }

    void output(char* data){
      m_SerialBT.print(data);
      #if BT_ECHO
      Serial.print(data);
      Serial.flush();
      #endif
    }

    bool readData(char* input){
      /*byte size = m_SerialBT.readBytesUntil('\n', input, 100);
      input[size] = NULL;*/
      String message = m_SerialBT.readStringUntil('\n');
      strcpy(input, message.c_str());
      return input != NULL && strlen(input) > 0;
    }
};