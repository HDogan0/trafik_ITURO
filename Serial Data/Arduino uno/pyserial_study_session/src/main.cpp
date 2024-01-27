#include <Arduino.h>

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

uint8_t my_char;
char buffer[20];
uint8_t idx = 0;
void loop() {
  while(Serial.available()>0){
    my_char =  Serial.read();
    if(my_char == '\n'){
      Serial.println(buffer);
      memset(&buffer, 0, sizeof(buffer));
      idx = 0;
    }
    else{
      buffer[idx] = my_char;
      idx ++;
    }
  }
}