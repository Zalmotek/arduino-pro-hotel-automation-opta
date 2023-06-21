#include <ArduinoRS485.h>

#include "arduino_secrets.h"
#include "thingProperties.h"

constexpr auto baudrate{ 115200 };

constexpr auto bitduration{ 1.f / baudrate };
constexpr auto wordlen{ 9.6f };  // OR 10.0f depending on the channel configuration
constexpr auto preDelayBR{ bitduration * wordlen * 3.5f * 1e6 };
constexpr auto postDelayBR{ bitduration * wordlen * 3.5f * 1e6 };

char buff[17];
int idx, adding;


void setup() {
  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud connection and errors
     the higher number the more granular information you’ll get.
     The default is 0 (only errors).
     Maximum is 4
  */
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  // Initialize serial and wait for port to open:
  // Serial.begin(baudrate);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  // delay(1500);

  pinMode(D0, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(LED_D0, OUTPUT);
  pinMode(LED_D2, OUTPUT);
  pinMode(LED_D3, OUTPUT);

  RS485.begin(baudrate);
  RS485.setDelays(preDelayBR, postDelayBR);

  // Defined in thingProperties.h
  initProperties();
}

double prevTemperature = 0;
int prevHumidity = 0, prevLight = 0;


/* Gonna use the last 5 bits to represent the last 5 values that got sent */
int prevFanStatus = 0, prevLightStatus = 0, prevPir = 0, prevLockStatus = 0;

int Convert(int len, int prevVal = 0) {
  int ret = 0, aux = 0;
  int p[5] = {1, 10, 100, 1000, 10000};
  for (int i = 1; i <= len; i++) {
    if ('0' <= buff[idx] && buff[idx] <= '9') {
      ret += (buff[idx++] - '0') * p[len - i];
    }
    else {
      idx++;
      aux += ((prevVal / p[len - i]) % 10) * p[len - i];
    }
  }
  if (aux == 0) {
    return ret;
  }

  /* Weighted average between previous values and the reconstructed one */
  return (int)(prevVal * 0.7 + (ret + aux) * 0.3);
}

/* Specifically for 5 values => mask of 31 */
void BinaryEC(int &prev, bool &curr) {
  prev = (((prev << 1) ^ curr) & 31);
  int cnt = 0, aux = prev;
  while (aux) {
    cnt += aux % 2;
    aux /= 2;
  }
  curr = (cnt >= 3);
}

void GetData() {
  temperature = 0;
  humidity = 0;
  light = 0;
  fanStatus = 0;
  lightStatus = 0;
  pir = 0;
  lockStatus = 0;

  temperature = Convert(3, prevTemperature * 10) / 10.0;
  humidity = Convert(2, prevHumidity);
  fanStatus = Convert(1);
  lightStatus = Convert(1);
  light = Convert(4, prevLight);
  pir = Convert(1);
  lockStatus = Convert(1);

  if (light > 1024) {
    light = light % 1000;
  }

  prevTemperature = temperature;
  prevHumidity = humidity;
  prevLight = light;

  /* Error correction for the binary values - best of 5 (it will lead to small delays when one is truly updated) */
  BinaryEC(prevFanStatus, fanStatus);
  BinaryEC(prevLightStatus, lightStatus);
  BinaryEC(prevPir, pir);
  BinaryEC(prevLockStatus, lockStatus);

  if (fanStatus == 1) {
    digitalWrite(D2, HIGH);
    digitalWrite(LED_D2, HIGH);
  }
  else if (fanStatus == 0) {
    digitalWrite(D2, LOW);
    digitalWrite(LED_D2, LOW);
  }

  if (lightStatus == 1) {
    digitalWrite(D3, HIGH);
    digitalWrite(LED_D3, HIGH);
  }
  else if (lightStatus == 0) {
    digitalWrite(D3, LOW);
    digitalWrite(LED_D3, LOW);
  }

  if (lockStatus == 0) {
    digitalWrite(D0, HIGH);
    digitalWrite(LED_D0, LOW);
  }
  else if (lockStatus == 1) {
    digitalWrite(D0, LOW);
    digitalWrite(LED_D0, HIGH);
  }

  //  Serial.print("Temperature: "); Serial.println(temperature);
  //  Serial.print("Humidity: "); Serial.println(humidity);
  //  Serial.print("Fan Status: "); Serial.println(fanStatus);
  //  Serial.print("Light Status: "); Serial.println(lightStatus);
  //  Serial.print("Luminosity: "); Serial.println(light);
  //  Serial.print("Motion Dected: "); Serial.println(pir);
  //  Serial.print("Lock Status: "); Serial.println(lockStatus);
  //  Serial.println("");
}

void loop() {
  // Your code here
  RS485.receive();
  auto aval = RS485.available();

  if (aval > 0) {

    char ch = RS485.read();
    if (tolower(ch) == 'e') {
      idx = 0;
      adding = 1;
    }
    else if (tolower(ch) == 'n') {
      idx = 1;
      adding = 1;
    }
    else if (tolower(ch) == 'd') {
      idx = 2;
      adding = 1;
    }
    if (adding) {
      buff[idx++] = ch;
    }
    if (idx == 16) {
      adding = 0;
      idx = 3;
      GetData();
      idx = 0;
    }
    ArduinoCloud.update();

  }
  RS485.noReceive();
}
