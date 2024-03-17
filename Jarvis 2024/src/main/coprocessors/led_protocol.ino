#include<Adafruit_NeoPixel.h>

#define LED_COUNT 255
#define LED_PIN 3
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
byte LED_COLOR_RX[7];

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  LED_COLOR_RX[0] = 0x00;
  LED_COLOR_RX[1] = 0x00;
  LED_COLOR_RX[2] = LED_COUNT;
  LED_COLOR_RX[3] = LED_COUNT >> 8;
  LED_COLOR_RX[4] = 0x00;
  LED_COLOR_RX[5] = 0x00;
  LED_COLOR_RX[6] = 0x00;

  strip.begin();
  updateStripColor();
}

void loop() {
  if (Serial.available()) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.readBytes(LED_COLOR_RX, 7);
    Serial.flush();
    updateStripColor();
    digitalWrite(LED_BUILTIN, LOW);
  } 
}

void updateStripColor() {
  uint16_t start = (uint16_t)LED_COLOR_RX[1] << 8 | (uint16_t)LED_COLOR_RX[0];
  uint16_t end = (uint16_t)LED_COLOR_RX[3] << 8 | (uint16_t)LED_COLOR_RX[2];
  if (start < end) {
    for (uint16_t i = start; i <= end; i++)
      strip.setPixelColor(i, LED_COLOR_RX[4], LED_COLOR_RX[5], LED_COLOR_RX[6]);
    strip.show();
  }
}