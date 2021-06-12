#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneButton.h>
#include <Rotary.h>

Rotary r = Rotary(2, 3);
OneButton button(10,true);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
// static const unsigned char PROGMEM logo_bmp[] =
// { B00000000, B11000000,
//   B00000001, B11000000,
//   B00000001, B11000000,
//   B00000011, B11100000,
//   B11110011, B11100000,
//   B11111110, B11111000,
//   B01111110, B11111111,
//   B00110011, B10011111,
//   B00011111, B11111100,
//   B00001101, B01110000,
//   B00011011, B10100000,
//   B00111111, B11100000,
//   B00111111, B11110000,
//   B01111100, B11110000,
//   B01110000, B01110000,
//   B00000000, B00110000 };

boolean isWorking, isErr;
int input = -2;
int count = 1;


void setup() {
  Serial.begin(9600);
  //rotary interrupt
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();

  // click on the encoder
  button.attachDoubleClick(doubleclick);
  button.attachClick(singleclick);

  Serial.println("Testing Rotary with SSD1306 OLED Screen");
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  display.setTextColor(WHITE);
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  delay(2000);
}
 void loop() {
     if (isWorking != true) {
         button.tick();
         switch (input) {
             case 2:        //"DOUBLE_CLICK":
                display.clearDisplay();
                display.setCursor(10,3);
                display.println("Double clicked");
                display.display();
                break;
             case 0:        //"SINGLE_CLICK":
                display.clearDisplay();
                display.setCursor(10,3);
                display.println("Clicked");
                display.display();
                break;
             case 1:        //"CLOCKWISE":
                if (count !=3) {
                    count++;
                    displaySetup(count);
                }
                else{
                    count = 1;
                    displaySetup(count);
                }
                break;
             case -1:       //"COUNTERCLOCK":
                if (count !=1) {
                    count--;
                    displaySetup(count);
                }
                else{
                    count = 3;
                    displaySetup(count);
                }
                break;
         }
     }

     else {
         // Disable all input from Rotary
         isWorking = false;
     }
 }

 void doubleclick() {
     input = 2;
     Serial.println("DOUBLE_CLICK");
 }

 void singleclick() {
     input = 0;
     Serial.println("SINGLE_CLICK");
 }

 ISR(PCINT2_vect) {
   unsigned char result = r.process();
   if (result == DIR_NONE) {
   }
   else if (result == DIR_CW) {
     input = 1;
     Serial.println("CLOCKWISE");
   }
   else if (result == DIR_CCW) {
     input = -1;
     Serial.println("COUNTERCLOCK");
   }
 }

void displaySetup(int cursorPos) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(2);
    display.println("SELECT OUTPUT TEMP:");

    display.setCursor(10,20);
    display.setTextSize(1);
    display.println("33");
    display.write(248);
    display.println("C");

    display.setCursor(10,30);
    display.setTextSize(1);
    display.println("36");
    display.write(248);
    display.println("C");

    display.setCursor(10,40);
    display.setTextSize(1);
    display.println("39");
    display.write(248);
    display.println("C");

    display.setCursor(2, (cursorPos*10)+10);
    display.write(16);

    display.display();
}
