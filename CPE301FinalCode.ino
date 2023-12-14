/*
Author: Quinn Contaldi
Author: Garrett Sharp
Author: AJ Di Diana
Author: Patrick Lear
Author: Nate Michelotti 
Lab: Final CPE 301 Project
Date: 12/8/2023
*/
#include <LiquidCrystal.h>
#include <RTClib.h>
#include <DHT.h>
#include <Stepper.h>

// Constants
#define TEMP_THRESHOLD 27  // Temperature threshold
#define WATER_LEVEL_THRESHOLD 150  // Water level threshold
#define STEPS_PER_REV 2038  // Steps per revolution for stepper motor

// Pin assignments
#define DHT_PIN 7
#define DHT_TYPE DHT11
#define MOTOR_PIN 6
#define WATER_LEVEL_PIN 5
#define LED_PINR 2
#define LED_PINY 3
#define LED_PING 4
#define LED_PINB 5
#define BUTTON_ON_OFF 52
#define BUTTON_RESET 53
#define BUTTON_STEPPER_UP 49
#define BUTTON_STEPPER_DOWN 48

volatile bool startPressed = false;
// Global objects
LiquidCrystal lcd(30, 31, 32, 33, 34, 35);
RTC_DS1307 RTC;
DHT dht(DHT_PIN, DHT_TYPE);
Stepper stepper(STEPS_PER_REV, 8, 10, 9, 11);

// states
enum States {
  IDLE,
  DISABLED,
  RUNNING,
  ERROR,
};

States currentState = DISABLED;
States previousState = DISABLED;

void setup(){
  Serial.begin(9600); // Start Serial communication
  lcd.begin(16, 2); // Initialize LCD
  RTC.begin(); // Initialize RTC

  DateTime now = DateTime(2023, 12, 8, 0, 0, 0);
  RTC.adjust(now);

  dht.begin(); // Initialize DHT sensor
  delay(1000);

  // Initialize LED pins as output
  // Set data direction for LED pins (OUTPUT)
  DDRE |= (1 << LED_PINY);  // YELLOW LED on pin 3 (PORTE)
  DDRC |= (1 << LED_PING);  // GREEN LED on pin 4 (PORTC)
  DDRC |= (1 << LED_PINB);  // BLUE LED on pin 5 (PORTC)
  DDRH |= (1 << LED_PINR);  // RED LED on pin 2 (PORTH)
  DDRH |= (1 << 3);  // Set MOTOR_PIN (pin 6, PH3) as output

  // Set BUTTON_ON_OFF and BUTTON_RESET as input
  DDRD &= ~(1 << 2); // Clear bit 2 of DDRD (Data Direction Register for Port D), setting pin 19 (PD2) as input
  EICRA |= (1 << ISC11); // Set the falling edge of INT1 (pin 19) to trigger the interrupt
  EIMSK |= (1 << INT1); // Enable INT1 (pin 19) interrupt
  // Attach the interrupt function
  attachInterrupt(digitalPinToInterrupt(19), handleStartButton, FALLING);
  DDRD &= ~((1 << 5)); 
  // Set BUTTON_STEPPER_UP and BUTTON_STEPPER_DOWN as input
  DDRL &= ~((1 << 0) | (1 << 1)); 

  lcd.write("Nyaa! Starting");
  stepper.setSpeed(10);
}

void loop() {
  handleButtons(); // Handle button inputs

  // checks for state change
  if(currentState != previousState)
  {
    previousState = currentState;
    sendTimeToUART();
  }

  // Handles states
  switch (currentState) {
    case DISABLED:
      handleDisabledState();
      break;
    case IDLE:
      handleIdleState();
      checkWaterLevel();
      controlStepper();
      break;
    case RUNNING:
      handleRunningState();
      checkWaterLevel();
      controlStepper();
      break;
    case ERROR:
      handleErrorState();
      break;
  }
  delay(1000); // Delay for stability
  int sensorValue = analogRead(WATER_LEVEL_PIN);
}

void handleDisabledState() {
  turnLEDOn(3); // Turn on YELLOW LED
  setFanMotor(0); // Activates Flag
  // Disable all monitoring and control functions
  // Wait for start button press to change state
    if (startPressed) {
        // Handle start button press
    startPressed = false; // Reset the flag
    currentState = IDLE; // sets state
  }
}

void handleIdleState() {
  turnLEDOn(2); // Turn on GREEN LED
  updateLCD();
  checkWaterLevel();
  controlStepper();
  // Transition to RUNNING state based on temperature
  if (dht.readTemperature() > TEMP_THRESHOLD) {
    currentState = RUNNING;
  }
    if (startPressed) {
        // Handle start button press
      startPressed = false; // Reset the flag
    currentState = DISABLED;
  }
}

void handleRunningState() {
  turnLEDOn(5); // Turn on BLUE LED
  setFanMotor(0); // Deactivate fan
  updateLCD();
  checkWaterLevel();
  controlStepper();
  // Transition to IDLE state based on temperature
  if (dht.readTemperature() <= TEMP_THRESHOLD) {
    currentState = IDLE;
    setFanMotor(0); // Deactivate fan
  }
    if (startPressed) {
    // Handle start button press
    startPressed = false; // Reset the flag
    currentState = DISABLED;
  }
}

void handleErrorState() {
  turnLEDOn(2); // Turn on RED LED
  setFanMotor(0); // Deactivates Fan
  lcd.clear();
  lcd.print("Error: Low Water");
  // Check reset condition
  if (digitalRead(BUTTON_RESET) && adc_read(WATER_LEVEL_PIN) > WATER_LEVEL_THRESHOLD) {
    currentState = IDLE;
  }
    if (startPressed) {
        // Handle start button press
      startPressed = false; // Reset the flag
    currentState = DISABLED;
  }
}

void handleStartButton() {
    startPressed = true;
}

// Updates Temperature and Humidity then displays using the lcd
void displayTemperatureAndHumidity() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temp);
  lcd.setCursor(0, 1);
  lcd.print("Hum: ");
  lcd.print(hum);
}

// updates Time and Date then displays using the lcd
void displayTimeAndDate() {
    DateTime now = RTC.now();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(now.year(), DEC);
    lcd.print('/');
    lcd.print(now.month(), DEC);
    lcd.print('/');
    lcd.print(now.day(), DEC);
    lcd.setCursor(0, 1);
    lcd.print(now.hour(), DEC);
    lcd.print(':');
    lcd.print(now.minute(), DEC);
    lcd.print(':');
    lcd.print(now.second(), DEC);
}

// Continuesly updates the LCD screan avoiding using delay function in order to keep button functionality
void updateLCD() {
    static unsigned long lastSwitchTime = 0;
    const long interval = 6000;  // 1 minute
    unsigned long currentMillis = millis();

    if (currentState == DISABLED || currentState == ERROR) {
        return;  // Don't update LCD in DISABLED or ERROR states
    }

    if (currentMillis - lastSwitchTime >= interval) {
        lastSwitchTime = currentMillis;

        // Switch between displaying temperature/humidity and time/date
        static bool displayTempHum = true;
        if (displayTempHum) {
            displayTemperatureAndHumidity();
        } else {
            displayTimeAndDate();
        }
        displayTempHum = !displayTempHum;
    }
}

// Water Level Monitoring
void checkWaterLevel() {
  unsigned int waterLevel = adc_read(WATER_LEVEL_PIN);
  if (waterLevel <= WATER_LEVEL_THRESHOLD) {
    currentState = ERROR;
  }
}

// Stepper Motor Control
void controlStepper() {
    int stepperDirection = 0;
    if (PINL & (1 << (1))) { // Check if BUTTON_STEPPER_DOWN is pressed
        stepperDirection = -1;

    } else if (PINL & (1 << (0))) { // Check if BUTTON_STEPPER_UP is pressed
        stepperDirection = 1;
    }

 if (stepperDirection != 0) {
        stepper.step(stepperDirection * 100); // Adjust step size as needed

        // Print time and direction to serial monitor
        DateTime now = RTC.now();
        char buffer[20];
        sprintf(buffer, "%02d:%02d:%02d ", now.hour(), now.minute(), now.second());
        sendStringToUART(buffer);

        // sends to sieral monitor which direction the step motor is turning
        if (stepperDirection == 1) {
            sendStringToUART("Right\n");
        } else {
            sendStringToUART("Left\n");
        }
    }
}

// Is used to continuesly check for button press
void handleButtons() {
    if (PIND & (1 << 4)) { // Check pin 52 for ON/OFF
        currentState = (currentState == DISABLED) ? IDLE : DISABLED;
    }
    if (currentState == ERROR && (PIND & (1 << 5))) { // Check pin 53 for RESET
        currentState = IDLE;
    }
}




// Method to turn on the specified LED and turn off the other LEDs
void turnLEDOn(int ledPin) {
  // Turn off all LEDs
  PORTE &= ~(1 << LED_PINY);  // Turn off YELLOW LED on pin 3 (PORTE)
  PORTC &= ~(1 << LED_PING);  // Turn off GREEN LED on pin 4 (PORTC)
  PORTC &= ~(1 << LED_PINB);  // Turn off BLUE LED on pin 5 (PORTC)
  PORTH &= ~(1 << LED_PINR);  // Turn off RED LED on pin 2 (PORTH)

  // Turn on the selected LED
  switch (ledPin) {
    case 0:
      PORTH |= (1 << LED_PINR);  // Turn on RED LED on pin 2 (PORTH)
      break;
    case 1:
      PORTE |= (1 << LED_PINY);  // Turn on YELLOW LED on pin 3 (PORTE)
      break;
    case 2:
      PORTC |= (1 << LED_PING);  // Turn on GREEN LED on pin 4 (PORTC)
      break;
    case 3:
      PORTC |= (1 << LED_PINB);  // Turn on BLUE LED on pin 5 (PORTC)
      break;
  }
}


// Start analog read
void adc_init() {
  ADCSRA = 0x80; // Enable ADC
  ADCSRB = 0x00; // ADC in free running mode
  ADMUX = 0x40;  // Reference voltage set to AVCC
}

unsigned int adc_read(unsigned char adc_channel) {
  if (adc_channel > 7) {
    return 0; // Return 0 for invalid channels
  }
  ADCSRB &= 0xF7;                  // Reset MUX5
  ADCSRB |= (adc_channel & 0x08);  // Set MUX5 if required
  ADMUX &= 0xF8;                   // Reset MUX2:0
  ADMUX |= (adc_channel & 0x07);   // Set MUX2:0 for channel selection

  ADCSRA |= 0x40;                  // Start the conversion
  while (ADCSRA & 0x40) {}         // Wait for conversion to complete
  return ADC;                      // Return converted value
}
//end analog read

// start serial print
// UART initialization
void U0init(unsigned long U0baud) {
  unsigned int ubrr_value = F_CPU / 16 / U0baud - 1;
  
  // Set baud rate
  UBRR0H = (unsigned char)(ubrr_value >> 8);
  UBRR0L = (unsigned char)ubrr_value;
  
  // Enable receiver and transmitter
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  
  // Set frame format: 8 data, 1 stop bit
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Check if data is available to read
unsigned char U0kbhit() {
  return (UCSR0A & (1 << RXC0));
}

// Read a character from UART
unsigned char U0getchar() {
  while (!(UCSR0A & (1 << RXC0)));
  return UDR0;
}

// Send a character via UART
void U0putchar(unsigned char U0pdata) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = U0pdata;
}

void sendTimeToUART() {
    DateTime now = RTC.now();
char buffer[50];
    const char* stateStr = "";

    switch (currentState) {
        case IDLE: stateStr = "IDLE"; break;
        case DISABLED: stateStr = "DISABLED"; break;
        case RUNNING: stateStr = "RUNNING"; break;
        case ERROR: stateStr = "ERROR"; break;
    }

    snprintf(buffer, sizeof(buffer), "%04d/%02d/%02d %02d:%02d:%02d State: %s\n", 
             now.year(), now.month(), now.day(), 
             now.hour(), now.minute(), now.second(),
             stateStr);
    sendStringToUART(buffer);
}

void sendStringToUART(const char* str) {
    while (*str) {
        U0putchar(*str++);
    }
}
//end serial print

void setFanMotor(bool on) {
  if (on) {
    PORTH |= (1 << 3);  // Turn on the fan (pin 6, PH3)
  } else {
    PORTH &= ~(1 << 3);  // Turn off the fan (pin 6, PH3)
  }
}

