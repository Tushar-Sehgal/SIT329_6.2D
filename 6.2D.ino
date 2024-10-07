#include "ADC_DEAKIN.h"
#include "DHT.h"
#include "GPIO_DEAKIN.h"
#include "TIMER_DEAKIN.h"
#include <Arduino.h>

// Define constants
#define LED1_PIN 5 // LED1 at D5
#define LED2_PIN 6 // LED2 at D6
#define DHT_PIN 4  // DHT sensor pin (D4)

// Initialize components
ADC_DEAKIN adc;
DHT dht(DHT_PIN, DHT22); // Initialize DHT22 sensor
GPIO_DEAKIN gpio;
TIMER_DEAKIN timer;

// Constants for RN3440 thermistor
const float R_known = 10000.0; // 10kΩ resistor
const float V_in = 3.3; // Supply voltage (3.3V for Arduino Nano 33 IoT)
const float BETA = 3950; // Typical β-value for thermistors (adjust according to datasheet)
const float R_0 = 10000.0; // Resistance at 25°C (10kΩ)
const float T_0 = 298.15; // Reference temperature (25°C in Kelvin)

// Number of samples for averaging
const int numSamples = 10;

// Variables for averaging
float temperatureSamples[numSamples];
float humiditySamples[numSamples];
int sampleIndex = 0; // Index for current sample

// Variables for average readings
float averageTemperature = 0; // Average temperature
float averageHumidity = 0; // Average humidity

// Timing variables
unsigned long lastReadTime = 0;
unsigned long samplingInterval = 60000; // Default 1 minute
unsigned long alarmTimestamp = 0; // Timestamp for when alarms are triggered
int frequency = 6; // Default frequency

// Function to calculate thermistor resistance
float calculateThermistorResistance(int adcValue) {
    float V_out = (adcValue * V_in) / 1023.0; // Convert ADC value to voltage
    if (V_out == 0) return -1; // Prevent division by zero
    return R_known * ((V_in / V_out) - 1); // Calculate thermistor resistance
}

// Function to calculate temperature using β-model in Celsius
float calculateTemperatureCelsius(float resistance) {
    if (resistance <= 0) return NAN; // Invalid resistance values
    float tempKelvin = 1.0 / ((1.0 / T_0) + (1.0 / BETA) * log(resistance / R_0));
    return tempKelvin - 273.15; // Convert from Kelvin to Celsius
}

// Function to read humidity
void readHumidity() {
    float humidity = dht.readHumidity(); // Read humidity
    if (isnan(humidity)) {
        Serial.println("Failed to read humidity from DHT sensor!");
        return;
    }
    humiditySamples[sampleIndex] = humidity; // Store humidity sample
}

// Function to read temperature
void readTemperature() {
    int adcValue;
    adc.read_ADC(&adcValue); // Read ADC value
    float resistance = calculateThermistorResistance(adcValue); // Calculate resistance
    float temperature = calculateTemperatureCelsius(resistance); // Calculate temperature

    // Adjust temperature readings
    if (temperature < 0) {
        temperature += 22; // Increase negative temperature readings by 22°C
    } else if (temperature > 0) {
        temperature -= 18; // Slightly decrease positive temperature readings
    }

    if (!isnan(temperature)) {
        temperatureSamples[sampleIndex] = temperature; // Store temperature sample
    } else {
        temperatureSamples[sampleIndex] = 0; // Default value if invalid
    }
}

// Function to calculate averages
void calculateAverages() {
    float totalTemperature = 0;
    float totalHumidity = 0;
    int validTemperatureCount = 0;

    for (int i = 0; i < numSamples; i++) {
        if (temperatureSamples[i] >= 0) {
            totalTemperature += temperatureSamples[i];
            validTemperatureCount++;
        }
        totalHumidity += humiditySamples[i];
    }

    // Set averages
    if (validTemperatureCount > 0) {
        averageTemperature = totalTemperature / validTemperatureCount;
    } else {
        averageTemperature = 0; // Default if no valid readings
    }
    averageHumidity = totalHumidity / numSamples;

    // Debug outputs
    Serial.print("Average Temperature: ");
    Serial.println(averageTemperature);
    Serial.print("Average Humidity: ");
    Serial.println(averageHumidity);
}

// Function to blink LED
void blinkLED(uint8_t pin, int blinkFrequency) {
    unsigned long interval = 1000 / blinkFrequency; // Calculate interval based on frequency
    gpio.digitalWrite(pin, HIGH); // Turn LED on
    timer.start(interval); // Start timer for LED on duration
    while (!timer.isComplete()); // Wait until timer is complete
    gpio.digitalWrite(pin, LOW); // Turn LED off
    timer.start(interval); // Start timer for LED off duration
    while (!timer.isComplete()); // Wait until timer is complete
}

// Function to check alarms
void checkAlarms() {
    // Temperature alarm
    if (averageTemperature < 4) {
        Serial.println("ALARM: Average temperature below 4°C - LED1 blinking at 10Hz");
        alarmTimestamp = millis(); // Store the timestamp for alarm
        for (int i = 0; i < 15; i++) { // Blink LED1 for 15 times
            blinkLED(LED1_PIN, 10);
        }
    } else if (averageTemperature < 10) {
        Serial.println("ALARM: Average temperature below 10°C - LED1 blinking at 1Hz");
        alarmTimestamp = millis(); // Store the timestamp for alarm
        for (int i = 0; i < 15; i++) { // Blink LED1 for 15 times
            blinkLED(LED1_PIN, 1);
        }
    }

    // Humidity alarm
    if (averageHumidity < 30) {
        Serial.println("ALARM: Average humidity below 30% - LED2 ON");
        alarmTimestamp = millis(); // Store the timestamp for alarm
        gpio.digitalWrite(LED2_PIN, HIGH);  // Turn LED2 on continuously
        timer.start(15000); // Keep LED2 on for 15 seconds
        while (!timer.isComplete()); // Wait for timer to complete
    } else if (averageHumidity < 50) {
        Serial.println("ALARM: Average humidity below 50% - LED2 blinking at 0.5Hz");
        alarmTimestamp = millis(); // Store the timestamp for alarm
        for (int i = 0; i < 15; i++) { // Blink LED2 for 15 times
            blinkLED(LED2_PIN, 2); // Blink at 0.5Hz (1 second on, 1 second off)
        }
    }
}

// Function to handle the serial menu
void handleMenu() {
    Serial.println("Menu:");
    Serial.println("a - Display current sensor readings with timestamps");
    Serial.println("b - Display current alarm states with timestamps");
    Serial.println("Or enter a number between 1 and 30 to set the sampling frequency");
    Serial.print("Select an option: ");

    while (!Serial.available()); // Wait for user input

    String input = Serial.readStringUntil('\n'); // Read the user input until newline
    input.trim(); // Remove any leading/trailing whitespace

    if (input.equalsIgnoreCase("a")) {
        // Option 'a' selected
        Serial.print("\nCurrent Temperature: ");
        Serial.print(temperatureSamples[sampleIndex]); // Latest temperature
        Serial.print(" at time: ");
        Serial.println(millis()); // Current time in milliseconds

        Serial.print("\nCurrent Humidity: ");
        Serial.print(humiditySamples[sampleIndex]); // Latest humidity
        Serial.print(" at time: ");
        Serial.println(millis()); // Current time in milliseconds
    } else if (input.equalsIgnoreCase("b")) {
        // Option 'b' selected
        Serial.println("\nCurrent Alarm States:");
        Serial.print("Temperature alarm at time: ");
        Serial.println(alarmTimestamp); // Last alarm timestamp
        Serial.print("Humidity alarm at time: ");
        Serial.println(alarmTimestamp); // Last alarm timestamp
    } else {
        // Check if input is a number between 1 and 30
        int freq = input.toInt();
        if (freq >= 1 && freq <= 30) {
            frequency = freq;
            samplingInterval = frequency * 1000; // Convert to milliseconds
            Serial.print("\nSampling frequency set to ");
            Serial.print(frequency);
            Serial.println(" seconds.");
        } else {
            Serial.println("\nInvalid input. Please enter 'a', 'b', or a number between 1 and 30.");
        }
    }
}

void setup() {
    Serial.begin(9600);
    while (!Serial); // Wait for serial to be ready
    dht.begin(); // Initialize DHT sensor
    adc.setup_ADC('A', 3, 10, 0); // Using A3 pin, single-shot mode
    adc.enable_ADC(); // Enable the ADC

    // Initialize LED pins
    gpio.pinMode(LED1_PIN, OUTPUT);
    gpio.pinMode(LED2_PIN, OUTPUT);

    timer.init(); // Initialize the timer
    Serial.println("System Initialized.");
}

void loop() {
    // Display the menu before taking readings
    handleMenu();

    unsigned long currentTime = millis();
    if (lastReadTime == 0 || currentTime - lastReadTime >= samplingInterval) {
        lastReadTime = currentTime;

        // Take readings and calculate averages
        for (int i = 0; i < numSamples; i++) {
            readTemperature(); // Read temperature
            readHumidity(); // Read humidity

            // Increment the sample index and wrap around if needed
            sampleIndex = (sampleIndex + 1) % numSamples;

            // Use frequency as the timer count
            for (int j = 0; j < frequency; j++) {
                timer.start(1000); // Wait for 1 second
                while (!timer.isComplete()); // Wait until timer is complete
            }
        }

        // Calculate averages after filling the array
        calculateAverages();
        checkAlarms(); // Check alarms based on average readings
    }
}
