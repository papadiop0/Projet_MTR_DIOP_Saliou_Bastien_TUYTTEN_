//Projet de multi-tâches et temps réel
//DIOP Saliou, TUYTTEN Bastien
//Ce programme a été généré grâce à une intelligence artificielle
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Pin definitions
#define LIGHT_SENSOR_PIN 34
#define MOTION_SENSOR_PIN 13

// I2C addresses for LCDs
const int LCD1_ADDRESS = 0x27;
const int LCD2_ADDRESS = 0x28;
const int LCD3_ADDRESS = 0x29;

#define SEUIL_LUMINOSITE 500

LiquidCrystal_I2C lcd1(LCD1_ADDRESS, 16, 2);
LiquidCrystal_I2C lcd2(LCD2_ADDRESS, 16, 2);
LiquidCrystal_I2C lcd3(LCD3_ADDRESS, 16, 2);

// States
enum TVState { OFF, ON, URGENT };

// Global variables
TVState tv1State = OFF;
TVState tv2State = OFF;
TVState tv3State = OFF;
bool isDay = true;
SemaphoreHandle_t xSemaphore;

QueueHandle_t luminosityEventQueue;
QueueHandle_t motionEventQueue1;

TimerHandle_t screenOffTimer1;
TimerHandle_t screenOffTimer2;
TimerHandle_t screenOffTimer3;

void Task_LightSensor(void *pvParameters) {
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  int luminosity;
  while (1) {
    int lightValue = analogRead(LIGHT_SENSOR_PIN);
    
    // Calcul de la luminosité
    const float GAMMA = 0.7;
    const float RL10 = 50;
    float voltage = lightValue / 1024.0 * 5;
    float resistance = 2000 * voltage / (1 - voltage / 5);
    luminosity = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA));

    Serial.print("Light Sensor Value: ");
    Serial.println(luminosity);
    
    if (luminosity > SEUIL_LUMINOSITE) {
      isDay = true;
      xQueueSend(luminosityEventQueue, &luminosity, portMAX_DELAY);
    } else {
      isDay = false;
      // Éteindre les écrans la nuit
      lcd1.clear();
      lcd1.noBacklight();
      tv1State = OFF;
      
      lcd2.clear();
      lcd2.noBacklight();
      tv2State = OFF;

      lcd3.clear();
      lcd3.noBacklight();
      tv3State = OFF;
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Vérifie la luminosité chaque seconde
  }
}

void Task_TVControl(void *pvParameters) {
  lcd1.init();
  lcd1.backlight();
  lcd1.clear();
  
  lcd2.init();
  lcd2.backlight();
  lcd2.clear();
  
  lcd3.init();
  lcd3.backlight();
  lcd3.clear();

  int luminosity;
  while (1) {
    if (xQueueReceive(luminosityEventQueue, &luminosity, portMAX_DELAY) == pdPASS) {
      if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
        if (tv1State == OFF && tv2State == OFF && tv3State == OFF) {
          if (isDay) {
            lcd1.setCursor(0, 0);
            lcd1.print("TV1 ON");
            tv1State = ON;
          }
        } else if (tv2State == OFF && tv3State == OFF) {
          if (isDay) {
            lcd2.setCursor(0, 0);
            lcd2.print("TV2 ON");
            tv2State = ON;
          }
        } else if (tv3State == OFF && tv1State == OFF) {
          if (isDay) {
            lcd3.setCursor(0, 0);
            lcd3.print("TV3 ON");
            tv3State = ON;
          }
        }
        xSemaphoreGive(xSemaphore);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Vérifie l'état toutes les secondes
  }
}


void motionSensorISR() {
  int danger = 1; // Indicateur de danger
  xQueueSendFromISR(motionEventQueue1, &danger, NULL);
}

void Task_MotionHandler1(void *pvParameters) {
  int danger;
  while (1) {
    if (xQueueReceive(motionEventQueue1, &danger, portMAX_DELAY) == pdPASS) {
      if (!isDay && tv2State == ON && tv3State == ON) { // Si c'est la nuit et les TV2 et TV3 sont allumées
        xSemaphoreTake(xSemaphore, portMAX_DELAY); // Prendre le sémaphore pour allumer TV1
        lcd1.clear();
        lcd1.setCursor(0, 0);
        lcd1.print("TV1 ON (URGENT)");
        tv1State = URGENT;
        xTimerStart(screenOffTimer1, 0);  // Start a short timer to turn off TV1 after 5 seconds
        xSemaphoreGive(xSemaphore);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void turnOffScreen1(TimerHandle_t xTimer) {
  lcd1.clear();
  lcd1.noBacklight();
  tv1State = OFF;
}

void turnOffScreen2(TimerHandle_t xTimer) {
  lcd2.clear();
  lcd2.noBacklight();
  tv2State = OFF;
}

void turnOffScreen3(TimerHandle_t xTimer) {
  lcd3.clear();
  lcd3.noBacklight();
  tv3State = OFF;
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTION_SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), motionSensorISR, RISING);

  xSemaphore = xSemaphoreCreateCounting(1, 1); // Sémaphore avec 1 ressource disponible

  luminosityEventQueue = xQueueCreate(5, sizeof(int));
  motionEventQueue1 = xQueueCreate(5, sizeof(int));

  screenOffTimer1 = xTimerCreate("Timer Ecran 1", pdMS_TO_TICKS(5000), pdFALSE, 0, turnOffScreen1);
  screenOffTimer2 = xTimerCreate("Timer Ecran 2", pdMS_TO_TICKS(7200000), pdFALSE, 0, turnOffScreen2); // 2 heures
  screenOffTimer3 = xTimerCreate("Timer Ecran 3", pdMS_TO_TICKS(7200000), pdFALSE, 0, turnOffScreen3); // 2 heures

  xTaskCreate(Task_LightSensor, "Light Sensor Task", 2048, NULL, 2, NULL);
  xTaskCreate(Task_TVControl, "TV Control Task", 2048, NULL, 1, NULL);
  xTaskCreate(Task_MotionHandler1, "Motion Handler 1 Task", 2048, NULL, 3, NULL);
}

void loop() {
  // Le code FreeRTOS fonctionne indépendamment dans les tâches définies.
}
