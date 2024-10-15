
#include "BluetoothSerial.h"
#include <Wire.h>
#include "DHT.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Ticker.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

/* Using core 1 of ESP32 */
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif
static bool flag;
/* Sensor pins */
#define DHTPIN 33                 // DHT11 temperature sensor
#define DHTTYPE DHT11
#define lightSensor 26            // LDR sensor
#define smokeSensor 25            // MQ2 smoke and gas sensor
#define touchSensor 4             // Touch sensor (GPIO 4)
#define echo 2                    // Ultrasonic sensor echo pin
#define trigger 15                // Ultrasonic sensor trigger pin
         
/* Relay pins */          
#define fanRelay 17               // Relay for fan
#define lightRelay 16             // Relay for light

/* Buzzer pins */
#define smokeBuzzer 14            // Buzzer for alerting smoke or gas
#define touchBuzzer 27            // Buzzer for alerting touch

/* Led pins */
#define smokeLed 5                // Led for alerting smoke
#define touchLed 19               // Led for alerting touch
#define ultrasonicLed 18          // Led for alerting when someone in range


/* Defining objects */
DHT dht(DHTPIN, DHTTYPE);
BluetoothSerial SerialBT;                                 
Ticker ultrasonic;

/* Defining queues */
static QueueHandle_t tempReading;
static QueueHandle_t lightReading;
static QueueHandle_t smokeAlarm;
static QueueHandle_t touchAlarm;

/* Defining task handles */
TaskHandle_t autoFan_handle = NULL;
TaskHandle_t autoLight_handle = NULL;

bool fanStatus = false;
bool lightStatus = false;
bool smokeStatus = false;
bool touchStatus = false;
bool ultrasonicStatus = false;

/*
* ---------------------------------------------------------------------------------------------------------------------------------
* Setup  
* ---------------------------------------------------------------------------------------------------------------------------------
*/

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  Serial.begin(115200);                                     // Serial baud rate
  Wire.begin();
  SerialBT.begin("ESP32");
  Serial.println("The device started, now you can pair it with bluetooth!");
  dht.begin();
  ultrasonic.attach(1, ultrasonicDetect);
  
  /* Defining pin modes */  
  pinMode(fanRelay, OUTPUT);
  pinMode(lightRelay, OUTPUT);
  pinMode(smokeLed, OUTPUT);
  pinMode(touchLed, OUTPUT);
  pinMode(ultrasonicLed, OUTPUT);
  pinMode(smokeBuzzer, OUTPUT);
  pinMode(touchBuzzer, OUTPUT);
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(smokeSensor,INPUT);
  pinMode(lightSensor,INPUT);
  pinMode(touchSensor,INPUT);



  /* Relays off at start */
  // digitalWrite(fanRelay, HIGH);                              
  // digitalWrite(lightRelay, HIGH);                            
  digitalWrite(fanRelay, LOW);                              
  digitalWrite(lightRelay, LOW);


  /* Buzzers off at start */
  digitalWrite(touchBuzzer, LOW);                            
  digitalWrite(smokeBuzzer, LOW);                            

  /* Leds off at start */
  digitalWrite(smokeLed, LOW);                           
  digitalWrite(touchLed, LOW);                                   
  digitalWrite(ultrasonicLed, LOW);                                


  /* Creating queues */
  tempReading = xQueueCreate(10, sizeof(int));
  lightReading = xQueueCreate(10, sizeof(int));
  
  /* Creating tasks */
  xTaskCreatePinnedToCore (tempRead, "Temp read", 2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore (autoFan, "Auto fan", 2048, NULL, 1, &autoFan_handle, app_cpu); 
  xTaskCreatePinnedToCore (lightRead, "Light read", 1024, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore (autoLight, "Auto light", 1024, NULL, 1, &autoLight_handle, app_cpu); 
  xTaskCreatePinnedToCore (smokeDetect, "Smoke detect", 1024, NULL, 1, NULL, app_cpu);   
  xTaskCreatePinnedToCore (touchDetect, "Touch read", 1024, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore (switchControl, "Switch control", 4096, NULL, 1, NULL, app_cpu);

    
/* Suspending auto mode tasks at start */
  vTaskSuspend (autoFan_handle);
  vTaskSuspend (autoLight_handle);
}

void loop() {
  vTaskDelay(500 / portTICK_PERIOD_MS);
}

/*
* ---------------------------------------------------------------------------------------------------------------------------------
* Temperature monitoring and fan control 
* ---------------------------------------------------------------------------------------------------------------------------------
*/

/* Task for temperature sensing using DHT11 */
void tempRead(void *parameter) {
 int t = 0;
  
  while (true) {
    t = dht.readTemperature();
    
    if (isnan(t)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }
    
    /* Send temperature values via bluetooth */
    SerialBT.print("#");
    SerialBT.print(t);
    SerialBT.print("?");
    
    /* Print temperature and humidity values on serial monitor */
    Serial.print("Temperature: "); 
    Serial.print(t); 
    Serial.println(" °C"); 
    
    xQueueSend (tempReading, (void*)&t, 8);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

/* Task for fan control in auto mode */
void autoFan(void *parameter) {
  int t ;
  while (true) { 
    xQueueReceive(tempReading, (void *)&t, portMAX_DELAY);    
    
    if (t >= 40) {
      SerialBT.print ("Fan on?"); 
      digitalWrite(fanRelay,HIGH); ;
      fanStatus = true;
    }
    else if (t < 40) {
      SerialBT.print ("Fan off?");
      digitalWrite(fanRelay,LOW);
      fanStatus = false; 
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

/*
* ---------------------------------------------------------------------------------------------------------------------------------
* LDR based bulb control 
* ---------------------------------------------------------------------------------------------------------------------------------
*/

/* Task for light intensity sensing using LDR */
void lightRead(void *parameter) {
  int lightValue;
  
  while (true) {
    lightValue = analogRead(lightSensor); 
    Serial.print("Light intensity: ");
    Serial.println(lightValue);

    xQueueSend (lightReading, (void*)&lightValue, 10);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  } 
}

/* Task for bulb control in auto mode */
void autoLight(void *parameter) {

  int lightValue;
  
  while (true) { 
    xQueueReceive(lightReading, (void *)&lightValue, portMAX_DELAY);    
    
    if (lightValue >= 3800) {
      SerialBT.print("Bulb on?");
      digitalWrite(lightRelay,HIGH);
      lightStatus = true;
    }
    else if (lightValue < 3800) {
      SerialBT.print("Bulb off?");
      digitalWrite(lightRelay,LOW); 
      lightStatus = false;
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  } 
}

/*
* ---------------------------------------------------------------------------------------------------------------------------------
* Safety and Security system
* ---------------------------------------------------------------------------------------------------------------------------------
*/

/* Task for detecting smoke or gas using MQ2 sensor */
void smokeDetect(void *parameter) {
  int smokeValue;
  
  while (true) {
    smokeValue = analogRead(smokeSensor); 
    Serial.print("Smoke: ");
    Serial.println(smokeValue);
    
    if (smokeValue >= 3500) {
      SerialBT.print("Smoke active?");
      digitalWrite(smokeLed, HIGH);
      digitalWrite(smokeBuzzer, HIGH);
      smokeStatus = true;      
    }
    else if (smokeValue < 3500) {
      SerialBT.print("Smoke inactive?");
      digitalWrite(smokeLed, LOW);
      digitalWrite(smokeBuzzer, LOW);
      smokeStatus = false; 
    }
    
    vTaskDelay(500 / portTICK_PERIOD_MS);      
  }
}

/* Task for detecting touch using inbuilt touch sensor */
void touchDetect(void *parameter) {
  int touchValue;

  while (true) {
    
    touchValue = (touchRead(touchSensor));  
   Serial.print(" touchValue: "); // In ra màn hình dòng chữ "Sensor value: "
  Serial.println(touchValue); // In giá trị của cảm biến, xuống dòng sau đó
    if (touchValue <20  && touchValue >1 ) {
      SerialBT.print("Touch active?");
      digitalWrite(touchLed, HIGH);
      digitalWrite(touchBuzzer, HIGH);
      touchStatus = true; 
    }
    vTaskDelay(500 / portTICK_PERIOD_MS); 
  }
}

/* Task for finding distance using Ultrasonic sensor */
void ultrasonicDetect() { 
  int distance;
  int duration;

  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  duration = pulseIn(echo, HIGH);
  distance = (duration / 2) * 0.0343;

  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance > 20) {
    SerialBT.print("Ultrasonic inactive?");
    digitalWrite(ultrasonicLed, LOW);
    ultrasonicStatus = false;
  }
  else if (distance <= 20) {
    SerialBT.print("Ultrasonic active?");
    digitalWrite(ultrasonicLed, HIGH);
    ultrasonicStatus = true;
  }
}

/*
* ---------------------------------------------------------------------------------------------------------------------------------
* App based switch controls
* ---------------------------------------------------------------------------------------------------------------------------------
*/

/* Task for controlling relays and alarms using app */
void switchControl(void *parameter) {
  char input;
  
  while (true) {
    if (SerialBT.available() > 0) {
      input = SerialBT.read();
      switch (input) {
        /* Select manual mode */
        case 'M': {
          vTaskSuspend (autoFan_handle);
          vTaskSuspend (autoLight_handle);
          break;
        }
        /* Switch on fan */
        case 'F': {
          SerialBT.print("Fan on?");
          digitalWrite(fanRelay, HIGH);
          fanStatus = true;
          break;
        }
        /* Switch off fan */
        case 'Y': {
          SerialBT.print("Fan off?");
          digitalWrite(fanRelay, LOW);  
          fanStatus = false;       
          break;
        }
        /* Switch on light */
        case 'L': {
          SerialBT.print("Bulb on?");
          digitalWrite(lightRelay, HIGH);
          lightStatus = true;
          break;
        }
        /* Switch off light  */
        case 'Z': {
          SerialBT.print("Bulb off?");
          digitalWrite(lightRelay, LOW);
          lightStatus = false;
          break;
        }
        /* Select automatic mode */
        case 'A': {
          vTaskResume(autoFan_handle);
          vTaskResume(autoLight_handle);
          break;
        }
        /* Select off mode */
        case 'O': {
          vTaskSuspend(autoFan_handle);
          vTaskSuspend(autoLight_handle);
          SerialBT.print("Fan off?");
          SerialBT.print("Bulb off?");
          digitalWrite(fanRelay, LOW);
          digitalWrite(lightRelay, LOW);
          fanStatus = false;
          lightStatus = false;
          break;
        }
        /* Turn off touch alarm */
        case 'T': {
          SerialBT.print("Touch inactive?");
          digitalWrite(touchBuzzer, LOW);
          digitalWrite(touchLed, LOW);
          touchStatus = false; 
          break;
        }
      }
    }   
  }
}
