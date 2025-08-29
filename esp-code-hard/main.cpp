#include <Arduino.h>
#include <Keypad.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <WiFiClientSecure.h> 
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ====== WiFi & MQTT Config ======
const char* ssid ="Vodafone_VDSL_F093";
const char* password = "F4RFABTX3D76Q";

const char* mqtt_server = "5e69119781b74a90b6a9443c6e278c19.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "yasmina";
const char* mqtt_password = "Yasmin22452245";

const char* mqtt_topic_publish = "yasmin/sensors";
const char* mqtt_topic_subscribe = "game/control";

WiFiClientSecure espClient;
PubSubClient client(espClient);




// ================== Servos ==================
Servo gateServo;
Servo flameServo;
#define SERVO_PIN 4
#define SERVO_OPEN_ANGLE 180
#define SERVO_CLOSED_ANGLE 0
#define FLAME_SERVO_PIN 13

// ================== Buzzer ==================
#define BUZZER_PIN 18
const int BUZZ_CH = 0;
void buzz(int freq) { if(freq<=0) ledcWriteTone(BUZZ_CH,0); else ledcWriteTone(BUZZ_CH,freq);}
void buzzFor(int freq,int ms){buzz(freq); delay(ms); buzz(0);}

// ================== Keypad ==================
#define LDR_PIN 33
#define LDR_THRESHOLD 119
const char* CORRECT_CODE = "1234";
const uint8_t CODE_LEN = 4;
const uint8_t WRONG_LIMIT = 3;
const unsigned long LOCK_MS = 10000;
const unsigned long IDLE_TIMEOUT_MS = 15000;

const byte ROWS=3, COLS=3;
char keys[ROWS][COLS]={{'1','2','3'},{'4','5','6'},{'7','8','9'}};
byte rowPins[ROWS]={13,12,14};
byte colPins[COLS]={25,26,27};
Keypad keypad=Keypad(makeKeymap(keys),rowPins,colPins,ROWS,COLS);

uint8_t pos=0, wrongCountKeypad=0;
bool gateOpened=false, locked=false;
unsigned long lockUntil=0, lastKeyMillis=0;

// ================== Level Control ==================
int currentLevel = 0; // 0=Keypad,1=LDR,2=Gas,3=IR Game,4=Flame

// ================== Level 1: LDR ==================
int ldrStep=-1;
bool covered=false;

// ================== Level 2: Gas Puzzle ==================
const int gasSensor=35, potPin=34, buttonPin=19;
const int redLED=15, yellowLED=2, greenLED=23;
int potValue=0,target=2000,tolerance=500;
int baseline=0,thresholdDelta=20;
bool gasDetected=false,puzzleSolved=false;
unsigned long startTime=0;
const unsigned long timeLimit=60000;

// ================== Level 3: IR Game ==================
const int irPin = 32;
bool isGreen=false;
unsigned long lastChange=0;
unsigned long interval=3000;
int roundCount=0;
const int maxRounds=10;
bool gameOverIR=false;
unsigned long ignoreUntilIR=0;
const unsigned long minMovementTime=150;
int wrongCountIR=0;
const int maxWrongIR=3;

bool irViolationThisRound = false; // <<<<< ADD THIS LINE


// ================== Level 4: Flame Sensor ==================
#define FLAME_SENSOR_PIN 34
bool flameLevelSolved=false;
char flameCode[5];
char inputCode[5];
byte flamePos=0;
int wrongCountFlame=0;
const int maxWrongFlame=3;


// setup wifi

void setup_wifi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());
  espClient.setInsecure();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to HiveMQ...");
    if (client.connect("ESP32GameClient", mqtt_username, mqtt_password)) {
      Serial.println("Connected!");
      client.subscribe(mqtt_topic_subscribe);
    } else {
      Serial.print("Failed, state=");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("📩 Command Received: ");
  Serial.println(message);

  String command = "";

  // حاول الأول تعمل Parse للـ JSON
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);
  if (!error) {
    command = String((const char*)doc["command"]);
  } else {
    // لو JSON فشل، اعتبر الرسالة String عادي
    command = message;
  }

  // دلوقتي command فيه اسم الأمر جاهز
  if (command == "reset") {
    currentLevel = 0;
    gateOpened = false;
    flameLevelSolved = false;
    wrongCountKeypad = wrongCountFlame = 0;
    pos = 0;
    Serial.println("🔄 Game Reset!");
  }
  else if (command == "skip") {
    currentLevel++;
    Serial.println("⏭ Skipping to next level...");
  }
  else if (command == "led_on") {
    digitalWrite(redLED, HIGH);
    Serial.println("💡 LED ON");
  }
  else if (command == "led_off") {
    digitalWrite(redLED, LOW);
    Serial.println("💡 LED OFF");
  }
  else if (command == "servo_open") {
    gateServo.write(SERVO_OPEN_ANGLE);
    Serial.println("🔓 Servo Opened");
  }
  else if (command == "servo_close") {
    gateServo.write(SERVO_CLOSED_ANGLE);
    Serial.println("🔒 Servo Closed");
  }
}

void publishSensorData(
    String stage, 
    String event, 
    float value = 0.0, 
    int attempts = -1, 
    int time = -1, 
    String status = "", 
    String code = ""     // 🆕 الكود المدخل
) {
    StaticJsonDocument<300> doc;

    doc["stage"] = stage;
    doc["event"] = event;
    if (value != 0.0) doc["value"] = value;
    if (attempts != -1) doc["attempts"] = attempts;
    if (time != -1) doc["time"] = time;
    if (status != "") doc["status"] = status;
    if (code != "") doc["code"] = code;  // 🆕 إضافة الباسورد في الرسالة

    char buffer[300];
    serializeJson(doc, buffer);
    client.publish(mqtt_topic_publish, buffer);
    Serial.println("Data sent: " + String(buffer));
}


// ================== Helpers ==================
void showProgress(){
    Serial.print("Code: ");
    for(uint8_t i=0;i<CODE_LEN;i++){
        if(i<pos) Serial.print('*'); else Serial.print('_');
        Serial.print(' ');
    }
    Serial.println();
}

void showLockedCountdown(){
    long msLeft=(long)(lockUntil-millis());
    int secs=msLeft>0?(msLeft+999)/1000:0;
    Serial.print("Too many errors. Locked for ");
    Serial.print(secs);
    Serial.println("s");
}

void openGate() {
    Serial.println("✅ Access Granted. Opening gate...");
    gateServo.write(SERVO_OPEN_ANGLE);  // افتح البوابة
    gateOpened = true;
    // متقفلش هنا، خليه يفضل مفتوح لحد المرحلة الجاية
}
// ================== IR Game Helpers ==================
void changeLightIR(){
    isGreen = !isGreen; 
    lastChange = millis();
    ignoreUntilIR = millis() + 400; 

    if(isGreen){
        digitalWrite(greenLED,HIGH); digitalWrite(redLED,LOW);
        Serial.println("🟢 Green Light: You can move.");
        interval=random(3000,6000);
    } else {
        digitalWrite(greenLED,LOW); digitalWrite(redLED,HIGH);
        Serial.println("🔴 Red Light: Don't move.");
        interval=random(3000,6000);
    }
}

void playerFailedIR(){
    wrongCountIR++;
    digitalWrite(redLED,HIGH); digitalWrite(greenLED,LOW);
    buzzFor(1000,500);

    Serial.print("🚨 Player triggered IR! Strike #");
    Serial.println(wrongCountIR);

    if(wrongCountIR >= maxWrongIR + 1){
        Serial.println("💀 Game Over!");
        gameOverIR = true;
        digitalWrite(greenLED,LOW); digitalWrite(redLED,HIGH);
        buzzFor(400,1500);
    }
}

// ================== Flame Level Helpers ==================
void generateFlameCode() {
  for(int i=0;i<4;i++) flameCode[i] = random(0,10)+'0';
  flameCode[4]='\0';
  Serial.print("Flame Code: ");
  Serial.println(flameCode);
}

void unlockFlameServo(){
  flameServo.write(80);
  delay(4000);
  flameServo.write(0);
}

// ================== Setup ==================
void setup() {
    Serial.begin(115200);

    // ====== WiFi Connect ======
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());

    // ====== MQTT Connect ======
    espClient.setInsecure();  // عشان TLS بدون شهادات
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        if (client.connect("ESP32GameClient", mqtt_username, mqtt_password)) {
            Serial.println("Connected to HiveMQ!");
            client.subscribe(mqtt_topic_subscribe);
        } else {
            Serial.print("Failed, state=");
            Serial.println(client.state());
            delay(5000);
        }
    }

    // ====== Game Setup ======
    gateServo.attach(SERVO_PIN); 
    gateServo.write(SERVO_CLOSED_ANGLE);
    
    flameServo.attach(FLAME_SERVO_PIN); 
    flameServo.write(0);

    pinMode(BUZZER_PIN, OUTPUT); 
    ledcAttachPin(BUZZER_PIN, BUZZ_CH);

    pinMode(redLED, OUTPUT); 
    pinMode(yellowLED, OUTPUT); 
    pinMode(greenLED, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);

    analogReadResolution(12); 
    analogSetPinAttenuation(potPin, ADC_11db); 
    analogSetPinAttenuation(gasSensor, ADC_11db);

    // ====== MQ2 warmup ======
    Serial.println("Calibrating MQ2 sensor...");
    unsigned long t0 = millis();
    while (millis() - t0 < 20000) { 
        Serial.print("Warming: "); 
        Serial.print((20000 - (millis() - t0)) / 1000); 
        Serial.println("s"); 
        delay(200);
    }
    long sum = 0; 
    for (int i = 0; i < 200; i++) { 
        sum += analogRead(gasSensor); 
        delay(5);
    }
    baseline = sum / 200;

    // generateFlameCode();  // لو عندك الكود ده استدعيه هنا
    Serial.println("Level 0: Keypad - Enter Code");
    showProgress(); 
    lastKeyMillis = millis();
}

void loop() {
    client.loop();  // Maintain MQTT connection

    // ---------- LEVEL 0: Keypad ----------
    if (currentLevel == 0 && !gateOpened) {
        static String enteredCode = "";

        if (locked) {
            if ((long)(millis() - lockUntil) < 0) {
                static unsigned long lastUpdate = 0;
                if (millis() - lastUpdate > 200) {
                    Serial.println("🔒 Locked! Countdown...");
                    lastUpdate = millis();
                }
                return;
            } else {
                locked = false; wrongCountKeypad = 0; pos = 0; enteredCode = "";
                Serial.println("Progress reset.");
            }
        }

        if (pos > 0 && millis() - lastKeyMillis > IDLE_TIMEOUT_MS) {
            pos = 0; enteredCode = "";
            Serial.println("Idle timeout. Progress reset.");
        }

        char key = keypad.getKey();
        if (!key) return;
        lastKeyMillis = millis();
        if (key < '1' || key > '9') return;

        enteredCode += key;

        if (key == CORRECT_CODE[pos]) {
            pos++;
            Serial.print("Digit correct! Position: "); Serial.println(pos);
            if (pos == CODE_LEN) {
                openGate(); currentLevel = 1;
                Serial.println("Level 1: LDR - Cover sensor!");
                enteredCode = "";
                delay(1500);
            }
        } else {
            wrongCountKeypad++;
            Serial.print("Wrong digit (#"); Serial.print(wrongCountKeypad); Serial.println("). Try again...");
            if (wrongCountKeypad >= WRONG_LIMIT) {
                locked = true; lockUntil = millis() + LOCK_MS;
                Serial.println("🔒 Too many wrong attempts! Locked.");
                return;
            }
        }
    }

    // ---------- LEVEL 1: LDR ----------
    else if (currentLevel == 1) {
        static unsigned long levelStartTime = 0;
        static int lastHint = -1;
        int ldrValue = analogRead(LDR_PIN);
        const int weakLight = 500;
        const int strongLight = 900;

        if (ldrStep == -1 && ldrValue <= weakLight) {
            Serial.println("🎮 Game Started! Increase the light for Step 1.");
            ldrStep = 0; lastHint = -1;
            levelStartTime = millis();
            delay(300);
        }

        if (ldrStep == 0) {
            if (ldrValue >= strongLight) {
                Serial.println("✅ Step 1 complete! Now make the light weak again.");
                ldrStep = 1; lastHint = -1; delay(300);
            } else if (lastHint != 1) {
                Serial.println("⬆ Increase the light for Step 1.");
                lastHint = 1;
            }
        }
        else if (ldrStep == 1) {
            if (ldrValue <= weakLight) {
                Serial.println("✅ Step 2 complete! You passed Level 1.");
                buzzFor(900, 500);
                openGate();
                currentLevel = 2; ldrStep = -1; lastHint = -1;
                delay(500);
            } else if (lastHint != 2) {
                Serial.println("⬇ Make the light weak for Step 2.");
                lastHint = 2;
            }
        }

        delay(200);
    }

    // ---------- LEVEL 2: Gas Puzzle ----------
    else if (currentLevel == 2 && !puzzleSolved) {
        int gasValue = analogRead(gasSensor);
        potValue = analogRead(potPin);
        int diff = abs(potValue - target);

        if (!gasDetected && abs(gasValue - baseline) > thresholdDelta) {
            gasDetected = true;
            startTime = millis();
            Serial.println("⚠ Gas Leak !!!");
            buzzFor(900, 200);
        }

        if (gasDetected) {
            unsigned long elapsed = millis() - startTime;
            unsigned long remaining = (elapsed >= timeLimit) ? 0 : (timeLimit - elapsed) / 1000;
            Serial.print("Time "); Serial.print(remaining); Serial.print("s | D = "); Serial.println(diff);

            if (diff < tolerance) {
                digitalWrite(greenLED, HIGH); digitalWrite(yellowLED, LOW); digitalWrite(redLED, LOW);
                Serial.println("✅ Within tolerance! Green LED ON");
            } else if (diff < 2 * tolerance) {
                digitalWrite(greenLED, LOW); digitalWrite(yellowLED, HIGH); digitalWrite(redLED, LOW);
                Serial.println("⚠ Close! Yellow LED ON");
            } else {
                digitalWrite(greenLED, LOW); digitalWrite(yellowLED, LOW); digitalWrite(redLED, HIGH);
                Serial.println("❌ Too far! Red LED ON");
            }

            if (digitalRead(buttonPin) == LOW) {
                delay(25); while (digitalRead(buttonPin) == LOW) delay(10);
                if (diff < tolerance) {
                    Serial.println("✅ Correct! Puzzle solved.");
                    buzzFor(1500, 300); puzzleSolved = true;
                    digitalWrite(greenLED, LOW); digitalWrite(redLED, LOW); digitalWrite(yellowLED, LOW);
                    openGate();
                } else {
                    Serial.println("❌ Wrong! Try again");
                    buzzFor(300, 600); digitalWrite(redLED, HIGH);
                    delay(600); digitalWrite(redLED, LOW);
                }
            }

            if (elapsed >= timeLimit) {
                Serial.println("💀 TIME OUT! YOU ARE DEAD");
                buzzFor(200, 1200); buzzFor(180, 800); buzz(0);
                delay(2000);
                Serial.println("➡ Moving to next level...");
                puzzleSolved = true;
            }
        }

        if (puzzleSolved) {
            currentLevel = 3; Serial.println("Level 3: IR Game"); delay(1000);
        }
    }

    // ---------- LEVEL 3: IR RED LIGHT ----------
    else if (currentLevel == 3 && !gameOverIR) {
    unsigned long currentMillis = millis();

    // Change light every interval
    if (currentMillis - lastChange > interval) {
        changeLightIR();
        roundCount++;
        lastChange = currentMillis;
        irViolationThisRound = false; // reset violation flag for new interval

        if (roundCount >= maxRounds) {
            Serial.println("✅ You Passed! Next Puzzle...");
            publishSensorData("level3", "success", 1, roundCount, -1, "success");
            gameOverIR = true;
            currentLevel = 4;
            delay(1000);
        }
    }

    // Check movement only if red light
    if (!isGreen && currentMillis >= ignoreUntilIR && !irViolationThisRound) {
        static unsigned long movementStart = 0;
        static bool countingMovement = false;

        if (digitalRead(irPin) == LOW) {
            if (!countingMovement) {
                movementStart = currentMillis;
                countingMovement = true;
            } else if (currentMillis - movementStart >= minMovementTime) {
                Serial.println("❌ Movement detected in RED zone! Strike logged.");
                publishSensorData("level3", "failed", 0, roundCount, -1, "fail");
                buzzFor(200, 800);

                // Treat the rest of this red zone like green
                irViolationThisRound = true; 
            }
        } else {
            countingMovement = false;
        }
    }
}

    // ---------- LEVEL 4: Flame Sensor ----------
    else if (currentLevel == 4 && !flameLevelSolved) {
        static bool flameDetected = false;
        static unsigned long lastCharMillis = 0;
        static int codeCharIndex = 0;

        if (!flameDetected) {
            if (digitalRead(FLAME_SENSOR_PIN) == HIGH) {
                flameDetected = true;
                codeCharIndex = 0;
                lastCharMillis = millis();
                Serial.println("🔥 Flame detected! Code appearing...");
            } else {
                return;
            }
        }

        if (flameDetected && codeCharIndex < 4 && millis() - lastCharMillis >= 500) {
            Serial.print(flameCode[codeCharIndex]);
            lastCharMillis = millis();
            codeCharIndex++;
            if (codeCharIndex == 4) Serial.println();
        }

        if (codeCharIndex >= 4) {
            char key = keypad.getKey();
            if (!key) return;

            inputCode[flamePos++] = key;
            Serial.print("*");

            if (flamePos == 4) {
                inputCode[4] = '\0';
                if (strcmp(inputCode, flameCode) == 0) {
                    Serial.println("\n✅ Correct! Unlocking...");
                    unlockFlameServo();
                    flameLevelSolved = true;
                    currentLevel++;
                } else {
                    wrongCountFlame++;
                    Serial.print("\n❌ Wrong Code. Strike ");
                    Serial.println(wrongCountFlame);
                    if (wrongCountFlame >= maxWrongFlame) {
                        Serial.println("💀 Game Over!");
                        flameLevelSolved = true;
                    }
                }
                flamePos = 0;
            }
        }

        delay(80);
    }
}
