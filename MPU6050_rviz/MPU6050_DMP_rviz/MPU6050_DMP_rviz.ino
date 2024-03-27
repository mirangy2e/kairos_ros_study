#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
#define INTERRUPT_PIN 2
#define LED_PIN 13
bool blinkState = false;

bool dmpReady = false;
volatile bool mpuInterrupt = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;


uint8_t teapotPacket[28] = { '$', 0x03, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXAccelOffset(-1169);
    mpu.setYAccelOffset(744);
    mpu.setZAccelOffset(478);
    mpu.setXGyroOffset(128);
    mpu.setYGyroOffset(-41);
    mpu.setZGyroOffset(6);

    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    //DMP(디지털모션프로세서)가 초기화 되어 준비되었는지 확인 
    if (!dmpReady) return;


    //DMP에서 데이터를 읽어오는 동안 다른 프로그램 동작을 수행하지 않고 대기
    while (!mpuInterrupt && fifoCount < packetSize) {
        // mpuInterrupt: DMP에서 데이터가 준비되었음을 나타내는 플래그. 
        // FIFO 버퍼에서 packetSize만큼의 데이터를 읽어와서 후속 코드 실행
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    //FIFO 오버플로우 확인
    //mpuIntStatus & 0x10 : 이진수로 표현하면 00010000이고 5번 인덱스의 비트가 1임-> 오버플로 상태 비트
    //fifoCount == 1024 : FIFO 버퍼에 저장된 데이터의 수 -> 버퍼가 가득찼는지
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    //mpuIntStatus & 0x02 : 00000010 로 표현되며 2번째 비트가 활성 -> DMP 데이터가 준비됨
    } else if (mpuIntStatus & 0x02) {


        //DMP 패킷 하나의 크기만큼의 데이터가 충분히 쌓일 때까지 대기하는 역할
        //fifoCount는 현재 FIFO 버퍼에 저장된 데이터의 수를 나타내고
        //packetSize는 기대되는 DMP(Digital Motion Processor) 패킷의 크기
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // FIFO 버퍼에서 packetSize만큼의 바이트를 읽어와서 fifoBuffer 배열에 저장
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        teapotPacket[2] = fifoBuffer[0];
        teapotPacket[3] = fifoBuffer[1];
        teapotPacket[4] = fifoBuffer[4];
        teapotPacket[5] = fifoBuffer[5];
        teapotPacket[6] = fifoBuffer[8];
        teapotPacket[7] = fifoBuffer[9];
        teapotPacket[8] = fifoBuffer[12];
        teapotPacket[9] = fifoBuffer[13];
        teapotPacket[10] = fifoBuffer[16];
        teapotPacket[11] = fifoBuffer[17];
        teapotPacket[12] = fifoBuffer[20];
        teapotPacket[13] = fifoBuffer[21];
        teapotPacket[14] = fifoBuffer[24];
        teapotPacket[15] = fifoBuffer[25];
        teapotPacket[16] = fifoBuffer[28];
        teapotPacket[17] = fifoBuffer[29];
        teapotPacket[18] = fifoBuffer[32];
        teapotPacket[19] = fifoBuffer[33];
        teapotPacket[20] = fifoBuffer[36];
        teapotPacket[21] = fifoBuffer[37];
        int16_t temperature = mpu.getTemperature();
        teapotPacket[22] = temperature >> 8;
        teapotPacket[23] = temperature & 0xFF;
        Serial.write(teapotPacket, 28);
        teapotPacket[25]++;
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
