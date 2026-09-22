
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>


#define FAN_LIFT_PIN   PD5
#define FAN_DIR_PIN    PD6
#define LED_PIN        PB5
#define SERVO_PIN      PB1


const int SERVO_CENTER_US = 3150;
const int SERVO_LEFT_US   = 1750;
const int SERVO_RIGHT_US  = 4500;

volatile int servoPos = SERVO_CENTER_US;
int servoTarget = SERVO_CENTER_US;
const int SERVO_SPEED = 150;

volatile uint32_t millisCount = 0;

#define MPU6050_ADDR 0x68
const int FAN_DIR_FULL = 255;   
const int FAN_LIFT_90  = 230;
const int FAN_STOP = 0;


const int FAN_LIFT_TURN      = 50;
const uint32_t SCAN_SETTLE_MS = 200;
const uint32_t SCAN_MAX_MS    = 1500;
const uint32_t TURN_MAX_MS    = 3000;

const int      FAN_LIFT_SETTLE = 30;
const uint32_t SETTLE_MS       = 250;


const int32_t SLOW_CM      = 50;   
const int32_t TURN_CM      = 35;    
const int     FAN_DIR_SLOW = 110;   


const int32_t Kd_speed   = 150;  
const int     FAN_DIR_MIN = 85;  


const int32_t STUCK_CM      = 37;   // was 32: +7 to track the bumper geometry
const int32_t STUCK_DELTA   = 3;
const uint32_t STUCK_MS     = 900;

//  post-turn straighten 
const uint32_t STRAIGHTEN_MAX_MS = 500;

// #1 PD TURN CONTROLLER gains 
const int32_t Kp_turn = 18;   
const int32_t Kd_turn = 90;    
const int32_t TURN_DONE = 300; 
// steering gains (straight-line PI) 
const int32_t Kp = 30;
const int32_t Ki          = 2;
const int32_t I_CLAMP     = 8000;
const int     I_OUT_CLAMP = 600;


int32_t yaw = 0;
int32_t gyroOffset = 0;
uint32_t prevTime = 0, nowTime = 0;
uint32_t settleTimer = 0;
uint32_t straightTimer = 0;
uint32_t scanTimer = 0;
uint32_t turnTimer = 0;
uint32_t brakeTimer = 0;

int32_t yawSum = 0;

// scan state
int scanStep = 0;
uint8_t scanWait = 0;
int32_t leftDist = 0, rightDist = 0;

// stuck-detect state
uint32_t stuckTimer = 0;
int32_t  stuckDist = 0;
uint8_t  stuckCheck = 0;

// PD turn: latest gyro rate shared in
int32_t gyroRate = 0;

// PD approach: closing-rate tracking
int32_t lastFrontDist = 999;
int32_t closingRate = 0;

// Ultrasonic via INT0 (PD2), non-blocking timing 
volatile uint32_t timer0Ovf = 0;
volatile uint8_t  echoStep = 0;
volatile uint32_t echoStart = 0;
volatile uint32_t echoTime = 0;
volatile uint8_t  echoDone = 0;

static inline uint32_t micros_now(void) {
    uint32_t ov;
    uint8_t  t;
    uint8_t  oldSREG = SREG;
    cli();
    ov = timer0Ovf;
    t  = TCNT0;
    if ((TIFR0 & (1 << TOV0)) && t < 255) ov++;
    SREG = oldSREG;
    return (ov * 1024UL) + ((uint32_t)t * 4UL);
}

ISR(TIMER0_OVF_vect) {
    millisCount++;
    timer0Ovf++;
}

ISR(INT0_vect) {
    uint32_t now = micros_now();
    if (PIND & (1 << PD2)) {
        echoStart = now;
        echoStep = 2;
    } else {
        if (echoStep == 2) {
            echoTime = now - echoStart;
            echoDone = 1;
            echoStep = 0;
        }
    }
}

uint32_t get_millis() {
    uint32_t m;
    uint8_t oldSREG = SREG;
    cli();
    m = millisCount;
    SREG = oldSREG;
    return m;
}

void trig_pulse(void) {
    PORTB |= (1 << PB3);
    _delay_us(10);
    PORTB &= ~(1 << PB3);
}

int32_t us_sensor_get_distance(void) {
    echoDone = 0;
    echoStep = 1;
    trig_pulse();

    uint16_t guard = 0;
    while (!echoDone && guard < 30000) {
        _delay_us(1);
        guard++;
    }
    if (!echoDone) { echoStep = 0; return 0; }

    uint32_t w = echoTime;
    int32_t d = (int32_t)(w / 58UL);
    return d;
}


void UART_init(unsigned int ubrr) {
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  UCSR0B = (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
void UART_TxChar(char ch) { while (!(UCSR0A & (1 << UDRE0))); UDR0 = ch; }

void I2C_init() { TWSR = 0; TWBR = 0x48; TWCR = (1 << TWEN); }
void I2C_start() { TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); while(!(TWCR&(1<<TWINT))); }
void I2C_stop() { TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); }
void I2C_write(uint8_t d){ TWDR=d; TWCR=(1<<TWINT)|(1<<TWEN); while(!(TWCR&(1<<TWINT))); }
uint8_t I2C_read_nack(){ TWCR=(1<<TWINT)|(1<<TWEN); while(!(TWCR&(1<<TWINT))); return TWDR; }

void MPU6050_write(uint8_t r, uint8_t d){
  I2C_start(); I2C_write(MPU6050_ADDR<<1); I2C_write(r); I2C_write(d); I2C_stop();
}
uint8_t MPU6050_read(uint8_t r){
  uint8_t d; I2C_start(); I2C_write(MPU6050_ADDR<<1); I2C_write(r);
  I2C_start(); I2C_write((MPU6050_ADDR<<1)|1); d = I2C_read_nack(); I2C_stop(); return d;
}
void MPU6050_init(){ MPU6050_write(0x6B,0x00); }

int16_t read_gyro_z_raw() {
  int16_t z_raw = (MPU6050_read(0x47) << 8) | MPU6050_read(0x48);
  return z_raw * 1;
}

void calculate_IMU_error() {
  int32_t gyroSum = 0;
  for(int i=0; i<200; i++) {
    gyroSum += read_gyro_z_raw();
    if(i % 10 == 0) PORTB ^= (1 << PB5);
    _delay_ms(10);
  }
  int32_t gyroAvg = gyroSum / 200;
  gyroOffset = (gyroAvg * 100) / 131;
  PORTB &= ~(1 << PB5);
}

int32_t read_open(void) {
    int32_t a = us_sensor_get_distance(); _delay_ms(20);
    int32_t b = us_sensor_get_distance(); _delay_ms(20);
    int32_t c = us_sensor_get_distance();
    if (a == 0) a = 999;
    if (b == 0) b = 999;
    if (c == 0) c = 999;
    int32_t hi = (a > b) ? a : b; hi = (hi > c) ? hi : c;
    int32_t lo = (a < b) ? a : b; lo = (lo < c) ? lo : c;
    return a + b + c - hi - lo;
}

// Servo physically rotated 180deg, mirror commanded pulse about center before output.
ISR(TIMER1_COMPA_vect){ OCR1A = 2 * SERVO_CENTER_US - servoPos; }

//  States 
enum SystemState {
  CRUISE       = 0,
  APPROACH     = 1,
  SCAN_SIDES   = 2,
  EXECUTE_TURN = 3,
  SETTLE       = 4,   // brief deflation to kill residual spin after a turn
  STRAIGHTEN   = 5
};
SystemState systemState = CRUISE;

int turnDir = 1;
int32_t yawTarget = 0;

int main(void) {
    I2C_init();
    MPU6050_init();

    DDRB |= (1 << PB3);
    DDRD &= ~(1 << PD2);
    DDRB |= (1 << PB5);
    DDRD |= (1 << FAN_LIFT_PIN) | (1 << FAN_DIR_PIN);

    EICRA = (1 << ISC00);
    EIMSK = (1 << INT0);

    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01) | (1 << CS00);
    TIMSK0 |= (1 << TOIE0);

    DDRB |= (1<<DDB1);
    TCCR1A=0; TCCR1B=0; TCNT1=0;
    ICR1=20000;
    OCR1A=SERVO_CENTER_US;
    servoTarget = SERVO_CENTER_US;
    TCCR1A=(1<<COM1A1)|(1<<WGM11);
    TCCR1B=(1<<WGM13)|(1<<WGM12)|(1<<CS11);
    TIMSK1|=(1<<OCIE1A);

    UART_init(103);
    sei();

    calculate_IMU_error();

    OCR0A = FAN_STOP; OCR0B = FAN_STOP; _delay_ms(3000);
    OCR0B = FAN_LIFT_90; OCR0A = FAN_STOP; _delay_ms(1500);
    OCR0A = FAN_DIR_FULL;

    prevTime = get_millis();
    nowTime = prevTime;

    while(1) {
        int16_t gyroRaw = read_gyro_z_raw();
        int32_t gyroNow = ((int32_t)gyroRaw * 100) / 131;
        gyroRate = gyroNow;

        prevTime = nowTime;
        nowTime = get_millis();
        uint32_t dt = (nowTime >= prevTime) ? (nowTime - prevTime) : 0;

        yaw += ((gyroNow - gyroOffset) * (int32_t)dt) / 1000;

        int32_t frontCm = us_sensor_get_distance();

        int32_t closingNow = 0;
        if (frontCm > 0 && lastFrontDist > 0 && dt > 0) {
            closingNow = ((lastFrontDist - frontCm) * 1000L) / (int32_t)dt;
        }
        closingRate = (closingRate * 3 + closingNow) / 4;
        lastFrontDist = frontCm;

        switch(systemState){

            case CRUISE: {
                OCR0A = FAN_DIR_FULL;
                OCR0B = FAN_LIFT_90;

                int32_t error = yawTarget - yaw;

                yawSum += (error * (int32_t)dt) / 1000;
                if (yawSum >  I_CLAMP) yawSum =  I_CLAMP;
                if (yawSum < -I_CLAMP) yawSum = -I_CLAMP;

                int pCorr = (int)((error * Kp) / 100);
                int iCorr = (int)((yawSum * Ki) / 100);
                if (iCorr >  I_OUT_CLAMP) iCorr =  I_OUT_CLAMP;
                if (iCorr < -I_OUT_CLAMP) iCorr = -I_OUT_CLAMP;

                int steerAdj = pCorr + iCorr;
                int servoCmd = SERVO_CENTER_US + steerAdj;
                if (servoCmd < SERVO_LEFT_US)  servoCmd = SERVO_LEFT_US;
                if (servoCmd > SERVO_RIGHT_US) servoCmd = SERVO_RIGHT_US;
                servoTarget = servoCmd;

                static int32_t lastDist = 999;
                if (frontCm > 0) lastDist = frontCm;

                uint8_t nearWall =
                    (frontCm > 0 && frontCm < SLOW_CM) ||
                    (frontCm == 0 && lastDist < SLOW_CM);

                if (abs(servoPos - SERVO_CENTER_US) < 200 && nearWall) {
                    lastDist = 999;
                    stuckCheck = 0;
                    systemState = APPROACH;
                }
                break;
            }

            case APPROACH: {
                static int32_t lastDist2 = 999;
                if (frontCm > 0) lastDist2 = frontCm;

                int32_t distUsed = (frontCm > 0) ? frontCm : lastDist2;

                int32_t baseThrust;
                if (distUsed >= SLOW_CM)      baseThrust = FAN_DIR_FULL;
                else if (distUsed <= TURN_CM) baseThrust = FAN_DIR_SLOW;
                else baseThrust = FAN_DIR_SLOW +
                        ((int32_t)(FAN_DIR_FULL - FAN_DIR_SLOW) * (distUsed - TURN_CM))
                        / (SLOW_CM - TURN_CM);

                int32_t brakeAdj = (closingRate > 0) ? (Kd_speed * closingRate) / 100 : 0;

                int32_t thrustCmd = baseThrust - brakeAdj;
                if (thrustCmd > FAN_DIR_FULL) thrustCmd = FAN_DIR_FULL;
                if (thrustCmd < FAN_DIR_MIN)  thrustCmd = FAN_DIR_MIN;

                OCR0A = (uint8_t)thrustCmd;
                OCR0B = FAN_LIFT_90;

                int32_t error = yawTarget - yaw;
                yawSum += (error * (int32_t)dt) / 1000;
                if (yawSum >  I_CLAMP) yawSum =  I_CLAMP;
                if (yawSum < -I_CLAMP) yawSum = -I_CLAMP;

                int pCorr = (int)((error * Kp) / 100);
                int iCorr = (int)((yawSum * Ki) / 100);
                if (iCorr >  I_OUT_CLAMP) iCorr =  I_OUT_CLAMP;
                if (iCorr < -I_OUT_CLAMP) iCorr = -I_OUT_CLAMP;

                int steerAdj = pCorr + iCorr;
                int servoCmd = SERVO_CENTER_US + steerAdj;
                if (servoCmd < SERVO_LEFT_US)  servoCmd = SERVO_LEFT_US;
                if (servoCmd > SERVO_RIGHT_US) servoCmd = SERVO_RIGHT_US;
                servoTarget = servoCmd;

                uint8_t atWall =
                    (frontCm > 0 && frontCm < TURN_CM) ||
                    (frontCm == 0 && lastDist2 < TURN_CM);

                uint8_t stuckRange =
                    (frontCm > 0 && frontCm < STUCK_CM) ||
                    (frontCm == 0 && lastDist2 < STUCK_CM);
                uint8_t stuckFlag = 0;
                if (stuckRange) {
                    if (!stuckCheck) {
                        stuckCheck = 1;
                        stuckTimer = get_millis();
                        stuckDist = (frontCm > 0) ? frontCm : lastDist2;
                    } else {
                        int32_t cur = (frontCm > 0) ? frontCm : lastDist2;
                        if (labs((long)(cur - stuckDist)) > STUCK_DELTA) {
                            stuckTimer = get_millis();
                            stuckDist = cur;
                        } else if (get_millis() - stuckTimer >= STUCK_MS) {
                            stuckFlag = 1;
                        }
                    }
                } else {
                    stuckCheck = 0;
                }

                if (atWall || stuckFlag) {
                    lastDist2 = 999;
                    stuckCheck = 0;
                    scanStep = 0;
                    scanWait = 0;
                    scanTimer = get_millis();
                    systemState = SCAN_SIDES;
                }
                break;
            }

            case SCAN_SIDES:
                OCR0A = FAN_STOP;
                OCR0B = FAN_LIFT_TURN;

                if (scanStep == 0) {
                    servoTarget = SERVO_LEFT_US;
                    if (abs(servoPos - servoTarget) <= SERVO_SPEED ||
                        (get_millis() - scanTimer) >= SCAN_MAX_MS) {
                        if (!scanWait) { settleTimer = get_millis(); scanWait = 1; }
                        else if (get_millis() - settleTimer >= SCAN_SETTLE_MS) {
                            leftDist = read_open();
                            scanStep = 1; scanWait = 0;
                            scanTimer = get_millis();
                        }
                    }
                } else {
                    servoTarget = SERVO_RIGHT_US;
                    if (abs(servoPos - servoTarget) <= SERVO_SPEED ||
                        (get_millis() - scanTimer) >= SCAN_MAX_MS) {
                        if (!scanWait) { settleTimer = get_millis(); scanWait = 1; }
                        else if (get_millis() - settleTimer >= SCAN_SETTLE_MS) {
                            rightDist = read_open();

                            if (leftDist > rightDist) turnDir = -1;
                            else                      turnDir = 1;

                            yawTarget += (9000 * turnDir);   // 9000 = 90.00 degrees
                            servoTarget = SERVO_CENTER_US;
                            scanWait = 0;
                            turnTimer = get_millis();
                            systemState = EXECUTE_TURN;
                        }
                    }
                }
                break;

            case EXECUTE_TURN: {
                OCR0A = FAN_DIR_FULL;
                OCR0B = FAN_LIFT_TURN;

                int32_t yawLeft = yawTarget - yaw;
                int32_t pTerm = (yawLeft * Kp_turn) / 100;
                int32_t dTerm = (gyroRate * Kd_turn) / 100;
                int32_t turnCmd = pTerm - dTerm;

                int servoCmd = SERVO_CENTER_US + (int)turnCmd;
                if (servoCmd < SERVO_LEFT_US)  servoCmd = SERVO_LEFT_US;
                if (servoCmd > SERVO_RIGHT_US) servoCmd = SERVO_RIGHT_US;
                servoTarget = servoCmd;

                if (labs((long)yawLeft) <= TURN_DONE ||
                    (get_millis() - turnTimer >= TURN_MAX_MS)) {
                    yawSum = 0;
                    brakeTimer = get_millis();
                    systemState = SETTLE;
                }
                break;
            }

            case SETTLE:
                // Turn just finished. Deflate hard and cut thrust so floor friction kills leftover
                // spin, then reinflate and hand to straighten. Rudder centered.
                OCR0A = FAN_STOP;
                OCR0B = FAN_LIFT_SETTLE;
                servoTarget = SERVO_CENTER_US;
                if (get_millis() - brakeTimer >= SETTLE_MS) {
                    OCR0B = FAN_LIFT_90;
                    straightTimer = get_millis();
                    systemState = STRAIGHTEN;
                }
                break;

            case STRAIGHTEN: {
                OCR0A = FAN_DIR_SLOW;
                OCR0B = FAN_LIFT_90;

                int32_t error = yawTarget - yaw;

                yawSum += (error * (int32_t)dt) / 1000;
                if (yawSum >  I_CLAMP) yawSum =  I_CLAMP;
                if (yawSum < -I_CLAMP) yawSum = -I_CLAMP;

                int pCorr = (int)((error * Kp) / 100);
                int iCorr = (int)((yawSum * Ki) / 100);
                if (iCorr >  I_OUT_CLAMP) iCorr =  I_OUT_CLAMP;
                if (iCorr < -I_OUT_CLAMP) iCorr = -I_OUT_CLAMP;

                int steerAdj = pCorr + iCorr;
                int servoCmd = SERVO_CENTER_US + steerAdj;
                if (servoCmd < SERVO_LEFT_US)  servoCmd = SERVO_LEFT_US;
                if (servoCmd > SERVO_RIGHT_US) servoCmd = SERVO_RIGHT_US;
                servoTarget = servoCmd;

                if ((abs(servoPos - SERVO_CENTER_US) < 200) ||
                    (get_millis() - straightTimer >= STRAIGHTEN_MAX_MS)) {
                    systemState = CRUISE;
                }
                break;
            }
        }

        int diff = servoTarget - servoPos;
        if (abs(diff) > SERVO_SPEED) {
            if (diff > 0) servoPos += SERVO_SPEED;
            else          servoPos -= SERVO_SPEED;
        } else {
            servoPos = servoTarget;
        }
        OCR1A = 2 * SERVO_CENTER_US - servoPos;

        _delay_ms(40);
    }
    return 0;
}
