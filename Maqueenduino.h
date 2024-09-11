/*!
 * @file Maqueenduino.h
 * @brief This is a library for MaqueenPlus V2.1 from DFRobot
 * @details This library can be used to control the corresponding sensors on MaqueenPlus to complete different projects and functions
 * @copyright    Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence      The MIT License (MIT)
 * @author       [TangJie](jie.tang@dfrobot.com)
 * @version      V1.0.1
 * @eGDAte       2020-09-15
 * @url          https://github.com/DFRobot/DFRobot_MaqueenPlus
 */

#ifndef  _MAQUEENDUINO_H_
#define  _MAQUEENDUINO_H_

#include <Arduino.h>
#include <Wire.h>

#define ENABLE_DBG
#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif

#define MAQUEENDUINO_VERSION_CNT_REGISTER    0x32
#define MAQUEENDUINO_I2C_ADDR                0x10
#define MAQUEENDUINO_VERSION_DATA_REGISTER   0x33
#define MAQUEENDUINO_LEFT_LED_REGISTER       0x0B
#define MAQUEENDUINO_RIGHT_LED_REGISTER      0x0C
#define MAQUEENDUINO_LEFT_MOTOR_REGISTER     0x00
#define MAQUEENDUINO_RIGHT_MOTOR_REGISTER    0x02
#define MAQUEENDUINO_NEOPIXELS_N             4
#define MAQUEENDUINO_ADC0_REGISTER           0x1E
#define MAQUEENDUINO_ADC1_REGISTER           0x20
#define MAQUEENDUINO_ADC2_REGISTER           0x22
#define MAQUEENDUINO_ADC3_REGISTER           0x24
#define MAQUEENDUINO_ADC4_REGISTER           0x26
#define MAQUEENDUINO_LINE_STATE_REGISTER     0x1D

#define DFROBOT_MAQUEEMPLUS_IRPIN    16
#define DFROBOT_MAQUEEMPLUS_LEFT     1
#define DFROBOT_MAQUEEMPLUS_RIGHT    2
#define DFROBOT_MAQUEEMPLUS_L1       1
#define DFROBOT_MAQUEEMPLUS_L2       2
#define DFROBOT_MAQUEEMPLUS_L3       5
#define DFROBOT_MAQUEEMPLUS_R1       3
#define DFROBOT_MAQUEEMPLUS_R2       4
#define DFROBOT_MAQUEEMPLUS_R3       6
#define DFROBOT_MAQUEEMPLUS_S1       1
#define DFROBOT_MAQUEEMPLUS_S2       2

class Maqueenduino {

    public:
        /**
         * @struct ePID
         * @brief Enable or disable PID
         */
        typedef enum {
            OFF = 0,
            ON  = 1
        } State;

        /**
         * @enum ePosition
         * @brief Positionition selection, suitable for RGB LEDs and motor selection
         */
        typedef enum {
            LEFT  = 1,
            RIGHT = 2,
            ALL   = 3
        } Position;

        /**
         * @enum eDir
         * @brief Motor direction selection
         */
        typedef enum {
            CW  = 0,
            CCW = 1
        } Dir;

        /**
         * @enum LineSensor
         * @brief line-tracking sensor selection
         */
        typedef enum {
            L2 = 1,
            L1 = 2,
            M = 3,
            R1 = 4,
            R2 = 5
        } LineSensor;

        /**
         * @enum eServo
         * @brief Serve port selection
         */
        typedef enum {
            S1 = 1,
            S2 = 2,
            S3 = 3
        } Servo;

        /**
         * @enum ePin
         * @brief Ultrasonic pin selection
         */
        typedef enum {
            P0  = 0,
            P1  = 1,
            P2  = 2,
            P8  = 8,
            P12 = 12,
            P13 = 13,
            P14 = 14,
            P15 = 15
        } Pin;

        /**
         * @fn DFRobot_MaqueenPlus
         * @brief Constructor
         * @param pWire  I2C object
         * @param address I2C address
         * @return None 
         */
        Maqueenduino(TwoWire *pWire = &Wire, uint8_t address = MAQUEENDUINO_I2C_ADDR) {
            _pWire = pWire;
            _address = address;
        };

        /**
         * @fn begin
         * @brief Init I2C until success 
         * @return uint8_t type, indicate returning init status
         * @retval 0 Init succeeded
         * @retval -1 Init failed
         */
        uint8_t begin(void);

        /**
         * @fn PIDSwitch
         * @brief PID operation control
         * @param state To disable or enable PID
         * @return None
         */
        void PIDSwitch(State state);

        /**
         * @fn motorControl
         * @brief Control the direction and speed of MaqueenPlus
         * @param motor Motor control selection
         * @param direction Motor rotation direction 
         * @param speed Motor speed(range:0~255)
         * @return None
         */
        void motorControl(Position motor, Dir direction, uint8_t speed);

        /**
         * @fn motorControl
         * @brief Control the direction and speed of MaqueenPlus
         * @param motor Motor control selection
         * @param direction Motor rotation direction 
         * @param speed Motor speed(range:0~255)
         * @return None
         */
        void motorStop(Position motor);

        /**
         * @fn getSpeed
         * @brief Get wheel speed
         * @param motor Select left or right motor
         * @return Return the speed of the selected motor 
         */
        uint8_t getSpeed(Position motor);

        /**
         * @fn getDirection
         * @brief Get rotation direction
         * @param motor Select left or right motor 
         * @return  0: stop 1: forward 2: backward
         */
        uint8_t getDirection(Position motor);

        /**
         * @fn getDistance
         * @brief  Get the number of revolutions
         * @param motor Select left or right motor
         * @return Return the revolutions
         */
        float getDistance(Position motor);

        /**
         * @fn clearDistance
         * @brief Clear the number of revolutions
         * @param motor Select left or right motor
         * @return None
         */
        void clearDistance(Position motor);

        /**
         * @fn getLineSensor
         * @brief  Get line-tracking sensor status
         * @param senser Select line-tracking sensor 
         * @return Returns the status of line-tracking sensor
         */
        uint8_t getLineSensor(LineSensor senser);

        /**
         * @fn getGrayscale
         * @brief Get grayscale value of line-tracking sensor
         * @param senser Select line-tracking sensor 
         * @return  Return the grayscale value of line-tracking sensor
         */
        uint16_t getGrayscale(LineSensor senser);

        /**
         * @fn setRGB
         * @brief Set the RGB led color
         * @param colour Select Color
         * @return None
         */
        void setLED(Position light, State state);

        /**
         * @fn servoControl
         * @brief Servo control
         * @param servo Select servo
         * @param angle Control servo angle(range:0°~180°)
         * @return None
         */
        void servoControl(Servo servo, uint8_t angle);

        /**
         * @fn ultraSonic
         * @brief Get ultrasonic distance
         * @param trig   TRIG Pin
         * @param echo   ECHO Pin
         * @return  Return ultrasonic distance information
         */
        uint8_t ultraSonic(Pin trig, Pin echo);

        /**
         * @fn getIR
         * @brief Get infrared data
         * @return  Return infrared key information
         */
        uint32_t getIR(void);

        /**
         * @fn getVersion
         * @brief  Get version information
         * @return  Return version information
         */
        String getVersion(void); 

    private:
        uint8_t _echoPin;
        uint8_t _trlgPin;
        TwoWire *_pWire;
        uint8_t _address;
        uint16_t _pulseWidth = 0;
        uint16_t _irCode = 0x00;
        uint32_t _duration;
        /**
         * @fn myPulseIn
         * @brief  Get level change time
         */
        uint64_t myPulseIn(uint32_t pin, uint32_t value, long maxDuration = 100000);

        /**
         * @fn writeReg
         * @brief   Write register value through IIC bus
         * @param reg  Register address 8bits
         * @param pBuf Storage cache to write data in
         * @param size The length of data to be written
         */
        void writeReg(uint8_t reg, void *pBuf, size_t size);

        /**
         * @fn readReg
         * @brief Read register value through IIC bus
         * @param reg  Register address 8bits
         * @param pBuf Read data storage cache
         * @param size Read the length of data
         * @return Return the read length, 0 indicates it failed to read 
         */
        uint8_t readReg(uint8_t reg, void *pBuf, size_t size);

};
#endif // _MAQUEENDUINO_H_
