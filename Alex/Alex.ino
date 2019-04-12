

#include <avr/sleep.h>
#include <serialize.h>
#include <math.h>

#include "packet.h"
#include "constants.h"
#include "buffer.h"

/*
   Alex's configuration constants
 */

//#define PI            3.141592654

#define ALEX_LENGTH   8
#define ALEX_BREADTH  3

#define PRR_TWI_MASK 0b10000000 
#define PRR_SPI_MASK 0b00000100 
#define ADCSRA_ADC_MASK 0b10000000 
#define PRR_ADC_MASK 0b00000001 
#define PRR_TIMER2_MASK 0b01000000 
#define PRR_TIMER0_MASK 0b00100000 
#define PRR_TIMER1_MASK 0b00001000 
#define SMCR_SLEEP_ENABLE_MASK  0b00000001 
#define SMCR_IDLE_MODE_MASK 0b11110001 

typedef enum {
    STOP = 0,
    FORWARD = 1,
    BACKWARD = 2,
    LEFT = 3,
    RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV      192

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.1

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  5   // Left forward pin
#define LR                  6   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  9  // Right reverse pin

TBuffer _recvBuffer; //for serial comms
TBuffer _xmitBuffer; //for serial comms
/*
   Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variables to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

unsigned long deltaTicks;
unsigned long targetTicks;

volatile long val;


/* Arduino Code for turning off Watchdog Timer (WDT) W11 Studio 2*/
void WDT_off(void) { /* Global interrupt should be turned OFF here if not already done so */ 

    /* Clear WDRF in MCUSR */ 
    MCUSR &= ~(1<<WDRF); 

    /* Write logical one to WDCE and WDE */ /* Keep old prescaler setting to prevent unintentional time-out */ 
    WDTCSR |= (1<<WDCE) | (1<<WDE); 

    /* Turn off WDT */ 
    WDTCSR = 0x00; 

    /* Global interrupt should be turned ON here if subsequent operations after calling this function do not require
     * turning off global interrupt */ 
}


/*W11 Studio 2 Add*/
void setupPowerSaving(){
    //Turn off Watchdog Timer
    WDT_off();

    //Modify PRR to shut down TWI
    PRR |= PRR_TWI_MASK;

    // Modify PRR to shut down SPI
    PRR |= PRR_SPI_MASK;

    // Modify ADCSRA to disable ADC,    
    ADCSRA &= ~(ADCSRA_ADC_MASK);

    // then modify PRR to shut down ADC  
    PRR |= PRR_ADC_MASK;

    // Set the SMCR to choose the IDLE sleep mode   
    SMCR |= SMCR_IDLE_MODE_MASK;

    // Do not set the Sleep Enable (SE) bit yet 

    // Set Port B Pin 5 as output pin, then write a logic LOW   
    // to it so that the LED tied to Arduino's Pin 13 is OFF.
    DDRB |= PRR_TIMER0_MASK;
    PORTB &= ~(PRR_TIMER0_MASK);

}

/*W11 Studio 2 Add*/
void putArduinoToIdle() {   
    // Modify PRR to shut down TIMER 0, 1, and 2      
    PRR |= (PRR_TIMER0_MASK | PRR_TIMER1_MASK | PRR_TIMER2_MASK);

    // Modify SE bit in SMCR to enable (i.e.,allow) sleep      
    SMCR |= SMCR_SLEEP_ENABLE_MASK;

    // This function puts ATmega328P’s MCU into sleep   
    sleep_cpu();     

    // Modify SE bit in SMCR to disable (i.e., disallow) sleep 
    SMCR &= ~(SMCR_SLEEP_ENABLE_MASK);

    // Modify PRR to power up TIMER 0, 1, and 2 
    PRR &= ~(PRR_TIMER0_MASK | PRR_TIMER1_MASK | PRR_TIMER2_MASK);
} 

/*

   Alex Communication Routines.

 */

float AlexDiagonal = 0.0;
float AlexCirc = 0.0;

TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".

    char buffer[PACKET_SIZE];
    unsigned char temp[PACKET_SIZE];
    int len;
    
    len = readSerial(temp);
    for(int i = 0; i < PACKET_SIZE; i++){
      
    }

    if (len == 0)
        return PACKET_INCOMPLETE;
    else
        return deserialize(buffer, len, packet);

}

void sendStatus()
{
    // Implement code to send back a packet containing key
    // information like leftTicks, rightTicks, leftRevs, rightRevs
    // forwardDist and reverseDist
    // Use the params array to store this information, and set the
    // packetType and command files accordingly, then use sendResponse
    // to send out the packet. See sendMessage on how to use sendResponse.
    //h


    TPacket statusPacket;
    statusPacket.packetType = PACKET_TYPE_RESPONSE;
    statusPacket.command = RESP_STATUS;
    statusPacket.params[0] = leftForwardTicks;
    statusPacket.params[1] = rightForwardTicks;
    statusPacket.params[2] = leftReverseTicks;
    statusPacket.params[3] = rightReverseTicks;
    statusPacket.params[4] = leftForwardTicksTurns;
    statusPacket.params[5] = rightForwardTicksTurns;
    statusPacket.params[6] = leftReverseTicksTurns;
    statusPacket.params[7] = rightReverseTicksTurns;
    statusPacket.params[8] = forwardDist;
    statusPacket.params[9] = reverseDist;

    sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
    // Sends text messages back to the Pi. Useful
    // for debugging.

    TPacket messagePacket;
    messagePacket.packetType = PACKET_TYPE_MESSAGE;
    strncpy(messagePacket.data, message, MAX_STR_LEN);
    sendResponse(&messagePacket);
}

void sendBadPacket()
{
    // Tell the Pi that it sent us a packet with a bad
    // magic number.

    TPacket badPacket;
    badPacket.packetType = PACKET_TYPE_ERROR;
    badPacket.command = RESP_BAD_PACKET;
    sendResponse(&badPacket);

}

void sendBadChecksum()
{
    // Tell the Pi that it sent us a packet with a bad
    // checksum.

    TPacket badChecksum;
    badChecksum.packetType = PACKET_TYPE_ERROR;
    badChecksum.command = RESP_BAD_CHECKSUM;
    sendResponse(&badChecksum);
}

void sendBadCommand()
{
    // Tell the Pi that we don't understand its
    // command sent to us.

    TPacket badCommand;
    badCommand.packetType = PACKET_TYPE_ERROR;
    badCommand.command = RESP_BAD_COMMAND;
    sendResponse(&badCommand);

}

void sendBadResponse()
{
    TPacket badResponse;
    badResponse.packetType = PACKET_TYPE_ERROR;
    badResponse.command = RESP_BAD_RESPONSE;
    sendResponse(&badResponse);
}

void sendOK()
{
    TPacket okPacket;
    okPacket.packetType = PACKET_TYPE_RESPONSE;
    okPacket.command = RESP_OK;
    sendResponse(&okPacket);
}

void sendResponse(TPacket *packet)
{
    // Takes a packet, serializes it then sends it out
    // over the serial port.
    char buffer[PACKET_SIZE];
    int len;

    len = serialize(buffer, packet, sizeof(TPacket));
    writeSerial(buffer, len);
}


/*
   Setup and start codes for external interrupts and
   pullup resistors.

 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
    // Use bare-metal to enable the pull-up resistors on pins
    // 2 and 3. These are pins PD2 and PD3 respectively.
    // We set bits 2 and 3 in DDRD to 0 to make them inputs.

    DDRD  &= 0b11110011;
    PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
    if (dir == FORWARD) {
        leftForwardTicks++;
    } else if (dir == BACKWARD) {
        leftReverseTicks++;
    } else if (dir == RIGHT) {
        leftForwardTicksTurns++;
    } else if (dir == LEFT) {
        leftReverseTicksTurns++;
    }

    //  leftRevs = leftTicks / COUNTS_PER_REV;

    // We calculate forwardDist only in leftISR because we
    // assume that the left and right wheels move at the same
    // time.
    //  forwardDist = leftRevs * WHEEL_CIRC;
    //
    //  Serial.print("LEFT: ");
    //  Serial.println(leftTicks);
}

void rightISR()
{
    if (dir == FORWARD) {
        rightForwardTicks++;
        forwardDist = (unsigned long) ((float) rightForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
    } else if (dir == BACKWARD) {
        rightReverseTicks++;
        reverseDist = (unsigned long) ((float) rightReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
    } else if (dir == LEFT) {
        rightForwardTicksTurns++;
    } else if (dir == RIGHT) {
        rightReverseTicksTurns++;
    }

    //  rightRevs = rightTicks / COUNTS_PER_REV;
    //  Serial.print("RIGHT: ");
    //  Serial.println(rightForwardTicks);
}

// ISR for receive interrupt. Any data we get from UDR0 is written to the receive buffer.
ISR(USART_RX_vect)
{
  unsigned char data = UDR0;
  
  //Note: This will fail silently and data will be lost if recvBuffer is full.
  writeBuffer(&_recvBuffer, data);
}

// ISR for UDR0 empty interrupt. We get the next character from the trasmit buffer if any and write to UDR0.
// Otherwise we disable the UDRE interrupt by writing a 0 to the UDRIE0 bit (bit 5) of UCSR0B
ISR(USART_UDRE_vect)
{
  unsigned char data = UDR0;
  TBufferResult result = readBuffer(&_xmitBuffer, &data);

  if(result == BUFFER_OK)
    UDR0 = data;
  else;
    if(result == BUFFER_EMPTY)
      UCSR0B &=0b11011111;
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
    // Use bare-metal to configure pins 2 and 3 to be
    // falling edge triggered. Remember to enable
    // the INT0 and INT1 interrupts.

    EICRA = 0b00001010;
    EIMSK = 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect)
{
    leftISR();
}

ISR(INT1_vect)
{
    rightISR();
}

ISR(TIMER0_COMPA_vect)
{
  OCR0A = val;  
}

ISR(TIMER0_COMPB_vect)
{
  OCR0B = val;
}


ISR(TIMER1_COMPA_vect)
{
  OCR1AL = val;  
}

ISR(TIMER1_COMPB_vect)
{
  OCR1BL = val;
}

// Implement INT0 and INT1 ISRs above.

/*
   Setup and start codes for serial communications

 */
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
    // To replace later with bare-metal.
    UBRR0L = 16;
    UBRR0H = 0;
    UCSR0C = 0b00000110;
    UCSR0A = 0;
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
    // Empty for now. To be replaced with bare-metal code
    // later on.
    UCSR0B = 0b10111000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(unsigned char *buffer)
{

    int count = 0;

    TBufferResult result;

    do
    {
       result = readBuffer(&_recvBuffer, &buffer[count]);

       if(result == BUFFER_OK)
        count++;
    } while (result == BUFFER_OK);

    return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  TBufferResult result = BUFFER_OK;

  int i;

  for(i=1; i < PACKET_SIZE && result == BUFFER_OK; i++)
    result = writeBuffer(&_xmitBuffer, buffer[i]);

  UDR0 = buffer[0];

  UCSR0B |= 0b00100000;
}

/*
   Alex's motor drivers.

 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.TBufferResult writeBuffer(TBuffer *buffer, unsigned char data)
void setupMotors()
{
    /* Our motor set up is:
       A1IN - Pin 5, PD5, OC0B
       A2IN - Pin 6, PD6, OC0A
       B1IN - Pin 10, PB2, OC1B
       B2In - pIN 11, PB3, OC2A
     */

}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{

}

void setupTimer() {
    TCNT0 = 0;
    TCNT1 = 0;
    TCNT2 = 0;
    OCR0A = 128;
    OCR0B = 128;
    OCR1B = 32767;
    OCR2A = 128;

}

// Convert percentages to PWM values
int pwmVal(float speed)
{
    if (speed < 0.0)
        speed = 0;

    if (speed > 100.0)
        speed = 100.0;

    return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  
  if (dist == 0)
    deltaDist = 999999;
  else
    deltaDist = dist;

  newDist = forwardDist + deltaDist;

  dir = FORWARD;

   val = pwmVal(speed);

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
//  OCR0A = val;
//  OCR1AL = val;
  TCCR0A |= 0b10000000; //set OCR0A LF
  TCCR1A |= 0b10000000; //set OCR1A RF
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  if (dist == 0)
    deltaDist = 9999999;
  else
    deltaDist = dist;

  newDist = reverseDist + deltaDist;

  dir = BACKWARD;

   val = pwmVal(speed);

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
//  OCR0B = val;
//  OCR1BL = val;
  TCCR0A |= 0b00100000; //set OCR0B LR
  TCCR1A |= 0b00100000; //set OCR1B RR
}

unsigned long computeDeltaTicks(float ang) {
  unsigned long ticks = (unsigned long) ((ang * AlexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));

  return ticks;
}


// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
   val = pwmVal(speed);

  dir = LEFT;

  if (ang == 0)
    deltaTicks = 9999999;
  else
    deltaTicks = computeDeltaTicks(ang);

  targetTicks = leftReverseTicksTurns + deltaTicks;
  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
//  OCR0B = val;
//  OCR1AL = val;
  TCCR0A |= 0b00100000; //set OCR0B LR
  TCCR1A |= 0b10000000; //set OCR1A RF
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;

     val = pwmVal(speed);


  if (ang == 0)
    deltaTicks = 9999999;
  else
    deltaTicks = computeDeltaTicks(ang);

  targetTicks = rightReverseTicksTurns + deltaTicks;

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
//  OCR0A = val;
//  OCR1BL = val;
  TCCR0A |= 0b10000000; //set OCR0A LF
  TCCR1A |= 0b00100000; //set OCR1B RR
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  TCCR0A &= 0b00001111;
  TCCR1A &= 0b00001111;
  PORTD &= ~(LR | LF);
  PORTB &= ~(RR | RF);
}
/*
   Alex's setup and run codes

 */

// Clears all our counters
void clearCounters()
{
    leftForwardTicks = 0;
    rightForwardTicks = 0;
    leftReverseTicks = 0;
    rightReverseTicks = 0;
    leftForwardTicksTurns = 0;
    rightForwardTicksTurns = 0;
    leftReverseTicksTurns = 0;
    rightReverseTicksTurns = 0;
    //  leftRevs=0;
    //  rightRevs=0;
    forwardDist = 0;
    reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
    clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
    clearCounters();
}

void handleCommand(TPacket *command)
{
    switch (command->command)
    {
        // For movement commands, param[0] = distance, param[1] = speed.
        case COMMAND_FORWARD:
            sendOK();
            forward((float) command->params[0], (float) command->params[1]);
            break;

        case COMMAND_REVERSE:
            sendOK();
            reverse((float) command->params[0], (float) command->params[1]);
            break;

        case COMMAND_TURN_LEFT:
            sendOK();
            left((float) command->params[0], (float) command->params[1]);
            break;

        case COMMAND_TURN_RIGHT:
            sendOK();
            right((float) command->params[0], (float) command->params[1]);
            break;

        case COMMAND_STOP:
            sendOK();
            stop();
            break;

        case COMMAND_GET_STATS:
            sendStatus();
            break;

        case COMMAND_CLEAR_STATS:
            sendOK();
            clearOneCounter(command->params[0]);
            break;

        default:
            sendBadCommand();
    }
}

void waitForHello()
{
    int exit = 0;

    while (!exit)
    {
        TPacket hello;
        TResult result;

        do
        {
            result = readPacket(&hello);
        } while (result == PACKET_INCOMPLETE);

        if (result == PACKET_OK)
        {
            if (hello.packetType == PACKET_TYPE_HELLO)
            {


                sendOK();
                exit = 1;
            }
            else
                sendBadResponse();
        }
        else if (result == PACKET_BAD)
        {
            sendBadPacket();
        }
        else if (result == PACKET_CHECKSUM_BAD)
            sendBadChecksum();
    } // !exit
}

void setup() {
    // put your setup code here, to run once:
    AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
    AlexCirc = PI * AlexDiagonal;
    cli();
    setupEINT();
    setupSerial();
    startSerial();
    setupMotors();
    startMotors();
    setupTimer();

    setupPowerSaving();

    enablePullups();
    initializeState();
    sei();


}

void handlePacket(TPacket *packet)
{
    switch (packet->packetType)
    {
        case PACKET_TYPE_COMMAND:
            handleCommand(packet);
            break;

        case PACKET_TYPE_RESPONSE:
            break;

        case PACKET_TYPE_ERROR:
            break;

        case PACKET_TYPE_MESSAGE:
            break;

        case PACKET_TYPE_HELLO:
            break;
    }
}

int count = 0;

void loop() {

    // Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2


    // Uncomment the code below for Week 9 Studio 2


    //  put your main code here, to run repeatedly:
    TPacket recvPacket; // This holds commands from the Pi

    TResult result = readPacket(&recvPacket);

    if (result == PACKET_OK)
        handlePacket(&recvPacket);

    /*W11 Studio 2 Add*/
    else if (result == PACKET_INCOMPLETE && dir == STOP){
        putArduinoToIdle();
    }

    else if (result == PACKET_BAD)
    {
        sendBadPacket();
    }

    else if (result == PACKET_CHECKSUM_BAD)
    {
        sendBadChecksum();
    }

    if (deltaDist > 0) {
        //Serial.println(deltaDist);
        if (dir == FORWARD) {
            if (forwardDist > newDist) {
                deltaDist = 0;
                newDist = 0;
                stop();
            }
        } else if (dir == BACKWARD) {
            if (reverseDist > newDist) {
                deltaDist = 0;
                newDist = 0;
                stop();
            }
        } else if (dir == STOP) {
            deltaDist = 0;
            newDist = 0;
            stop();
        }
    }

    if (deltaTicks > 0) {
        if (dir == LEFT) {
            if (leftReverseTicksTurns >= targetTicks) {
                deltaTicks = 0;
                targetTicks = 0;
                stop();
            }
        }
        else if (dir == RIGHT) {
            if (rightReverseTicksTurns >= targetTicks) {
                deltaTicks = 0;
                targetTicks = 0;
                stop();
            }
        }
        else if (dir == STOP) {
            deltaTicks = 0;
            targetTicks = 0;
            stop();
        }
    }

}
