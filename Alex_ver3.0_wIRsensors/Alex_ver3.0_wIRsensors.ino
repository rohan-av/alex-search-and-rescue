
#include <avr/sleep.h>
#include <serialize.h>
#include <math.h>

#include "packet.h"
#include "constants.h"

/*
   Alex's configuration constants
*/

//#define PI            3.141592654

#define ALEX_LENGTH   8
#define ALEX_BREADTH  3

/*
 *  Alex's Power saving bits
 */
#define PRR_TWI_MASK            0b10000000
#define PRR_SPI_MASK            0b00000100
#define ADCSRA_ADC_MASK         0b10000000
#define PRR_ADC_MASK            0b00000001
#define PRR_TIMER2_MASK         0b01000000
#define PRR_TIMER0_MASK         0b00100000
#define PRR_TIMER1_MASK         0b00001000
#define SMCR_SLEEP_ENABLE_MASK  0b00000001
#define SMCR_IDLE_MODE_MASK     0b11110001  

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
#define LR                0b00100000  //5   // Left reverse pin  OC0B
#define LF                0b01000000  //6   // Left foward pin  OC0A
#define RR                0b00000100  //10  // Right reverse pin  OC1B
#define RF                0b00000010  //9  // Right forward pin   OC1A

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

unsigned int adcleftvalue;
unsigned int adcrightvalue;
static int toggle = 0;
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
  int len;

  len = readSerial(buffer);

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
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.

  EICRA = 0b00001010;
  EIMSK = 0b00000011;
}


// Left Encoder
ISR(INT0_vect)
{
  leftISR();
}

// Right Encoder
ISR(INT1_vect)
{
  rightISR();
}

ISR(TIMER0_COMPA_vect)
{
  if(leftForwardTicks < rightForwardTicks && dir == FORWARD){
    OCR0A = val + 20;
  } else if (adcleftvalue > 140){
    OCR0A = val + 20;
  } else {
    OCR0A = val;  
  }
}

ISR(TIMER0_COMPB_vect)
{
  if(leftForwardTicks < rightForwardTicks && dir == BACKWARD){
    OCR0B = val + 20;
  } else if (adcleftvalue > 140){
    OCR0B = val + 20;
  } else {
    OCR0B = val;  
  }
}


ISR(TIMER1_COMPA_vect)
{
  if(leftForwardTicks > rightForwardTicks && dir == FORWARD){
    OCR1AL = val + 20;
  } else if (adcleftvalue > 140){
    OCR1AL = val + 20;
  } else {
    OCR1AL = val;  
  }
}

ISR(TIMER1_COMPB_vect)
{
  if(leftForwardTicks > rightForwardTicks && dir == BACKWARD){
    OCR1BL = val + 20;
  } else if (adcleftvalue > 140){
    OCR1BL = val + 20;
  } else {
    OCR1BL = val;  
  }
}

ISR(ADC_vect){
  unsigned int loval, hival;
  loval = ADCL;
  hival = ADCH;
  if(toggle == 1){
    adcrightvalue = hival * 256 + loval;
    adcrightvalue = adcrightvalue /1023.0 * 255.0;
    toggle = 0;
    ADMUX |= 0b00000001;
  } else {
    adcleftvalue = hival * 256 + loval;
    adcleftvalue = adcleftvalue / 1023.0 * 255.0;
    toggle = 1;    
    ADMUX &= 0b11111000;
  }  
  ADCSRA |= 0b01000000;
}
/*
   Setup and start codes for serial communications

*/
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(57600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.

}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count = 0;

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

/*
   Alex's motor drivers.

*/

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.

//working
void setupMotors()
{
  DDRD |= (LF | LR); // set them 
  DDRB |= (RR | RF); // as output

}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.

//working
void startMotors()
{
  PORTD &= ~(LF | LR);
  PORTB &= ~(RF | RR);
}

void setupTimer() {
  TCNT0 = 0; //set the timer to 0
  TCNT1L = 0; //set the timer to 0
  TCNT1H = 0;
  OCR0A = 0; //LR
  OCR0B = 0; //LF
  OCR1AL = 0; //RR
  OCR1AH = 0;
  OCR1BL = 0; //RF
  OCR1BH = 0;

  //settings for the clocks
  TCCR0A = 0b00000001; //phase correct 
  TCCR1A = 0b00000001; //8bit phase correct
  TCCR0B = 0b00000001; //phase correct 
  TCCR1B = 0b00000001;
  TIMSK0 = 0b110;
  TIMSK1 = 0b110;
}

void setupADC(){
  PRR &= 0b11111110;
  ADCSRA = 0b10001111;
  ADMUX = 0b01000000; // left
}


/*
 * Alex's Power Saving Function
 */
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
  setupADC();
  enablePullups();
  initializeState();
  setupPowerSaving();
  sei();
  ADCSRA |= 0b01000000;

  
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
