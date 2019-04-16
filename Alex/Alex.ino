#include <buffer.h>

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
    Alex's Power saving bits
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

/*
    Alex's direction commands
*/
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

// Motor control pins.
#define LR                0b00100000  //5   // Left reverse pin  OC0B
#define LF                0b01000000  //6   // Left foward pin  OC0A
#define RR                0b00000100  //10  // Right reverse pin  OC1B
#define RF                0b00000010  //9  // Right forward pin   OC1A

/*
    UART circular buffer
*/
TBuffer _recvBuffer;
TBuffer _xmitBuffer;

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

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variables to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

unsigned long deltaTicks;
unsigned long targetTicks;

volatile long val;
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
  // These are pins PD2 and PD3 respectively.
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

//relies solely on the right encoder for forward and reverse distance
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
  // Enable the INT0 and INT1 interrupts
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

// LF motor, timer0
ISR(TIMER0_COMPA_vect)
{
  OCR0A = val;
}

// LR motor, timer0
ISR(TIMER0_COMPB_vect)
{
  OCR0B = val;
}

// RF motor, timer1
ISR(TIMER1_COMPA_vect)
{
  OCR1AL = val;
}

// RR motor, timer1
ISR(TIMER1_COMPB_vect)
{
  OCR1BL = val;
}

//receive Interrupt. Any data we get from UDR0 is written to the _recvBuffer
ISR(USART_RX_vect)
{
  char data = UDR0;
  stop();
  writeBuffer(&_recvBuffer, data);
}

//UDR0 empty interrupt. WE get the next character from the _xmitBuffer if any and write it to UDR0.
// Otherwise we disable the UDRE interrupt by writing a 0 to the UDRIE0 bit(bit 5) of UCSR0B
ISR(USART_UDRE_vect)
{
  char data;
  TBufferResult result = readBuffer(&_xmitBuffer, &data);

  if (result == BUFFER_OK)
    UDR0 = data;
  else if (result == BUFFER_EMPTY)
    UCSR0B &= 0B11011111;
}
/*
   Setup and start codes for serial communications

*/
void setupSerial()
{
  // b = round(16000000 / (16 * 57600) - 1 = 16)
  UBRR0L = 16;
  UBRR0H = 0;

  //asynchronous mode bit 7 and 6 are 00.
  //no parity bits 5 and 4 are 00.
  //1 stop bit, bit 3 is 0.
  //8 bits, so bits 2 and 1(UCSR0C) should be 1. (UCSR0B bit 2 should be 0)
  //bit 0 (UCPOL0) should always be 0.
  UCSR0C = 0b00000110;
  //to ensure U2X0 and MPCM0 are cleared
  UCSR0A = 0;
}

void startSerial()
{
  //RXEN0 and TXEN0 (bits 4 and 3) to 1 to receive and transmit
  //RXCIE0 (bit 7) and UDRIE0 (bit 5) to 1
  //Disable TXCIE0 (bit 6)
  //UCSZ02 must be 0 for 8-bit data size
  //RXB80 and TXB80 are 00 since we are not using 9-bit data size
  UCSR0B = 0b10111000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
int readSerial(char *buffer)
{
  int count = 0;

  TBufferResult result;

  do
  {
    result = readBuffer(&_recvBuffer, &buffer[count]);

    if (result == BUFFER_OK)
      count++;
  } while (result == BUFFER_OK);

  return count;
}

// Write to the serial port.
void writeSerial(const char *buffer, int len)
{
  TBufferResult result = BUFFER_OK;

  int i;

  for (i = 1; i < PACKET_SIZE && result == BUFFER_OK; i++)
  {
    result = writeBuffer(&_xmitBuffer, buffer[i]);
  }

  //to start the proverbial ball rolling
  UDR0 = buffer[0];

  //Enable the UDRE interrupt. The enable bit is bit 5 of UCSR0B.
  UCSR0B |= 0b00100000;
}

/*
   Alex's motor drivers.

*/
void setupMotors()
{
  DDRD |= (LF | LR); // set them
  DDRB |= (RR | RF); // as output

}

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

/*  TCCR0A = 0b00000011; //fast PWM 64 Prescalar
  TCCR1A = 0b00000001; //8bit fast PWM
  TCCR0B = 0b00000011; //fast PWM 64 Prescalar
  TCCR1B = 0b00001011;
  TIMSK0 = 0b110;
  TIMSK1 = 0b110;*/
}

/*
   Alex's Power Saving Function
*/
void WDT_off(void) { /* Global interrupt should be turned OFF here if not already done so */

  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1 << WDRF);

  /* Write logical one to WDCE and WDE */ /* Keep old prescaler setting to prevent unintentional time-out */
  WDTCSR |= (1 << WDCE) | (1 << WDE);

  /* Turn off WDT */
  WDTCSR = 0x00;

  /* Global interrupt should be turned ON here if subsequent operations after calling this function do not require
     turning off global interrupt */
}


void setupPowerSaving() {
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

  TCCR0A |= 0b00100000; //set OCR0B LR
  TCCR1A |= 0b00100000; //set OCR1B RR
}

// compute angle input into ticks for turning.
unsigned long computeDeltaTicks(float ang)
{
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

  targetTicks = rightForwardTicksTurns + deltaTicks;

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

  TCCR0A |= 0b10000000; //set OCR0A LF
  TCCR1A |= 0b00100000; //set OCR1B RR
}

// Stop Alex.
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
  setupEINT(); // interrupt for external interrupt

  setupSerial();
  // initialise the _recvBuffer & _xmitBuffer.
  initBuffer(&_recvBuffer, PACKET_SIZE);
  initBuffer(&_xmitBuffer, PACKET_SIZE);
  startSerial();

  // enable the respective ports of motor to be an output
  setupMotors();
  startMotors();

  // Set up Timer Interrupt for PWM of motor.
  setupTimer();
  enablePullups();

  //Powersaving settings
  initializeState();
  setupPowerSaving();
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
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK)
    handlePacket(&recvPacket);

  else if (result == PACKET_INCOMPLETE && dir == STOP) {
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
      if (rightForwardTicksTurns >= targetTicks) {
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

