/* This firmware supports as many servos as possible using the Servo" library
* included in Arduino 0012
*
* This example code is in the public domain.
*/

#include <Servo.h>
#include <Firmata.h>

Servo servo9;
Servo servo10;

void analogWriteCallback(byte pin, int value)
{
if(pin == 9)
servo9.write(value);
if(pin == 10)
servo10.write(value);
}

void setup()
{
Firmata.setFirmwareVersion(0, 2);
Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);

servo9.attach(9);
servo10.attach(10);

Firmata.begin(57600);
}

void loop()
{
while(Firmata.available())
Firmata.processInput();
}
