#include "stdio.h" /* sprintf, sscanf */
#include "string.h" /* memset, strlen */

#include "Arduino.h"

#include "robot.h"

#define NUM_ARMS 3 /* The number of arms for the robot */
#define BUF_SIZE 32 /* Size of the serial input buffer, shouldn't be bigger imo */
#define TIMES 300 /* The number of loops the moveToward function will go through */

const double arm_lengths[NUM_ARMS] = { 1.20, 1.70, 3.1 }; /* make sure to change these to actual arm lengths in units e.g. centimeters */
double arm_angles[NUM_ARMS] = { 0.0, 0.0, 0.0 }; /* at setup, set the arms to specified angles */
Servo servos[NUM_ARMS]; /* Servo motor objects */
const uint8_t arm_pins[NUM_ARMS] = { 1, 2, 3 }; /* Please change these. These should preferably be PWM pins. */
const int servo_min[NUM_ARMS] = { 544, 544, 544 }; /* The PWM of each servo's "0 degree" angle */
const int servo_max[NUM_ARMS] = { 2400, 2400, 2400 }; /* The PWM of each servo's "180 degree" angle */

char buffer[BUF_SIZE]; /* buffer for serial messages from computer (maybe from Pi?) */
int buf_len = 0;
int sent_string = 0;

/*
 * Format for sending a command is "%f %f"
 * where those are the position of the object to grab
 * see "man printf" for more details
 */

/* simple radian to degree function */
inline int rad_to_deg(double rad)
{
	return (int)(rad * 360.0 / TAU);
}
/* every time a command is issued, this is called for every servo */
void moveArm(int arm, double delta_angle)
{
	servos[arm].write(rad_to_deg(arm_angles[arm]));
}
/* mandatory function */
void setup(void)
{
	int i;
	Serial.begin(9600);
	memset(buffer, '\0', BUF_SIZE);
	for (i = 0; i < NUM_ARMS; i++) {
		servos[i].attach(arm_pins[i], servo_min[i], servo_max[i]);
		servos[i].write(rad_to_deg(arm_angles[i]));
	}
}
/* mandatory function */
void loop(void)
{
	double gx, gy, error;
	if (!sent_string)
		return;
	sent_string = buf_len = 0;
       	sscanf(buffer, "%f %f", &gx, &gy);
       	error = moveTowards(NUM_ARMS, arm_lengths, arm_angles, TIMES, gx, gy, moveArm);
       	sprintf(buffer, "Moved arm to %f away from (%f, %f)", error, gx, gy);
	Serial.write(buffer, strlen(buffer)+1);
}
/*
 * Apparently this function is automatically called when a letter is typed
 * into the serial bus, after loop().
 */
void serialEvent(void)
{
	char c;
	while (Serial.available() && buf_len < BUF_SIZE) {
		c = (char)Serial.read();
		if (c == '\n') {
			sent_string = 1;
			return;
		}
		buffer[buf_len++] = c;
	}
}
