#include "math.h"
#include "robot.h"
#include "stdio.h"
#include "stdlib.h"
#include "sys/time.h"
#include "time.h"

#define TIMES 500

int get_info(int argc, char **argv, double *arms, double *angles, double *gx, double *gy);
void moveArm(int arm, double angle);
void output(mat3_t m, double error, struct timeval tv, clock_t clk);
void evaluate(int size, double *arms, double *angles, double gx, double gy);
double toDegrees(double d);
double toRadians(double d);

int main(int argc, char **argv)
{
	double *arms, *angles, gx, gy;
	int size = (argc - 3) / 2;
	arms = malloc(sizeof(double) * size);
	angles = malloc(sizeof(double) * size);
	if (get_info(argc, argv, arms, angles, &gx, &gy) < 0)
		return -1;
	evaluate(size, arms, angles, gx, gy);
	free(arms);
	free(angles);
	return 0;
}

void moveArm(int arm, double angle)
{
	angle = toDegrees(angle);
	while (angle > 180.0)
		angle -= 360.0;
	while (angle <= -180.0)
		angle += 360;
	printf("Moved arm %d by %f degrees\n", arm, angle);
}

int get_info(int argc, char **argv, double *arms, double *angles, double *gx, double *gy)
{
	int size, i, count;
       	if(argc % 2 - 1 || argc < 3) {
		printf("Usage: %s length_0 angle_0 length_1 angle_1 ... length_N angle_N goal_x goal_y\n\twhere angles are in degrees\n", *argv);
		return -1;
	}
	size = (argc - 3) / 2;
	for (i = 1, count = 0; count < size; count++) {
		arms[count] = atof(argv[i++]);
		angles[count] = toRadians(atof(argv[i++]));
	}
	*gx = atof(argv[i++]);
	*gy = atof(argv[i++]);
	return 0;
}
void output(mat3_t m, double error, struct timeval tv, clock_t clk)
{
	double ctm = (double)clk / CLOCKS_PER_SEC;
	double rtm = (double)tv.tv_sec + tv.tv_usec / 1000000.0;
	vec2_t v = mat3_getPosition(m);
	printf("\n");
	printf("Error\t\tCPU time\t\tReal time\t\tEnd effector position\n");
	printf("-----\t\t--------\t\t---------\t\t---------------------\n");
	printf("%f\t%f\t\t%f\t\t(%f, %f)\n", error, ctm, rtm, v.x, v.y);
}
void evaluate(int size, double *arms, double *angles, double gx, double gy)
{
	struct timeval start, end;
	clock_t clk;
	double error;
	puts("");
	gettimeofday(&start, NULL);
	clk = clock();
	error = moveTowards(size, arms, angles, TIMES, gx, gy, moveArm);
	gettimeofday(&end, NULL);
	clk = clock() - clk;
	if (error < 0) {
		printf("Unable to reach point (%f, %f).\n", gx, gy);
		exit(1);
	}
	end.tv_sec -= start.tv_sec;
	end.tv_usec -= start.tv_usec;
	output(calc_end(size, arms, angles), error, end, clk);
}
double toDegrees(double d)
{
	while (d > 180.0)
		d -= 360.0;
	while (d <= -180.0)
		d += 360.0;
	d *= 180.0 / M_PI;
	return d;
}
double toRadians(double d)
{
	d *= M_PI / 180.0;
	while (d > 180.0)
		d -= 360.0;
	while (d <= -180.0)
		d += 360.0;
	return d;
}
