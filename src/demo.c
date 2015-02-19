#include "robot.h"
#include "stdio.h"
#define NUM_ARMS 3
int main(int argc, char **argv)
{
	double arms[NUM_ARMS] = {3,7,4};
	double angles[NUM_ARMS] = {1,0.3,1};
	double precision = 0.000001;
	double error = 0;
	int i;
	do
	{
		printf("%f\n\narm\t\tangle\n",error);
		for(i = 0; i < NUM_ARMS; i++)
		{
			printf("%f\t%f\n",arms[i],angles[i]);
		}
		
	}while((error = calculate_position_times(NUM_ARMS,arms,angles,1,8,8)) > precision);
	printf("%f\n\n",error);
	mat3_t end = calc_end(NUM_ARMS,arms,angles);
	mat3_print(&end);
	return 0;
}
