#include "robot.h"
#include "stdio.h"
#include "stdlib.h"
#include "time.h"

#define TIMES 500

inline double toDegrees(double angle)
{
	return angle * 57.2957795131; /* == angle * 360 / tau */
}
void moveArm(int arm, double angle)
{
  printf("Moved arm %d by %f degrees\n", arm, toDegrees(angle));
}
int main(int argc, char **argv)
{
  int i, count;
  if(argc % 2 - 1 || argc < 3) return -1;
  int size = (argc - 3) / 2;
  double *arms = malloc(sizeof(double) * size);
  double *angles = malloc(sizeof(double) * size);
  for (i = 1, count = 0; count < size; count++) {
    arms[count] = atof(argv[i++]);
    angles[count] = atof(argv[i++]);
  }
  double gx = atof(argv[i++]);
  double gy = atof(argv[i++]);
  clock_t clk = clock();
  double error = moveTowards(size,arms,angles,TIMES,gx,gy,moveArm);
  clk = clock() - clk;
  printf("\nerror: %f\ntime used: %f seconds\n", error, (double)(clk) / CLOCKS_PER_SEC);
  mat3_t m = calc_end(size,arms,angles);
  mat3_print(&m);
  free(arms);
  free(angles);
  return 0;
}
