#ifndef ROBOT_H
#define ROBOT_H

#define TAU 2*M_PI

#ifdef __cplusplus 
extern "C" {		//if you accidentally use a c++ compiler
#endif
typedef struct
{
	double m[3][3];
} mat3_t,*mat3;
typedef struct
{
	double x,y;
} vec2_t, *vec2;


//iterates adjustion until error is sufficiently small
int calculate_position(int size, double *joints, double *angles, double error, double goal_x, double goal_y);
//iterates "iterations" times, returns the error
double calculate_position_times(int size, double *joints, double *angles, int iterations, double goal_x, double goal_y);

//adjusts one angle at a time to get closer and closer to the target
mat3_t adjust(int size, double *joints, double *angles, mat3_t tool, vec2_t goal);
//combine the rotation and translation matrices
mat3_t calc_joint(double joint, double angle);
//compute the end effector
mat3_t calc_end(int size, double *joints, double *angles);
//angle is a double,
//prev is a matrix
//tool is matrix (of the end effector)
//goal is a vector of the goal
double step(double angle, mat3_t prev, mat3_t tool, vec2_t goal);

//the identity matrix
mat3_t mat3_identity();
//print a matrix
void mat3_print(mat3 mat);
//get a translation matrix from a length
mat3_t mat3_translate(double length);
//get a rotation matrix from an angle
mat3_t mat3_rotate(double angle);
//multiply two 3x3 matrices
mat3_t mat3_multiply(mat3_t a, mat3_t b);
//multiply the [3x3] by [3x1] matrix (a column which is [0,0,1])
vec2_t mat3_getPosition(mat3_t m);
//an inverse of a 3x3 matrix
mat3_t mat3_inverse(mat3_t m);


#ifdef __cplusplus
}
#endif
#endif
