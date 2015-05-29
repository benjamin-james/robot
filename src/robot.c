#include "robot.h"

#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h" /* memcpy */

/*
 * does almost the same thing as calculate_position_times
 * but calls move at the end for easy application
 * move has two parameters:	(int) 		the index of the angle to be changed
 *				(double)	the change in angle measure in radians
 */
double moveTowards(int size, double *joints, double *angles, int times, double goal_x, double goal_y, void (*move)(int, double))
{
	double *prev, result, d;
	int i;
	if (!joints || !angles || size < 1 || !move) {
		printf("uh oh");
		return -1;
	}
	prev = malloc(sizeof(double)*size);
	memcpy(prev, angles, size);
	result = calculate_position_times(size, joints, angles, times, goal_x, goal_y);
	for (i = 0; result >= 0.0 && i < size; i++) {
		d = angles[i]-prev[i];
		move(i, d);
	}
	free(prev);
	return result;
}
/*
 * The meat of this file
 * it calculates the angle to meet the position of the goal
 * angle is a double,
 * prev is a matrix
 * tool is matrix (of the end effector)
 * goal is a vector of the goal
 */
void step(double *angle, mat3_t prev, mat3_t tool, vec2_t goal)
{
	vec2_t prevP, toolP, a, b;
	prevP = mat3_getPosition(prev);
	toolP = mat3_getPosition(tool);
	a.x = toolP.x - prevP.x;
	a.y = toolP.y - prevP.y;
	b.x = goal.x - prevP.x;
	b.y = goal.y - prevP.y;
	*angle += atan2(b.y, b.x) - atan2(a.y, a.x);
}

/*
 * iterates "iterations" times, returns error
 */
double calculate_position_times(int size, double *joints, double *angles, int iterations, double goal_x, double goal_y)
{
	mat3_t tool;
	int i;
	double err = 0;
	vec2_t toolP, goal;
	if (!joints || !angles)
		return -1;
	tool = calc_end(size, joints, angles);
	goal.x = goal_x;
	goal.y = goal_y;
	for (i = 0; i < iterations; i++)
		tool = adjust(size, joints, angles, tool, goal);/* the computation */
	toolP = mat3_getPosition(tool);
	err = (toolP.x - goal_x)*(toolP.x - goal_x) + (toolP.y - goal_y)*(toolP.y - goal_y);
	return sqrt(err);
}

/*
 * iterates adjustion until error is sufficiently small
 */
int calculate_position(int size, double *joints, double *angles, double error, double goal_x, double goal_y)
{
	mat3_t tool;
	vec2_t goal, toolP;
	int i;
	double err;
	if (!joints || !angles)
		return -1;
	error *= error; /* error is squared */
	tool = calc_end(size, joints, angles);
	goal.x = goal_x;
	goal.y = goal_y;
	for (err = 0.0, i = 0; i < size; i++)
		err += joints[i];
	if (err * err > goal_x * goal_x + goal_y * goal_y)
		return -1;
	err = error+1;
	for (i = 0; err >= error; i++) {
		tool = adjust(size, joints, angles, tool, goal);
		toolP = mat3_getPosition(tool);
		err = (toolP.x - goal_x)*(toolP.x - goal_x) + (toolP.y - goal_y)*(toolP.y - goal_y);
	}
	return i;
}

/*
 * adjusts one angle at a time to get closer and closer to the target
 */
mat3_t adjust(int size, double *joints, double *angles, mat3_t tool, vec2_t goal)
{
	mat3_t next, prev = mat3_identity();
	int i;
	for (i = 0; i < size; i++) {
		next = mat3_multiply(prev, calc_joint(joints[i], angles[i]));
		step(angles+i, prev, tool, goal);
		prev = mat3_multiply(prev, calc_joint(joints[i], angles[i]));
		tool = mat3_multiply(mat3_multiply(prev, mat3_inverse(next)), tool);
	}
	return prev;
}

/*
 * combine the rotation and translation matrices
 */
mat3_t calc_joint(double joint, double angle)
{
	return mat3_multiply(mat3_rotate(angle), mat3_translate(joint));
}

/*
 * compute the end effector position
 */
mat3_t calc_end(int size, double *joints, double *angles)
{
	mat3_t joint, mat = mat3_identity();
	int i;
	for (i = 0; i < size; i++) {
	        joint = calc_joint(joints[i], angles[i]);
		mat = mat3_multiply(mat, joint);
	}
	return mat;
}

/*
 * get a translation matrix from a length
 */
mat3_t mat3_translate(double length)
{
	mat3_t m = mat3_identity();
	m.m[0][2] = length;
	return m;
}

/*
 * get a rotation matrix from an angle
 */
mat3_t mat3_rotate(double angle)
{
	mat3_t m = mat3_identity();
	m.m[0][0] = cos(angle);
	m.m[1][0] = sin(angle);
	m.m[0][1] = -m.m[1][0];
	m.m[1][1] = m.m[0][0];
	return m;
}

/*
 * multiply two 3x3 matrices
 */
mat3_t mat3_multiply(mat3_t a, mat3_t b)
{
	mat3_t c = mat3_identity();
	int i, j, k;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			c.m[i][j] = 0;
			for (k = 0; k < 3; k++) {
				c.m[i][j] += a.m[i][k] * b.m[k][j];
			}
		}
	}
	return c;
}

/*
 * multiply the [3x3] by [3x1] matrix (a column which is [0,0,1])
 */
vec2_t mat3_getPosition(mat3_t m)
{
	vec2_t v;
	v.x = m.m[0][2];
	v.y = m.m[1][2];
	return v;
}

/*
 * the identity matrix
 */
mat3_t mat3_identity()
{
	mat3_t m;
	m.m[0][0] = 1;
	m.m[0][1] = 0;
	m.m[0][2] = 0;
	m.m[1][0] = 0;
	m.m[1][1] = 1;
	m.m[1][2] = 0;
	m.m[2][0] = 0;
	m.m[2][1] = 0;
	m.m[2][2] = 1;
	return m;
}

/*
 * print a matrix
 */
void mat3_print(mat3 mat)
{
	int i,j;
	if (!mat)
		return;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			printf(" %f", mat->m[i][j]);
		}
		puts(" ");
	}
}

/*
 * takes the determinant of a 2x2 matrix
 */
double mat2_det(double a[][2])
{
	return a[0][0]*a[1][1]-a[0][1]*a[1][0];
}

/*
 * takes the determinant of the minor matrix
 * specified by skipping row r and column c
 */
double mat2_det_mat3(mat3 m, int r, int c)
{
	int a, b, i, j;
	double minor[2][2];
	for (a = 0, i = 0; i < 2; a++, i++) {
		if (i == r)
			a++;
		for (b = 0, j = 0; j < 2; b++, j++) {
			if (j == c)
				b++;
			minor[i][j] = m->m[a][b];
		}
	}
	return mat2_det(minor);
}

/*
 * determinant of a 3x3 matrix
 */
double mat3_det(mat3 m)
{
	double ret[2][2];
	double total = 0;
	int sign = 1;
	int a, b, i, j, k;
	for (i = 0; i < 3; i++) {
		a = b = 0;
		for (j = 1; j < 3; j++) { /*Does not start from 0th row*/
			for (k = 0; k < 3; k++) {
				if (k == i)
					continue;
				ret[a][b++] = m->m[j][k];
			}
			a++;
			b = 0;
		}
		total += sign * m->m[0][i] * mat2_det(ret);
		sign *= -1;
	}
	return total;
}

/*
 * Recursive integer exponentiation
 */
int powerInt(int a, int b)
{
	if (b < 1)
		return 1;
	return a * powerInt(a, b-1);
}

/*
 * takes the inverse of a 3x3 matrix
 */
mat3_t mat3_inverse(mat3_t m)
{
	double det = mat3_det(&m);
	int i, j;
	mat3_t a;
	if (det == 0)
		exit(-1);
	a = mat3_identity();
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			a.m[j][i] = powerInt(-1, i+j) * mat2_det_mat3(&m, i, j) / det;
	return a;
}

/*
 * Gets the angle by using atan2
 */
double mat3_getAngle(mat3_t m)
{
	return atan2(m.m[1][0], m.m[0][0]);
}
