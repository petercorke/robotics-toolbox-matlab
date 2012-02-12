/**
 * \file vmath.h
 * \author Peter Corke
 * \brief Simple vector/matrix maths library.
 *
 * \note All vectors and matrices are passed by reference.
 */

/*
 * Copyright (C) 1999-2008, by Peter I. Corke
 *
 * This file is part of The Robotics Toolbox for Matlab (RTB).
 * 
 * RTB is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RTB is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Leser General Public License
 * along with RTB.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#ifndef	_vmath_h_
#define	_vmath_h_
typedef struct vector {
	double	x, y, z;
} Vect;

typedef struct matrix {
	Vect	n, o, a;
} Rot;

typedef struct homogeneous_matrix {
	Vect	n, o, a, p;
} Transform;
void	vect_cross (Vect *r, Vect *a, Vect *b);
double	vect_dot (Vect *a, Vect *b);
void	vect_add (Vect *r, Vect *a, Vect *b);
void	scal_mult (Vect *r, Vect *a, double s);
void	rot_vect_mult (Vect *r, Rot *m, Vect *v);
void	rot_trans_vect_mult (Vect *r, Rot *m, Vect *v);
void	mat_vect_mult (Vect *r, double *m, Vect *v);
void	rot_print(char *s, Rot *m);
void	vect_print(char *s, Vect *v);
#endif
