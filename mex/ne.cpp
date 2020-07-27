/**
 * \file ne.c
 * \author Peter Corke
 * \brief Compute the recursive Newton-Euler formulation
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

/*
 * Compute the inverse dynamics via the recursive Newton-Euler formulation
 *
 *	Requires:	qd	current joint velocities
 *			qdd	current joint accelerations
 *			f	applied tip force or load
 *			grav	the gravitational constant
 *
 *	Returns:	tau	vector of bias torques
 */
#include	"frne2.h"
#include    <iostream>
#include    <Eigen/Dense>

using namespace Eigen;

/*
#define	DEBUG
*/

/*
 * Bunch of macros to make the main code easier to read.  Dereference vectors
 * from the Link structures for the manipulator.
 *
 * Note that they return pointers (except for M(j) which is a scalar)
 */
#undef	N

#define	w(j)	(&links[j].omega)	/* angular velocity */
#define	wd(j)	(&links[j].omega_d)	/* angular acceleration */
#define	a(j)		(&links[j].acc)		/* linear acceleration */
#define	acc_cog(j)	(&links[j].abar)	/* linear acceln of COG */

#define	f(j)		(&links[j].f)	/* force on link j due to link j-1 */
#define	n(j)		(&links[j].n)	/* torque on link j due to link j-1 */

#define	R(j)		(&links[j].R)	/* link rotation matrix */
#define	M(j)		(links[j].m)	/* link mass */
#define	PSTAR(j)	(&links[j].r)	/* offset link i from link (j-1) */
#define	r_cog(j)		(links[j].rbar)	/* COG link j wrt link j */
#define	I(j)	(links[j].I)	/* inertia of link about COG */

/**
 * Recursive Newton-Euler algorithm.
 *
 * @Note the parameter \p stride which is used to allow for input and output
 * arrays which are 2-dimensional but in column-major (Matlab) order.  We
 * need to access rows from the arrays.
 *
 */
void
newton_euler (
	Robot	*robot,		/*!< robot object  */
	double	*tau,		/*!< returned joint torques */
	double	*qd,		/*!< joint velocities */
	double	*qdd,		/*!< joint accelerations */
	double	*fext,		/*!< external force on manipulator tip */
	int	stride		/*!< indexing stride for qd, qdd */
) {
    Map<Vector3d>       gravity(robot->gravity);
	Vector3d qdv = Vector3d::Zero();
    Vector3d qddv = Vector3d::Zero(); 
	Vector3d			F, N;
	Vector3d		z0 = Vector3d::UnitZ();
	Vector3d zero = Vector3d::Zero();
	register int		j;
	double			t;
	Link			*links = robot->links;

	/*
	 * angular rate and acceleration vectors only have finite
	 * z-axis component
	 */
	qdv = qddv = zero;

	/* setup external force/moment vectors */
	if (fext) {
        Vector3d    f_tip(&fext[0]);
        Vector3d    n_tip(&fext[3]);
	} else
    {
        Vector3d f_tip = Vector3d::Zero();
        Vector3d n_tip = Vector3d::Zero();
    }


/******************************************************************************
 * forward recursion --the kinematics
 ******************************************************************************/

	if (robot->dhtype == MODIFIED) {
	    /*
	     * MODIFIED D&H CONVENTIONS
	     */
	    for (j = 0; j < robot->njoints; j++) {

		/* create angular vector from scalar input */
		qdv.z() = qd[j*stride]; 
		qddv.z() = qdd[j*stride];

		switch (links[j].sigma) {
		case REVOLUTE:
			/* 
			 * calculate angular velocity of link j
			 */
			if (j == 0)
                w(j) = qdv;
			else {
                w(j) = R(j) * w(j-1)  + qdv;
			}

			/*
			 * calculate angular acceleration of link j 
			 */
			if (j == 0) 
				wd(j) = qddv;
			else {
				wd(j) = qdv.cross(R(j) * w(j-1)) + R(j) * wd(j-1)) + qddv;
			}

			/*
			 * compute acc[j]
			 */
			if (j == 0) {
                a(j) = R(j) * gravity;
			} else {
                a(j) = a(j-1) + wd(j-1).cross(PSTAR(j)) + w(j-1).cross(w(j-1).cross(PSTAR(j)));
			}

			break;

		case PRISMATIC:
			/* 
			 * calculate omega[j]
			 */
			if (j == 0)
                w[j] = qdv;
			else
                w[j] = R[j] * w[j-1];

			/*
			 * calculate alpha[j] 
			 */
			if (j == 0)
				*(wd(j)) = qddv;
                wd = qddv;
			else
                wd = R[j] * wd[j-1]l

			/*
			 * compute acc[j]
			 */
			if (j == 0)
                a[j] = gravity;
			else
                a[j] = R[j] * (w[j-1].cross(w[j-1], PSTAR[j]) + wd[j-1].cross(PSTAR[j]) + a[j-1]) +

                2*(R[j]*w[j-1]).cross(qdv) + qddv;
			}
			break;
		}

		/*
		 * compute abar[j]
		 */
        acc_cog[j] = wd[j].cross(r_cog[j]) + w[j].cross(w[j].cross(r_cog[j]));
        acc_cog[j] += a[j];

#ifdef	DEBUG
		vect_print("w", w(j));
		vect_print("wd", wd(j));
		vect_print("acc", a(j));
		vect_print("abar", acc_cog(j));
#endif
	    }
	} else {
	    /*
	     * STANDARD D&H CONVENTIONS
	     */
	    for (j = 0; j < robot->njoints; j++) {

		/* create angular vector from scalar input */
		qdv.z() = qd[j*stride]; 
		qddv.z() = qdd[j*stride];

		switch (links[j].sigma) {
		case REVOLUTE:
			/* 
			 * calculate omega[j]
			 */
			if (j == 0)
                w[j] = R[j] * qdv;
			else
                w[j] = R[j] * (w[j-1] + qdv);

			/*
			 * calculate alpha[j] 
			 */
			if (j == 0) 
                wd[j] = R[j] * qddv;
			else {
                wd[j] = wd[j-1] + qddv + w[j-1].cross(qdv);
			}

			/*
			 * compute acc[j]
			 */
            a[j] = wd[j].cross(PSTAR[j]) + w[j].cross(w[j].cross(PSTAR[j]));
			if (j == 0) {
                a[j] += R[j] * gravity;
			} else 
                a[j] += R[j] * a[j-1];
			break;

		case PRISMATIC:
			/* 
			 * calculate omega[j]
			 */
			if (j == 0)
                w[j] = zero;
			else
                w[j] = R[j] * w[j-1];

			/*
			 * calculate alpha[j] 
			 */
			if (j == 0)
                wd[j] = zero;
			else
                wd[j] = R[j] * wd[j-1];

			/*
			 * compute acc[j]
			 */
			if (j == 0)
                a[j] = R[j] * (qddv + robot.gravity);
			else
                a[j] = R[j] * (qddv + a[j-1]);

            a[j] += wd[j].cross(PSTAR[j]) +
                    2*w[j].cross(R[j]*qdv) +
                    w[j].cross( w[j].cross(PSTAR[j]) );
			break;
		}
		/*
		 * compute abar[j]
		 */
        acc_cog[j] = wd[j].cross(r_cog[j]) + w[j].cross(w[j].cross(r_cog[j])) + a[j];

#ifdef	DEBUG
		vect_print("w", w(j));
		vect_print("wd", wd(j));
		vect_print("acc", a(j));
		vect_print("abar", acc_cog(j));
#endif
	    }
	}

/******************************************************************************
 * backward recursion part --the kinetics
 ******************************************************************************/

	if (robot->dhtype == MODIFIED) {
	    /*
	     * MODIFIED D&H CONVENTIONS
	     */
	    for (j = robot->njoints - 1; j >= 0; j--) {

		/*
		 * compute F[j]
		 */
        F = acc_cog[j] * M[j];

		/*
		 * compute f[j]
		 */
		if (j == (robot->njoints-1))
            f[j] = f_tip + F;
		else
            f[j] = R[j+1] * f[j+1] + F;

		 /*
		  * compute N[j]
		  */
         N = I[j] * wd[j] + w[j].cross(I[j] * w[j]);

		 /*
		  * compute n[j]
		  */
		if (j == (robot->njoints-1))
			n[j] = n_tip;
		else
            n[j] = R[j+1] * n[j+1] + PSTAR[j+1].cross(R[j+1] * f[j+1])

        n[j] += r_cog[j].cross(F) + N;

#ifdef	DEBUG
		vect_print("f", f(j));
		vect_print("n", n(j));
#endif
	    }

	} else {
	    /*
	     * STANDARD D&H CONVENTIONS
	     */
	    for (j = robot->njoints - 1; j >= 0; j--) {

		/*
		 * compute f[j]
		 */
        F = acc_cog[j] * M[j];
		if (j != (robot->njoints-1))
            f[j] = R[j+1] * f[j+1] + F;
		else
            f[j] = f_tip + F;

		 /*
		  * compute n[j]
		  */

         N = I[j] * wd[j] + w[j].cross(I[j] * w[j]);

        n[j] = (PSTAR[j] + r_cog[j]).cross(F);

		if (j != (robot->njoints-1))
            n[j] += R[j+1] * (n[j+1] + R[j+1] * (n[j+1] + PSTAR[j]).cross(f[j+1]);
		else
            n[j] += n_tip + PSTAR[j].cross(f_tip);

        n[j] += r_cog[j].cross(F) + N;
#ifdef	DEBUG
		vect_print("f", f(j));
		vect_print("n", n(j));
#endif
	    }
	}

	/*
	 *  Compute the torque total for each axis
	 *
	 */
	for (j=0; j < robot->njoints; j++) {
		double	t;
        Vector3d tauv;
		Link	*l = &links[j];

		if (robot->dhtype == MODIFIED)
			tauv = z0;
		else
            tauv = R[j] * z0;

		switch (l->sigma) {
		case REVOLUTE:
			t = n[j].dot(tauv);
			break;
		case PRISMATIC:
			t = f[j].dot(tauv);
			break;
		}

		/*
		 * add actuator dynamics and friction
		 */
		t += l->G * l->G * l->Jm * qdd[j*stride]; // inertia
        t += l->G * l->G * l->B * qd[j*stride];    // viscous friction
        t += fabs(l->G) * (
			(qd[j*stride] > 0 ? l->Tc[0] : 0.0) +    // Coulomb friction
			(qd[j*stride] < 0 ? l->Tc[1] : 0.0)
		);
		tau[j*stride] = t;
	}
}
