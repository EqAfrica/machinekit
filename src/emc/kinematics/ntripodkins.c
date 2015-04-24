<<<<<<< HEAD
/********************************************************************
 Last change:                   2015.22.4 1:00AM Taiwan Time
********************************************************************/
#include "kinematics.h"             /* these decls */
#include "rtapi_math.h"

/* ident tag */
#ifndef __GNUC__
#ifndef __attribute__
#define __attribute__(x)
#endif
#endif

 /* robot geometry  

e = side length of end effector triangle, middle arm - "re"
f = side length of base triangle, middle drive joints - "rf"
re = length of end effector arm
rf = length of drive arm   

sample:
e = 115.0;
f = 457.3;
re = 232.0;
rf = 112.0;  */


#include "hal.h"

struct haldata {
    hal_float_t *e, *f, *re, *rf;
} *haldata = 0;

#define delta_e (*(haldata->e))    
#define delta_f (*(haldata->f))    
#define delta_re (*(haldata->re))  
#define delta_rf (*(haldata->rf)) 
#define VTVERSION VTKINEMATICS_VERSION1

 /* trigonometric constants */
 const double sqrt3 = 1.7320508075688772935274463415059;   /* sqrt(3.0);*/
#ifndef PI
#define PI 3.14159265358979323846
#endif
 const double sin120 = 0.86602540378443864676372317075294; /* (sqrt3)/2;*/   
 const double cos120 = -0.5;        
 const double tan60 = 1.7320508075688772935274463415059;   /* sqrt3;*/
 const double sin30 = 0.5;
 const double tan30 = 0.57735026918962576450914878050196;  /* 1/(sqrt3);*/

  
 /* forward kinematics: (joints[0], joints[1], joints[2]) -> (pos->tran.x, pos->tran.y, pos->tran.z)
 // returned status: 0=OK, -1=non-existing position*/

int kinematicsForward( const double *joints,
		      EmcPose *pos,
		      const KINEMATICS_FORWARD_FLAGS *fflags,
		      KINEMATICS_INVERSE_FLAGS *iflags)

 {

     
     double t = (delta_f-delta_e)*tan30/2;
     
     //float dtr = pi/(float)180.0;
 
     double theta1 = joints[0] * PI/180;
     double theta2 = joints[1] *  PI/180;
     double theta3 = joints[2] * PI/180;
 
     double y1 = -(t + delta_rf*cos(theta1));
     double z1 = -delta_rf*sin(theta1);
 
     double y2 = (t + delta_rf*cos(theta2))*sin30;
     double x2 = y2*tan60;
     double z2 = -delta_rf*sin(theta2);
 
     double y3 = (t + delta_rf*cos(theta3))*sin30;
     double x3 = -y3*tan60;
     double z3 = -delta_rf*sin(theta3);
 
     double dnm = (y2-y1)*x3-(y3-y1)*x2;
 
     double w1 = y1*y1 + z1*z1;
     double w2 = x2*x2 + y2*y2 + z2*z2;
     double w3 = x3*x3 + y3*y3 + z3*z3;
     
     /* x = (a1*z + b1)/dnm*/
     double a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     double b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
     /* y = (a2*z + b2)/dnm;*/
     double a2 = -(z2-z1)*x3+(z3-z1)*x2;
     double b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
 
     /* a*z^2 + b*z + c = 0*/
     double a = a1*a1 + a2*a2 + dnm*dnm;
     double b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     double c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - delta_re*delta_re);
  
     /* discriminant*/
     double d = b*b - (double)4.0*a*c;
     if (d < 0) return -1; /* non-existing point*/
 
     pos->tran.z = -(double)0.5*(b+sqrt(d))/a;
     pos->tran.x = (a1*pos->tran.z + b1)/dnm;
     pos->tran.y = (a2*pos->tran.z + b2)/dnm;

     // STEPPER motor joints
     pos->a = joints[3];
     pos->b = joints[4];
     // pos->c = 0.0;	

     return 0;

 }


 
 /* inverse kinematics
    helper functions, calculates angle theta1 (for YZ-pane)*/
 int delta_calcAngleYZ( double x0, double y0, double z0, double *theta )
  {
   
     double y1 = -0.5 * 0.57735 * delta_f; // f/2 * tg 30
     y0 -= 0.5 * 0.57735    * delta_e;    // shift center to edge
     /* z = a + b*y*/
     double a = (x0*x0 + y0*y0 + z0*z0 +delta_rf*delta_rf - delta_re*delta_re - y1*y1)/(2*z0);
     double b = (y1-y0)/z0;
     /* discriminant*/
     double d = -(a+b*y1)*(a+b*y1)+delta_rf*(b*b*delta_rf+delta_rf); 
     if (d < 0) return -1; /* non-existing point*/
     double yj = (y1 - a*b - sqrt(d))/(b*b + 1); /* choosing outer point*/
     double zj = a + b*yj;
     *theta = 180/PI * atan2(-zj,(y1 - yj)) + ((yj>y1)?180.0:0.0);
     return 0;
 }
 
 /* inverse kinematics: (pos->tran.x, pos->tran.y, pos->tran.z) -> (joints[0], joints[1], joints[2])
    returned status: 0=OK, -1=non-existing position*/
int kinematicsInverse(const EmcPose *pos,
		      double *joints,
		      const KINEMATICS_INVERSE_FLAGS *iflags,
		      KINEMATICS_FORWARD_FLAGS *fflags)
  
 {

     double x0 = pos->tran.x;
     double y0 = pos->tran.y;
     double z0 = pos->tran.z;
     double theta1;
     double theta2;
     double theta3;
     
     theta1 = theta2 = theta3 = 0;
     int status = delta_calcAngleYZ(x0, y0, z0, &theta1);
     if (status == 0) joints[0] = theta1;
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, &theta2); /*rotate coords to +120 deg*/
     if (status == 0) joints[1] = theta2;
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, &theta3); /*rotate coords to -120 deg*/
     if (status == 0) joints[2] = theta3;
     
     return status;
 }



 /* implemented for these kinematics as giving joints preference */
int kinematicsHome(EmcPose * world,
		   double *joint,
		   KINEMATICS_FORWARD_FLAGS * fflags,
		   KINEMATICS_INVERSE_FLAGS * iflags)
{
    *fflags = 0;
    *iflags = 0;

    return kinematicsForward(joint, world, fflags, iflags);
}

KINEMATICS_TYPE kinematicsType()
{
    return KINEMATICS_BOTH;
}

/* I ommitted main function and directives
related to main function
RTAPI main is the only function implemented
 */


#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"



static vtkins_t vtk = {
    .kinematicsForward = kinematicsForward,
    .kinematicsInverse  = kinematicsInverse,
    // .kinematicsHome = kinematicsHome,
    .kinematicsType = kinematicsType
};

static int comp_id, vtable_id;
static const char *name = "ntripodkins";
MODULE_LICENSE("GPL");

int rtapi_app_main(void) {
    int res = 0;

    comp_id = hal_init("ntripodkins");
    if(comp_id < 0) return comp_id;


 vtable_id = hal_export_vtable(name, VTVERSION, &vtk, comp_id);
    if (vtable_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "%s: ERROR: hal_export_vtable(%s,%d,%p) failed: %d\n",
                        name, name, VTVERSION, &vtk, vtable_id );
        return -ENOENT;
    }


    haldata = hal_malloc(sizeof(struct haldata));
    if(!haldata) goto error;
	
    if((res = hal_pin_float_new("ntripodkins.e", HAL_IO, &(haldata->e), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("ntripodkins.f", HAL_IO, &(haldata->f), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("ntripodkins.re", HAL_IO, &(haldata->re), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("ntripodkins.rf", HAL_IO, &(haldata->rf), comp_id)) < 0) goto error;
    delta_e = delta_f = delta_re = delta_rf = 1.0;
    
    hal_ready(comp_id);
    return 0;

error:
    hal_exit(comp_id);
    return res;
}


                                                     
void rtapi_app_exit(void)
{
    hal_remove_vtable(vtable_id);
    hal_exit(comp_id);
}
=======
/********************************************************************
* Description: tripodkins.c
*   Kinematics for 3 axis Tripod machine
*
*   Derived from a work by Fred Proctor
*
* Author: 
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

/*
  These kinematics are for a tripod with point vertices.

  Vertices A, B, and C are the base, and vertex D is the controlled point.
  Three tripod strut lengths AD, BD, and CD are the joints that move
  point D around.

  Point A is the origin, with coordinates (0, 0, 0). Point B lies on the
  x axis, with coordinates (Bx, 0, 0). Point C lies in the xy plane, with
  coordinates (Cx, Cy, 0). Point D has coordinates (Dx, Dy, Dz).

  The controlled Cartesian values are Dx, Dy, and Dz. A frame attached to
  D, say with x parallel to AD and y in the plane ABD, would change its
  orientation as the strut lengths changed. The orientation of this frame
  relative to the world frame is not computed.

  With respect to the kinematics functions,

  pos->tran.x = Dx
  pos->tran.y = Dy
  pos->tran.z = Dz
  pos->a,b,c  = 0

  joints[0] = AD
  joints[1] = BD
  joints[2] = CD

  The inverse kinematics have no singularities. Any values for Dx, Dy, and
  Dz will yield numerical results. Of course, these may be beyond the
  strut length limits, but there are no singular effects like infinite speed.

  The forward kinematics has a singularity due to the triangle inequalities
  for triangles ABD, BCD, and CAD. When any of these approach the limit,
  Dz is zero and D lies in the base plane.

  The forward kinematics flags, referred to in kinematicsForward and
  set in kinematicsInverse, let the forward kinematics select between
  the positive and negative values of Dz for given strut values.
  Dz > 0 is "above", Dz < 0 is "below". Dz = 0 is the singularity.

  fflags == 0 selects Dz > 0,
  fflags != 0 selects Dz < 0.

  The inverse kinematics flags let the inverse kinematics select between
  multiple valid solutions of strut lengths for given Cartesian values
  for D. There are no multiple solutions: D constrains the strut lengths
  completely. So, the inverse flags are ignored.
 */

#include "kinematics.h"             /* these decls */
#include "rtapi_math.h"

/* ident tag */
#ifndef __GNUC__
#ifndef __attribute__
#define __attribute__(x)
#endif
#endif

#include "hal.h"
struct haldata {
    hal_float_t *bx, *cx, *cy;
} *haldata = 0;

#define Bx (*(haldata->bx))
#define Cx (*(haldata->cx))
#define Cy (*(haldata->cy))

#define sq(x) ((x)*(x))

/*
  forward kinematics takes three strut lengths and computes Dx, Dy, and Dz
  pos->tran.x,y,z, respectively. The forward flag is used to resolve
  D above/below the xy plane. The inverse flags are not set since there
  are no ambiguities going from world to joint coordinates.

  The forward kins are derived as follows:

  1. Let x, y, z be Dx, Dy, Dz to save pixels. Cartesian displacement from
  D to A, B, and C gives

  AD^2 = x^2 + y^2 + z^2
  BD^2 = (x - Bx)^2 + y^2 + z^2
  CD^2 = (x - Cx)^2 + (y - Cy)^2 + z^2

  This yields

  I.   P = x^2 + y^2 + z^2
  II.  Q = x^2 + y^2 + z^2 + sx
  III. R = x^2 + y^2 + z^2 + tx + uy

  Where

  P = AD^2,
  Q = BD^2 - Bx^2
  R = CD^2 - Cx^2 - Cy^2
  s = -2Bx
  t = -2Cx
  u = -2Cy

  II - I gives Q - P = sx, so x = (Q - P)/s, s != 0. The constraint on s
  means that Bx != 0, or points A and B can't be the same.

  III - II gives R - Q = (t - s)x + uy, so y = (R - Q - (t - s)x)/u, u != 0.
  The constraint on u means that Cy != 0, or points A B C can't be collinear.

  Substituting x, y into I gives z = sqrt(P - x^2 - y^2), which has two
  solutions. Positive means the tripod is above the xy plane, negative
  means below.
*/
int kinematicsForward(const double * joints,
                      EmcPose * pos,
                      const KINEMATICS_FORWARD_FLAGS * fflags,
                      KINEMATICS_INVERSE_FLAGS * iflags)
{
#define AD (joints[0])
#define BD (joints[1])
#define CD (joints[2])
#define Dx (pos->tran.x)
#define Dy (pos->tran.y)
#define Dz (pos->tran.z)
  double P, Q, R;
  double s, t, u;

  P = sq(AD);
  Q = sq(BD) - sq(Bx);
  R = sq(CD) - sq(Cx) - sq(Cy);
  s = -2.0 * Bx;
  t = -2.0 * Cx;
  u = -2.0 * Cy;

  if (s == 0.0) {
    /* points A and B coincident. Fix Bx, #defined up top. */
    return -1;
  }
  Dx = (Q - P) / s;

  if (u == 0.0) {
    /* points A B C are colinear. Fix Cy, #defined up top. */
    return -1;
  }
  Dy = (R - Q - (t - s) * Dx) / u;
  Dz = P - sq(Dx) - sq(Dy);
  if (Dz < 0.0) {
    /* triangle inequality violated */
    return -1;
  }
  Dz = sqrt(Dz);
  if (*fflags) {
    Dz = -Dz;
  }

  pos->a = joints[3];
  pos->b = joints[4];
  pos->c = 0.0;

  return 0;

#undef AD
#undef BD
#undef CD
#undef Dx
#undef Dy
#undef Dz
}

int kinematicsInverse(const EmcPose * pos,
                      double * joints,
                      const KINEMATICS_INVERSE_FLAGS * iflags,
                      KINEMATICS_FORWARD_FLAGS * fflags)
{
#define AD (joints[0])
#define BD (joints[1])
#define CD (joints[2])
#define Dx (pos->tran.x)
#define Dy (pos->tran.y)
#define Dz (pos->tran.z)

  AD = sqrt(sq(Dx) + sq(Dy) + sq(Dz));
  BD = sqrt(sq(Dx - Bx) + sq(Dy) + sq(Dz));
  CD = sqrt(sq(Dx - Cx) + sq(Dy - Cy) + sq(Dz));
  joints[3] = pos->a;
  joints[4] = pos->b;
    
  rtapi_print("AD(%f) Dx(%f) Dy(%f) Dz(%f)\n", AD, Dx, Dy, Dz);

  *fflags = 0;
  if (Dz < 0.0) {
    *fflags = 1;
  }

  return 0;

#undef AD
#undef BD
#undef CD
#undef Dx
#undef Dy
#undef Dz
}

KINEMATICS_TYPE kinematicsType()
{
  return KINEMATICS_BOTH;
}

#ifdef MAIN

#include <stdio.h>
#include <string.h>

/*
  Interactive testing of kins.

  Syntax: a.out <Bx> <Cx> <Cy>
*/
int main(int argc, char *argv[])
{
#ifndef BUFFERLEN
#define BUFFERLEN 256
#endif
  char buffer[BUFFERLEN];
  char cmd[BUFFERLEN];
  EmcPose pos, vel;
  double joints[3], jointvels[3];
  char inverse;
  char flags;
  KINEMATICS_FORWARD_FLAGS fflags;

  inverse = 0;			/* forwards, by default */
  flags = 0;			/* didn't provide flags */
  fflags = 0;			/* above xy plane, by default */
  if (argc != 4 ||
      1 != sscanf(argv[1], "%lf", &Bx) ||
      1 != sscanf(argv[2], "%lf", &Cx) ||
      1 != sscanf(argv[3], "%lf", &Cy)) {
    fprintf(stderr, "syntax: %s Bx Cx Cy\n", argv[0]);
    return 1;
  }

  while (! feof(stdin)) {
    if (inverse) {
	printf("inv> ");
    }
    else {
	printf("fwd> ");
    }
    fflush(stdout);

    if (NULL == fgets(buffer, BUFFERLEN, stdin)) {
      break;
    }
    if (1 != sscanf(buffer, "%s", cmd)) {
      continue;
    }

    if (! strcmp(cmd, "quit")) {
      break;
    }
    if (! strcmp(cmd, "i")) {
      inverse = 1;
      continue;
    }
    if (! strcmp(cmd, "f")) {
      inverse = 0;
      continue;
    }
    if (! strcmp(cmd, "ff")) {
      if (1 != sscanf(buffer, "%*s %d", &fflags)) {
	printf("need forward flag\n");
      }
      continue;
    }

    if (inverse) {		/* inverse kins */
      if (3 != sscanf(buffer, "%lf %lf %lf", 
		      &pos.tran.x,
		      &pos.tran.y,
		      &pos.tran.z)) {
	printf("need X Y Z\n");
	continue;
      }
      if (0 != kinematicsInverse(&pos, joints, NULL, &fflags)) {
	printf("inverse kin error\n");
      }
      else {
	printf("%f\t%f\t%f\n", joints[0], joints[1], joints[2]);
	if (0 != kinematicsForward(joints, &pos, &fflags, NULL)) {
	  printf("forward kin error\n");
	}
	else {
	  printf("%f\t%f\t%f\n", pos.tran.x, pos.tran.y, pos.tran.z);
	}
      }
    }
    else {			/* forward kins */
      if (flags) {
	if (4 != sscanf(buffer, "%lf %lf %lf %d", 
			&joints[0],
			&joints[1],
			&joints[2],
			&fflags)) {
	  printf("need 3 strut values and flag\n");
	  continue;
	}
      }
      else {
	if (3 != sscanf(buffer, "%lf %lf %lf", 
			&joints[0],
			&joints[1],
			&joints[2])) {
	  printf("need 3 strut values\n");
	  continue;
	}
      }
      if (0 != kinematicsForward(joints, &pos, &fflags, NULL)) {
	printf("forward kin error\n");
      }
      else {
	printf("%f\t%f\t%f\n", pos.tran.x, pos.tran.y, pos.tran.z);
	if (0 != kinematicsInverse(&pos, joints, NULL, &fflags)) {
	  printf("inverse kin error\n");
	}
	else {
	  printf("%f\t%f\t%f\n", joints[0], joints[1], joints[2]);
	}
      }
    }
  } /* end while (! feof(stdin)) */

  return 0;
}

#endif /* MAIN */

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"

EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);

MODULE_LICENSE("GPL");



int comp_id;
int rtapi_app_main(void) {
    int res = 0;

    comp_id = hal_init("ntripodkins");
    if(comp_id < 0) return comp_id;

    haldata = hal_malloc(sizeof(struct haldata));
    if(!haldata) goto error;

    if((res = hal_pin_float_new("ntripodkins.Bx", HAL_IO, &(haldata->bx), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("ntripodkins.Cx", HAL_IO, &(haldata->cx), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("ntripodkins.Cy", HAL_IO, &(haldata->cy), comp_id)) < 0) goto error;

    Bx = Cx = Cy = 1.0;
    hal_ready(comp_id);
    return 0;

error:
    hal_exit(comp_id);
    return res;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }
>>>>>>> c01773447196b072f2711b0c091a44a2bd26f7b3
