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
