/********************************************************************
* Description: align_gantry_kins.c
*   Simple example kinematics for thita alignment in software
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author: Yishin Li, ARAIS ROBOT TECHNOLOGY
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2011 All rights reserved.
*
********************************************************************/

#include "motion.h"		/* these decls */
#include "hal.h"
#include "rtapi.h"
#include "rtapi_math.h"
#include "rtapi_string.h"
#include "rtapi_app.h"		/* RTAPI realtime module decls */

typedef struct {
    hal_float_t *yy_offset;
} align_pins_t;

static align_pins_t *align_pins;

#define YY_OFFSET       (*(align_pins->yy_offset))

const char *machine_type = "";
RTAPI_MP_STRING(machine_type, "Gantry Machine Type");

EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

int kinematicsForward(const double *joints,
		      EmcPose * pos,
		      const KINEMATICS_FORWARD_FLAGS * fflags,
		      KINEMATICS_INVERSE_FLAGS * iflags)
{

    pos->tran.x = joints[0];
    pos->tran.y = joints[1];
    pos->tran.z = joints[3];
    pos->w = joints[4];
    pos->u = joints[5];

    // DP("kFWD: x(%f), y(%f), j0(%f), j1(%f), j2(%f), yy_offset(%f)\n",
    //     pos->tran.x, pos->tran.y, joints[0], joints[1], joints[2], YY_OFFSET);
    // DP("kFWD: s(%f), j5(%f)\n", pos->s, joints[5]);

    return 0;
}

int kinematicsInverse(const EmcPose * pos,
		      double *joints,
		      const KINEMATICS_INVERSE_FLAGS * iflags,
		      KINEMATICS_FORWARD_FLAGS * fflags)
{
    joints[0] = pos->tran.x;
    joints[1] = pos->tran.y;
    joints[2] = pos->tran.y - YY_OFFSET;  // YY
    joints[3] = pos->tran.z;
    joints[4] = pos->w;
    joints[5] = pos->u;

    // DP("kINV: x(%f), y(%f), j0(%f), j1(%f), j2(%f), yy_offset(%f)\n",
    //    pos->tran.x, pos->tran.y, joints[0], joints[1], joints[2], YY_OFFSET);
    // DP("kINV: s(%f), j5(%f)\n", pos->s, joints[5]);

    return 0;
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


int comp_id;
int rtapi_app_main(void) 
{
    int res = 0;

// #if (TRACE!=0)
//     dptrace = fopen("kins.log","w");
// #endif
    
//    DP("begin\n");
    comp_id = hal_init("align_gantry_kins");
    if (comp_id < 0) {
        // ERROR
//        DP("ABORT\n");
        return comp_id;
    }
    
    align_pins = hal_malloc(sizeof(align_pins_t));
    if (!align_pins) goto error;
    if ((res = hal_pin_float_new("align-gantry-kins.yy-offset", HAL_IN, &(align_pins->yy_offset), comp_id)) < 0) goto error;
    YY_OFFSET = 0;

    hal_ready(comp_id);
//    DP ("success\n");
    return 0;
    
error:
//    DP("ERROR\n");
    hal_exit(comp_id);
#if (TRACE!=0)
    fclose(dptrace);
#endif
    return res;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }
