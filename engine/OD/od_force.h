/******************************************************************************
 * @file OD_force
 * @brief object dictionnary force
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/
#ifndef OD_OD_FORCE_H_
#define OD_OD_FORCE_H_

#include <string.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef struct
{
    float _private;
} moment_t;

typedef struct
{
    float _private;
} force_t;
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Function
 ******************************************************************************/

// moment are stored in Newton meter (Nm)
//******** Conversions ***********

// N.mm
static inline float ForceOD_MomentTo_N_mm(moment_t self)
{
    return self._private * 1000.0f;
}

static inline moment_t ForceOD_MomentFrom_N_mm(float n_mm)
{
    moment_t self;
    self._private = n_mm / 1000.0f;
    return self;
}

// N.cm
static inline float ForceOD_MomentTo_N_cm(moment_t self)
{
    return self._private * 100.0f;
}

static inline moment_t ForceOD_MomentFrom_N_cm(float n_cm)
{
    moment_t self;
    self._private = n_cm / 100.0f;
    return self;
}

// N.m
static inline float ForceOD_MomentTo_N_m(moment_t self)
{
    return self._private;
}

static inline moment_t ForceOD_MomentFrom_N_m(float n_m)
{
    moment_t self;
    self._private = n_m;
    return self;
}

// kgf.mm
static inline float ForceOD_MomentTo_kgf_mm(moment_t self)
{
    return self._private * 101.97f;
}

static inline moment_t ForceOD_MomentFrom_kgf_mm(float kgf_mm)
{
    moment_t self;
    self._private = kgf_mm / 101.97f;
    return self;
}

// kgf.cm
static inline float ForceOD_MomentTo_kgf_cm(moment_t self)
{
    return self._private * 10.2f;
}

static inline moment_t ForceOD_MomentFrom_kgf_cm(float kgf_cm)
{
    moment_t self;
    self._private = kgf_cm / 10.2f;
    return self;
}

// kgf.m
static inline float ForceOD_MomentTo_kgf_m(moment_t self)
{
    return self._private * 0.102f;
}

static inline moment_t ForceOD_MomentFrom_kgf_m(float kgf_m)
{
    moment_t self;
    self._private = kgf_m / 0.102f;
    return self;
}

// ozf.in
static inline float ForceOD_MomentTo_ozf_in(moment_t self)
{
    return self._private * 141.612f;
}

static inline moment_t ForceOD_MomentFrom_ozf_in(float ozf_in)
{
    moment_t self;
    self._private = ozf_in / 141.612f;
    return self;
}

// lbf.in
static inline float ForceOD_MomentTo_lbf_in(moment_t self)
{
    return self._private * 8.851f;
}

static inline moment_t ForceOD_MomentFrom_lbf_in(float lbf_in)
{
    moment_t self;
    self._private = lbf_in / 8.851f;
    return self;
}

//******** Messages management ***********
static inline void ForceOD_MomentToMsg(const moment_t *const self, msg_t *const msg)
{
    LUOS_ASSERT(self);
    LUOS_ASSERT(msg);
    msg->header.cmd = MOMENT;
    memcpy(msg->data, self, sizeof(moment_t));
    msg->header.size = sizeof(moment_t);
}

static inline void ForceOD_MomentFromMsg(moment_t *const self, const msg_t *const msg)
{
    LUOS_ASSERT(self);
    LUOS_ASSERT(msg);
    memcpy(self, msg->data, msg->header.size);
}

// force are stored in Newton (N)
//******************************** Conversions *******************************

// N
static inline float ForceOD_ForceTo_N(force_t self)
{
    return self._private;
}

static inline force_t ForceOD_ForceFrom_N(float n)
{
    force_t self;
    self._private = n;
    return self;
}

// kgf
static inline float ForceOD_ForceTo_kgf(force_t self)
{
    return self._private * 0.102f;
}

static inline force_t ForceOD_ForceFrom_kgf(float kgf)
{
    force_t self;
    self._private = kgf / 0.102f;
    return self;
}

// ozf
static inline float ForceOD_ForceTo_ozf(force_t self)
{
    return self._private * 3.5969430896f;
}

static inline force_t ForceOD_ForceFrom_ozf(float ozf)
{
    force_t self;
    self._private = ozf / 3.5969430896f;
    return self;
}

// lbf
static inline float ForceOD_ForceTo_lbf(force_t self)
{
    return self._private * 0.2248089431f;
}

static inline force_t ForceOD_ForceFrom_lbf(float lbf)
{
    force_t self;
    self._private = lbf / 0.2248089431f;
    return self;
}

//******** Messages management ***********
static inline void ForceOD_ForceToMsg(const force_t *const self, msg_t *const msg)
{
    LUOS_ASSERT(self);
    LUOS_ASSERT(msg);
    msg->header.cmd = FORCE;
    memcpy(msg->data, self, sizeof(force_t));
    msg->header.size = sizeof(force_t);
}

static inline void ForceOD_ForceFromMsg(force_t *const self, const msg_t *const msg)
{
    LUOS_ASSERT(self);
    LUOS_ASSERT(msg);
    memcpy(self, msg->data, msg->header.size);
}

#endif /* OD_OD_FORCE_H_ */
