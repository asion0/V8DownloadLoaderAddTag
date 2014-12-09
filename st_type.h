#ifndef ST_TYPE_INCLUDED
#define ST_TYPE_INCLUDED

// define standard types
typedef signed   char             S08;
typedef unsigned char             U08;
typedef signed   short int        S16;
typedef unsigned short int        U16;
typedef signed   long int         S32;
typedef unsigned long int         U32;
typedef float                     F32;
typedef double                    D64;

typedef signed   char            *PS08;
typedef unsigned char            *PU08;
typedef signed   short int       *PS16;
typedef unsigned short int       *PU16;
typedef signed   long int        *PS32;
typedef unsigned long int        *PU32;
typedef float                    *PF32;
typedef double                   *PD64;
//Andrew
typedef void		         VOID;

// define standard type limits
#include <limits.h>

#define S08_MIN   SCHAR_MIN
#define S08_MAX   SCHAR_MAX
#define U08_MAX   UCHAR_MAX

#define S16_MIN   SHRT_MIN
#define S16_MAX   SHRT_MAX
#define U16_MAX   USHRT_MAX

#define S32_MIN   LONG_MIN
#define S32_MAX   LONG_MAX
#define U32_MAX   ULONG_MAX

// defined position types
typedef struct {
  D64 px;
  D64 py;
  D64 pz;
} POS_T;

typedef struct {
  F32 vx;
  F32 vy;
  F32 vz;
} VEL_T;

typedef struct {
  F32 ax;
  F32 ay;
  F32 az;
} ACC_T;

typedef struct {
  D64 lat;
  D64 lon;
  F32 alt;
} LLA_T;

typedef POS_T *PPOS_T;
typedef VEL_T *PVEL_T;
typedef ACC_T *PACC_T;
typedef LLA_T *PLLA_T;

#endif  // #ifndef ST_TYPE_INCLUDED

