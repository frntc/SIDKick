#define XSTR(x) #x
#define STR(x) XSTR(x)
#define LOCAL_SKETCH_DIR c:\putyourpathhere\SIDKick\

#define HIGH_PRECISION_TABLES

//#define NO_RESID10

#include "reSID16/sid.h"
#ifndef NO_RESID10
#include "reSID/sid.h"
#endif
