#ifndef _AURA_PYMISSION_HXX
#define _AURA_PYMISSION_HXX

#include <Python.h>

// This function must be called (once) before update()
extern bool pyMissionInit();

// This function must be called before update()
extern bool pyMissionUpdate();

// This function can be called at exit to properly free resources
// requested by init()
extern void pyPropsClose(void);

#endif // _AURA_PYMISSION_HXX
