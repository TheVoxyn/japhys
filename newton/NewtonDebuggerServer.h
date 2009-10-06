#ifndef NEWTONDEBUGGER_H
#define NEWTONDEBUGGER_H

// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the NEWTONDEBUGGER_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// NEWTONDEBUGGER_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef NEWTONDEBUGGER_EXPORTS
#define NEWTONDEBUGGER_API __declspec(dllexport)
#else
#define NEWTONDEBUGGER_API __declspec(dllimport)
#endif


#include <windows.h>

#ifdef __cplusplus 
extern "C" {
#endif


	struct NewtonWorld;

	NEWTONDEBUGGER_API void* NewtonDebuggerCreateServer ();
	NEWTONDEBUGGER_API void NewtonDebuggerDestroyServer (void* server);
	NEWTONDEBUGGER_API void NewtonDebuggerServe (void* server, NewtonWorld* world);

#ifdef __cplusplus 
}
#endif

#endif