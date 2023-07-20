#ifndef SOTH_VISIBILITY_H
#define SOTH_VISIBILITY_H

// Generic helper definitions for shared library support
#if defined _WIN32 || defined __CYGWIN__
#define SOTH_HELPER_DLL_IMPORT __declspec(dllimport)
#define SOTH_HELPER_DLL_EXPORT __declspec(dllexport)
#define SOTH_HELPER_DLL_LOCAL
#else
#if __GNUC__ >= 4
#define SOTH_HELPER_DLL_IMPORT __attribute__ ((visibility ("default")))
#define SOTH_HELPER_DLL_EXPORT __attribute__ ((visibility ("default")))
#define SOTH_HELPER_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
#else
#define SOTH_HELPER_DLL_IMPORT
#define SOTH_HELPER_DLL_EXPORT
#define SOTH_HELPER_DLL_LOCAL
#endif
#endif

// Now we use the generic helper definitions above to define SOTH_API and SOTH_LOCAL.
// SOTH_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static build)
// SOTH_LOCAL is used for non-api symbols.

#ifdef SOTH_DLL // defined if SOTH is compiled as a DLL
#ifdef SOTH_DLL_EXPORTS // defined if we are building the SOTH DLL (instead of using it)
#define SOTH_API SOTH_HELPER_DLL_EXPORT
#else
#define SOTH_API SOTH_HELPER_DLL_IMPORT
#endif // SOTH_DLL_EXPORTS
#define SOTH_LOCAL SOTH_HELPER_DLL_LOCAL
#else // SOTH_DLL is not defined: this means SOTH is a static lib.
#define SOTH_API
#define SOTH_LOCAL
#endif // SOTH_DLL

#ifndef SOTH_API
#define SOTH_API
#endif

#ifndef SOTH_LOCAL
#define SOTH_LOCAL
#endif

#endif // VISIBILITY_H
