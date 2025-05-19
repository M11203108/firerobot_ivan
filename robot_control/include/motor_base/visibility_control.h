#ifndef ZLAC8015D_CONTROL__VISIBILITY_CONTROL_H_
#define ZLAC8015D_CONTROL__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ZLAC8015D_CONTROL_EXPORT __attribute__ ((dllexport))
    #define ZLAC8015D_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define ZLAC8015D_CONTROL_EXPORT __declspec(dllexport)
    #define ZLAC8015D_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef ZLAC8015D_CONTROL_BUILDING_DLL
    #define ZLAC8015D_CONTROL_PUBLIC ZLAC8015D_CONTROL_EXPORT
  #else
    #define ZLAC8015D_CONTROL_PUBLIC ZLAC8015D_CONTROL_IMPORT
  #endif
  #define ZLAC8015D_CONTROL_PUBLIC_TYPE ZLAC8015D_CONTROL_PUBLIC
  #define ZLAC8015D_CONTROL_LOCAL
#else
  #define ZLAC8015D_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define ZLAC8015D_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define ZLAC8015D_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define ZLAC8015D_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ZLAC8015D_CONTROL_PUBLIC
    #define ZLAC8015D_CONTROL_LOCAL
  #endif
  #define ZLAC8015D_CONTROL_PUBLIC_TYPE
#endif

#endif  // ZLAC8015D_CONTROL__VISIBILITY_CONTROL_H_
