#ifndef WHISKEYE_PLUGIN__VISIBILITY_CONTROL_H_
#define WHISKEYE_PLUGIN__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define WHISKEYE_PLUGIN_EXPORT __attribute__ ((dllexport))
    #define WHISKEYE_PLUGIN_IMPORT __attribute__ ((dllimport))
  #else
    #define WHISKEYE_PLUGIN_EXPORT __declspec(dllexport)
    #define WHISKEYE_PLUGIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef WHISKEYE_PLUGIN_BUILDING_LIBRARY
    #define WHISKEYE_PLUGIN_PUBLIC WHISKEYE_PLUGIN_EXPORT
  #else
    #define WHISKEYE_PLUGIN_PUBLIC WHISKEYE_PLUGIN_IMPORT
  #endif
  #define WHISKEYE_PLUGIN_PUBLIC_TYPE WHISKEYE_PLUGIN_PUBLIC
  #define WHISKEYE_PLUGIN_LOCAL
#else
  #define WHISKEYE_PLUGIN_EXPORT __attribute__ ((visibility("default")))
  #define WHISKEYE_PLUGIN_IMPORT
  #if __GNUC__ >= 4
    #define WHISKEYE_PLUGIN_PUBLIC __attribute__ ((visibility("default")))
    #define WHISKEYE_PLUGIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define WHISKEYE_PLUGIN_PUBLIC
    #define WHISKEYE_PLUGIN_LOCAL
  #endif
  #define WHISKEYE_PLUGIN_PUBLIC_TYPE
#endif

#endif  // WHISKEYE_PLUGIN__VISIBILITY_CONTROL_H_
