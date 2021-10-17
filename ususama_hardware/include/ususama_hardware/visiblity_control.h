#ifndef USUSAMA_HARDWARE__VISIBLITY_CONTROL_H_
#define USUSAMA_HARDWARE__VISIBLITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define USUSAMA_HARDWARE_EXPORT __attribute__((dllexport))
#define USUSAMA_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define USUSAMA_HARDWARE_EXPORT __declspec(dllexport)
#define USUSAMA_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef USUSAMA_HARDWARE_BUILDING_DLL
#define USUSAMA_HARDWARE_PUBLIC USUSAMA_HARDWARE_EXPORT
#else
#define USUSAMA_HARDWARE_PUBLIC USUSAMA_HARDWARE_IMPORT
#endif
#define USUSAMA_HARDWARE_PUBLIC_TYPE USUSAMA_HARDWARE_PUBLIC
#define USUSAMA_HARDWARE_LOCAL
#else
#define USUSAMA_HARDWARE_EXPORT __attribute__((visibility("default")))
#define USUSAMA_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define USUSAMA_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define USUSAMA_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define USUSAMA_HARDWARE_PUBLIC
#define USUSAMA_HARDWARE_LOCAL
#endif
#define USUSAMA_HARDWARE_PUBLIC_TYPE
#endif

#endif  // USUSAMA_HARDWARE__VISIBLITY_CONTROL_H_