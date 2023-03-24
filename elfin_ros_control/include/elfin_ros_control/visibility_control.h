#ifndef ELFIN_HARDWARE_INTERFACE__VISIBILITY_CONTROL_HPP_
#define ELFIN_HARDWARE_INTERFACE__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ELFIN_HARDWARE_INTERFACE_EXPORT __attribute__((dllexport))
#define ELFIN_HARDWARE_INTERFACE_IMPORT __attribute__((dllimport))
#else
#define ELFIN_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
#define ELFIN_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
#endif
#ifdef ELFIN_HARDWARE_INTERFACE_BUILDING_LIBRARY
#define ELFIN_HARDWARE_INTERFACE_PUBLIC ELFIN_HARDWARE_INTERFACE_EXPORT
#else
#define ELFIN_HARDWARE_INTERFACE_PUBLIC ELFIN_HARDWARE_INTERFACE_IMPORT
#endif
#define ELFIN_HARDWARE_INTERFACE_PUBLIC_TYPE ELFIN_HARDWARE_INTERFACE_PUBLIC
#define ELFIN_HARDWARE_INTERFACE_LOCAL
#else
#define ELFIN_HARDWARE_INTERFACE_EXPORT __attribute__((visibility("default")))
#define ELFIN_HARDWARE_INTERFACE_IMPORT
#if __GNUC__ >= 4
#define ELFIN_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))
#define ELFIN_HARDWARE_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#define ELFIN_HARDWARE_INTERFACE_PUBLIC
#define ELFIN_HARDWARE_INTERFACE_LOCAL
#endif
#define ELFIN_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // ELFIN_HARDWARE_INTERFACE__VISIBILITY_CONTROL_HPP_