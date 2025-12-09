/*
 * garagelib.h - Security+ garage door protocol library
 *
 * Header file exposing the debug callback API for runtime-configurable
 * debug output to MQTT or other destinations.
 */

#ifndef GARAGELIB_H
#define GARAGELIB_H

#ifdef __cplusplus
extern "C" {
#endif

// Debug callback type - receives formatted debug messages
typedef void (*garagelib_debug_callback_t)(const char* message);

// Set the debug callback. Pass NULL to disable debug output.
void garagelib_set_debug_callback(garagelib_debug_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif // GARAGELIB_H
