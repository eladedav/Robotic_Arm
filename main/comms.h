// comms.h
// Wi-Fi + HTTP server interface

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Initialize communications: NVS, Wi-Fi, HTTP server
void comms_init(void);

// Optional start hook (kept for symmetry). Can be empty if init starts everything.
void comms_start(void);

#ifdef __cplusplus
}
#endif
