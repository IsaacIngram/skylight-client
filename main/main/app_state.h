// 
// app_state.h
//
// Makeplus
// Isaac Ingram
//
// Provide universal state machine
//

#ifndef APP_STATE_H
#define APP_STATE_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef enum {
    STATE_BOOT,
    STATE_IDLE,
    STATE_DISCONNECTED,
    STATE_CONNECTED,
    STATE_PAIRING
} AppState;

typedef struct {
    AppState state;
    SemaphoreHandle_t mutex;
} AppStateContainer;

extern AppStateContainer app_state;

void init_app_state(void);
void cleanup_app_state(void);

#endif /* APP_STATE_H */