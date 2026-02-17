#ifndef RC_INPUT_H
#define RC_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

bool rc_signal_lost(void);

void rc_input_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // RC_INPUT_H
