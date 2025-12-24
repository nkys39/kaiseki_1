#if !defined(SENSING_HEADER)
#define SENSING_HEADER

#include "Def.h"

int sensorComm_Init(void);
void SensorDataReceive(void);
int Shutdown(void);
void logSave(FILE*, unsigned char[]);
#endif