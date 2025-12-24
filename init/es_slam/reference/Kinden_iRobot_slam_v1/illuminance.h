#if !defined(ILLUMINANCE_HEADER)
#define ILLUMINANCE_HEADER
#include "Def.h"

int T10A_Initial(void);
int T_10A_PC_MODE(void);
double getIlluminanceData(int wait);
void BCC_Calculate(char *Command, int len);
void illuminanceThread(void);

#endif