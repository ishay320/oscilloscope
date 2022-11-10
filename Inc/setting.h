#include <stdint.h>
#include <string.h>

#if defined(TEST)
#include <stdio.h>
#define CDC_Transmit_FS(c,s) puts((char*)c)
#else
#include "usbd_cdc_if.h"
#endif // TEST
