#ifndef __SDIO_H__
#define __SDIO_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

  extern SD_HandleTypeDef hsd; // SDIO handle

  void MX_SDIO_SD_Init(void); // SDIO Initialization Function

#ifdef __cplusplus
}
#endif

#endif /* __SDIO_H__ */
