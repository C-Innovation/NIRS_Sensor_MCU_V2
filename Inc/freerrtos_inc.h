/*
 * freerrtos.h
 *
 *  Created on: Jun 10, 2026
 *      Author: ivanov.y
 */

#ifndef FREERRTOS_H_
#define FREERRTOS_H_

#define BUFFER_SIZE (int)4096
#define USE_TFLITE 	1
#ifdef __cplusplus
extern "C" {
#endif

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

#ifdef __cplusplus
}
#endif

#endif /* FREERRTOS_H_ */
