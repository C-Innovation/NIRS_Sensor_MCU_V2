/*
 * aihandle.h
 *
 *  Created on: Jun 10, 2026
 *      Author: ivanov.y
 */

#ifndef AIHANDLE_H_
#define AIHANDLE_H_

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "model_weights.h"

class ai_handle {
public:
    ai_handle();
    virtual ~ai_handle();

    bool NN_Init(void);
    float NN_Predict(const uint16_t* inputs);

private:
    float hidden1[16];  // Промежуточный буфер слоя 1
    float hidden2[8];   // Промежуточный буфер слоя 2

    // Функции активации
    static inline float relu(float x) {
        return x > 0.0f ? x : 0.0f;
    }

    static inline float sigmoid(float x) {
        return 1.0f / (1.0f + expf(-x));
    }
};


#endif /* AIHANDLE_H_ */
