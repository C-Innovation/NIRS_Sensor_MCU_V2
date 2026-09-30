/*
 * aitflitehandle.h
 *
 *  Created on: 10 июн. 2026 г.
 *      Author: xwest
 */

#ifndef AITFLITEHANDLE_H_
#define AITFLITEHANDLE_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "model_tflite.h"

#define TENSOR_ARENA_SIZE 2048

class ai_tflite_handle {
public:
    ai_tflite_handle();
    virtual ~ai_tflite_handle();

    bool NN_Init(void);
    float NN_Predict(const uint16_t* inputs);

private:
    const tflite::Model* model;
    tflite::MicroInterpreter* interpreter;
    TfLiteTensor* input_tensor;
    TfLiteTensor* output_tensor;
    bool initialized;

    uint8_t tensor_arena[TENSOR_ARENA_SIZE];

    // 6 операций: FullyConnected, Relu, Logistic, Mul, Add, Reshape
    tflite::MicroMutableOpResolver<6> resolver;
};

#endif /* AITFLITEHANDLE_H_ */
