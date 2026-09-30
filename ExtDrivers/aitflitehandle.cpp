/*
 * aitflitehandle.cpp
 *
 *  Created on: 10 июн. 2026 г.
 *      Author: xwest
 */

#include "aitflitehandle.h"

ai_tflite_handle::ai_tflite_handle()
    : model(nullptr),
      interpreter(nullptr),
      input_tensor(nullptr),
      output_tensor(nullptr),
      initialized(false)
{
    memset(tensor_arena, 0, TENSOR_ARENA_SIZE);
}

ai_tflite_handle::~ai_tflite_handle() {
    if (interpreter) {
        delete interpreter;
        interpreter = nullptr;
    }
}

bool ai_tflite_handle::NN_Init(void) {
    printf("=== TFLite Init ===\n");
    printf("Arena: %d bytes\n", TENSOR_ARENA_SIZE);
    printf("Model: %d bytes\n", model_tflite_len);

    // 1. Загрузка модели
    model = tflite::GetModel(model_tflite);
    if (!model) {
        printf("ERROR: GetModel failed\n");
        return false;
    }

    if (model->version() != TFLITE_SCHEMA_VERSION) {
        printf("ERROR: Schema mismatch: %d vs %d\n",
               model->version(), TFLITE_SCHEMA_VERSION);
        return false;
    }
    printf("✓ Model loaded\n");

    // 2. Регистрация операций
    resolver.AddFullyConnected();
    resolver.AddRelu();
    resolver.AddLogistic();
    resolver.AddMul();
    resolver.AddAdd();
    resolver.AddReshape();
    printf("✓ Ops registered\n");

    // 3. Создание интерпретатора
    interpreter = new tflite::MicroInterpreter(
        model, resolver, tensor_arena, TENSOR_ARENA_SIZE
    );

    // 4. Выделение памяти - КРИТИЧНАЯ ПРОВЕРКА
    TfLiteStatus status = interpreter->AllocateTensors();
    if (status != kTfLiteOk) {
        printf("ERROR: AllocateTensors FAILED (status=%d)\n", (int)status);
        printf("Arena too small! Try 32768\n");
        delete interpreter;
        interpreter = nullptr;
        return false;
    }
    printf("✓ Tensors allocated\n");

    // 5. Получение тензоров
    input_tensor = interpreter->input(0);
    output_tensor = interpreter->output(0);

    if (!input_tensor || !output_tensor) {
        printf("ERROR: Null tensors\n");
        return false;
    }

    // 6. Проверка валидности
    if (input_tensor->dims == nullptr || input_tensor->dims->size == 0) {
        printf("ERROR: Invalid input dims\n");
        return false;
    }

    printf("✓ Input: type=%d, dims=%d\n",
           (int)input_tensor->type, input_tensor->dims->size);
    printf("✓ Output: type=%d, dims=%d\n",
           (int)output_tensor->type, output_tensor->dims->size);

    initialized = true;
    printf("=== Ready ===\n\n");
    return true;
}

float ai_tflite_handle::NN_Predict(const uint16_t* inputs) {
    if (!initialized || !input_tensor || !output_tensor) {
        printf("ERROR: Not initialized\n");
        return -1.0f;
    }

    // Нормализация
    float* input_data = input_tensor->data.f;
    for (int i = 0; i < 8; i++) {
        input_data[i] = (float)inputs[i] / 4095.0f;
    }

    // Инференс
    TfLiteStatus status = interpreter->Invoke();
    if (status != kTfLiteOk) {
        printf("ERROR: Invoke failed (status=%d)\n", (int)status);
        return -1.0f;
    }

    return output_tensor->data.f[0];
}
