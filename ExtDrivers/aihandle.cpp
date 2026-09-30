/*
 * aihandle.cpp
 *
 *  Created on: Jun 10, 2026
 *      Author: ivanov.y
 */

#include "aihandle.h"

ai_handle::ai_handle() {
    for (int i = 0; i < 16; i++) hidden1[i] = 0.0f;
    for (int i = 0; i < 8; i++) hidden2[i] = 0.0f;
}

ai_handle::~ai_handle() {}

bool ai_handle::NN_Init(void) {
    printf("=== Neural Network Initialized (Manual Inference) ===\n");
    printf("Model: 8 -> 16 -> 8 -> 1\n");
    printf("=====================================================\n");
    return true;
}

float ai_handle::NN_Predict(const uint16_t* inputs) {
    // ВАЖНО: используйте то же значение нормализации, что и при обучении!
    // Если данные в CSV были 0-4095, используйте 4095.0f
    // Если данные в CSV были 0-65535, используйте 65535.0f
    // Если данные в CSV уже были 0-1, используйте 1.0f (не делите!)

    const float NORMALIZATION_FACTOR = 4095.0f;  // ← ИЗМЕНИТЕ НА ПРАВИЛЬНОЕ ЗНАЧЕНИЕ

    // Нормализация входов
    float normalized[8];
    for (int i = 0; i < 8; i++) {
        normalized[i] = (float)inputs[i] / NORMALIZATION_FACTOR;
    }

    // Отладочный вывод (убрать после проверки)
    printf("Input normalized: [");
    for (int i = 0; i < 8; i++) {
//        printf("%.3f", normalized[i]);
        if (i < 7) printf(", ");
    }
    printf("]\n");

    // Слой 0: Dense(16) + ReLU
    for (int j = 0; j < 16; j++) {
        float sum = nn_weights.layer_1[j];  // bias
        for (int i = 0; i < 8; i++) {
            sum += normalized[i] * nn_weights.layer_0[i * 16 + j];
        }
        hidden1[j] = relu(sum);
    }

    // Слой 1: Dense(8) + ReLU
    for (int j = 0; j < 8; j++) {
        float sum = nn_weights.layer_3[j];  // bias
        for (int i = 0; i < 16; i++) {
            sum += hidden1[i] * nn_weights.layer_2[i * 8 + j];
        }
        hidden2[j] = relu(sum);
    }

    // Слой 2: Dense(1) + Sigmoid
    float sum = nn_weights.layer_5[0];  // bias
    for (int i = 0; i < 8; i++) {
        sum += hidden2[i] * nn_weights.layer_4[i];
    }

    float prediction = sigmoid(sum);
//    printf("Prediction: %.6f\n", prediction);

    return prediction;
}
