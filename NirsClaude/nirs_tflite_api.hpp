// ============================================================================
//  nirs_tflite_api.hpp — инференс модели через TensorFlow Lite Micro на МК.
//
//  Собирается вместе с nirs_nn_runtime.c и сгенерированным массивом модели
//  (gru_pre_f32_model.c из 03_export.py).
//
//  Модель рекуррентная: ДВА входа (признаки и скрытое состояние) и ДВА
//  выхода (предсказание и новое состояние). TFLM не хранит состояние между
//  вызовами, поэтому его гоняет вызывающий код — этим занимается
//  nirs_nn_runtime.c.
//
//  ВАЖНО, если AllocateTensors() возвращает не kTfLiteOk:
//  ------------------------------------------------------
//  Экспортированный граф содержит ВОСЕМЬ разных операций, а не семь:
//
//      CONCATENATION, FULLY_CONNECTED, LOGISTIC, MUL, TANH, SUB,
//      RESHAPE, ADD
//
//  RESHAPE появляется в графе из-за приведения формы перед сложением
//  (узел 12 из 15). В прежней версии этого файла он не был зарегистрирован,
//  и AllocateTensors() падал именно на нём: разбор операций происходит
//  внутри AllocateTensors, и на незнакомом коде операции она возвращает
//  kTfLiteError с сообщением "Didn't find op for builtin opcode 'RESHAPE'".
//
//  Второй капкан: размер резолвера задаётся ШАБЛОННЫМ параметром.
//  MicroMutableOpResolver<7> с семью Add*() заполнен под завязку, и восьмой
//  Add*() молча вернёт ошибку, если не проверять его результат. Поэтому
//  здесь <8> и проверка каждого вызова.
//
//  Этот заголовок содержит ОПРЕДЕЛЕНИЯ, а не только объявления: включайте
//  его ровно из одного .cpp, иначе будет дубль символов при линковке.
// ============================================================================
#ifndef NIRS_TFLITE_API_HPP_
#define NIRS_TFLITE_API_HPP_

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/schema/schema_generated.h"

extern "C" {
#include "nn/nirs_nn_runtime.h"
#include "nn/export/gru_pre_f32_model.h"
}

// Сгенерированный массив с моделью (gru_pre_f32_model.c).
// Массив ДОЛЖЕН быть выровнен на 16 байт — flatbuffer читается на месте.
//extern "C" const unsigned char g_nirs_model[];
//extern "C" const unsigned int  g_nirs_model_len;

namespace {

// Арена. Для этой модели (15 узлов, 26 тензоров, самый большой — [1,48])
// хватает единиц килобайт. Точный размер печатает arena_used_bytes() ПОСЛЕ
// AllocateTensors — возьмите его из лога и урежьте с запасом ~20 %.
// 96 КБ, как было раньше, — это треть всей RAM у STM32U545RE (274 КБ)
// впустую.
constexpr int kArenaSize = 16 * 1024;
alignas(16) uint8_t g_arena[kArenaSize];

const tflite::Model*      g_model  = nullptr;
tflite::MicroInterpreter* g_interp = nullptr;

// индексы входов/выходов определяются один раз при старте: порядок
// в файле .tflite не гарантирован, ищем по размеру тензора
int g_in_x = -1, g_in_h = -1, g_out_y = -1, g_out_h = -1;

// Диагностика: перечислить операции, которые РЕАЛЬНО нужны модели.
// Вызывается при ошибке разбора, чтобы не гадать, какого Add*() не хватает.
void dump_required_ops(const tflite::Model* m)
{
    auto* codes = m->operator_codes();
    if (codes == nullptr) return;
    MicroPrintf("модели нужны операции (%d шт.):", (int)codes->size());
    for (unsigned i = 0; i < codes->size(); ++i) {
        const tflite::OperatorCode* oc = codes->Get(i);
        // builtin_code появился в схеме позже deprecated_builtin_code;
        // у старых файлов заполнено только второе поле
        int bc = (int)oc->builtin_code();
        if (bc == 0) bc = (int)oc->deprecated_builtin_code();
        MicroPrintf("  [%d] %s", i,
                    tflite::EnumNameBuiltinOperator((tflite::BuiltinOperator)bc));
    }
}

}  // namespace

extern "C" int nirs_nn_setup(void)
{
    // Выравнивание массива модели: если тут не 0, уберите ручные правки из
    // gru_pre_f32_model.c и верните alignas(16). Невыровненный flatbuffer
    // читается «почти правильно» и ломается в самых неожиданных местах.
    if (((uintptr_t)g_nirs_model & 0xF) != 0) {
        MicroPrintf("массив модели не выровнен на 16 байт (адрес %p)",
                    (const void*)g_nirs_model);
        return -4;
    }

    g_model = tflite::GetModel(g_nirs_model);
    if (g_model->version() != TFLITE_SCHEMA_VERSION) {
        MicroPrintf("несовпадение версии схемы: %d против %d",
                    (int)g_model->version(), (int)TFLITE_SCHEMA_VERSION);
        return -1;
    }

    // Регистрируем ровно те операции, что есть в графе, — так прошивка
    // меньше. Число в <> должно быть НЕ МЕНЬШЕ количества Add*() ниже.
    static tflite::MicroMutableOpResolver<8> resolver;
    TfLiteStatus st = kTfLiteOk;
    if (resolver.AddConcatenation()  != kTfLiteOk) st = kTfLiteError;
    if (resolver.AddFullyConnected() != kTfLiteOk) st = kTfLiteError;
    if (resolver.AddLogistic()       != kTfLiteOk) st = kTfLiteError;
    if (resolver.AddMul()            != kTfLiteOk) st = kTfLiteError;
    if (resolver.AddTanh()           != kTfLiteOk) st = kTfLiteError;
    if (resolver.AddSub()            != kTfLiteOk) st = kTfLiteError;
    if (resolver.AddReshape()        != kTfLiteOk) st = kTfLiteError;   // <-- его и не хватало
    if (resolver.AddAdd()            != kTfLiteOk) st = kTfLiteError;
    if (st != kTfLiteOk) {
        MicroPrintf("резолвер переполнен: увеличьте MicroMutableOpResolver<N>");
        return -5;
    }

    static tflite::MicroInterpreter interp(g_model, resolver, g_arena, kArenaSize);
    g_interp = &interp;

    if (g_interp->AllocateTensors() != kTfLiteOk) {
        // Две основные причины: не зарегистрирована операция и мала арена.
        // Первую видно по списку ниже, вторую — по сообщению TFLM
        // "Failed to allocate tensor ..." прямо перед этим.
        MicroPrintf("AllocateTensors не удался");
        dump_required_ops(g_model);
        MicroPrintf("проверьте: все ли эти операции добавлены в резолвер "
                    "и хватает ли арены (сейчас %d байт)", kArenaSize);
        return -2;
    }

    MicroPrintf("арена использована: %d байт из %d",
                (int)g_interp->arena_used_bytes(), kArenaSize);

    // разбираем, какой вход что значит, по числу элементов
    for (size_t i = 0; i < g_interp->inputs_size(); ++i) {
        TfLiteTensor* t = g_interp->input(i);
        int n = t->dims->data[t->dims->size - 1];
        if (n == NIRS_NN_CH)          g_in_x = (int)i;
        else if (n == NIRS_NN_STATE)  g_in_h = (int)i;
    }
    for (size_t i = 0; i < g_interp->outputs_size(); ++i) {
        TfLiteTensor* t = g_interp->output(i);
        int n = t->dims->data[t->dims->size - 1];
        if (n == 1)                   g_out_y = (int)i;
        else if (n == NIRS_NN_STATE)  g_out_h = (int)i;
    }
    if (g_in_x < 0 || g_in_h < 0 || g_out_y < 0 || g_out_h < 0) {
        MicroPrintf("не найдены тензоры: проверьте NIRS_NN_CH / NIRS_NN_STATE");
        return -3;
    }

    // Модель float32: если экспортировали int8-вариант, здесь будет
    // kTfLiteInt8, и запись через data.f испортит вход молча.
    if (g_interp->input(g_in_x)->type != kTfLiteFloat32) {
        MicroPrintf("вход не float32 (type=%d) — это int8-модель, "
                    "она требует другого кода записи входа",
                    (int)g_interp->input(g_in_x)->type);
        return -6;
    }
    return 0;
}

// вызывается из nirs_nn_update()
extern "C" int nirs_nn_invoke(const float* feat8, const float* state_in,
                              float* y_out, float* state_out)
{
    if (!g_interp) return -1;

    float* x = g_interp->input(g_in_x)->data.f;
    float* h = g_interp->input(g_in_h)->data.f;
    for (int i = 0; i < NIRS_NN_CH;    ++i) x[i] = feat8[i];
    for (int i = 0; i < NIRS_NN_STATE; ++i) h[i] = state_in[i];

    if (g_interp->Invoke() != kTfLiteOk) return -2;

    *y_out = g_interp->output(g_out_y)->data.f[0];
    const float* hn = g_interp->output(g_out_h)->data.f;
    for (int i = 0; i < NIRS_NN_STATE; ++i) state_out[i] = hn[i];
    return 0;
}

#endif  // NIRS_TFLITE_API_HPP_

// ============================================================================
//  Вывод диагностики TFLM на STM32
// ============================================================================
//  MicroPrintf работает через DebugLog(). В поставке TFLM для «голого»
//  железа это пустая заглушка, поэтому все сообщения об ошибках уходят
//  в никуда — и AllocateTensors выглядит как молчаливый отказ.
//  Реализуйте её один раз, например через UART:
//
//      extern "C" void DebugLog(const char* s) {
//          HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 100);
//      }
//
//  или через ITM/SWO (у STM32U5 есть):
//
//      extern "C" void DebugLog(const char* s) {
//          while (*s) ITM_SendChar(*s++);
//      }
//
//  В новых версиях TFLM сигнатура может быть
//  `void DebugLog(const char* format, va_list args)` — смотрите
//  tensorflow/lite/micro/debug_log.h в своей копии.
// ============================================================================

// ============================================================================
//  Про TF_LITE_STATIC_MEMORY
// ============================================================================
//  Флаг правильный, его и надо держать включённым. Но он МЕНЯЕТ РАЗМЕР
//  структур TfLiteTensor / TfLiteNode / TfLiteIntArray. Поэтому он должен
//  быть определён ОДИНАКОВО для всех единиц трансляции: и для исходников
//  TFLM, и для этого файла, и для всего, что включает common.h.
//  Задавайте его глобально в настройках проекта (-DTF_LITE_STATIC_MEMORY),
//  а не в отдельном файле. Если определить его только у себя, структуры
//  разъедутся, и падать будет в случайных местах, а не в одном.
//
//  На AllocateTensors в вашем случае он не влияет: там не хватало RESHAPE.
// ============================================================================

// ============================================================================
//  Пример использования в прошивке
// ============================================================================
#if 0

#include "nirs_contraction.h"     // детерминированное ядро — для сверки

static nirs_nn_t g_nn;
static nirs_t    g_ref;           // опционально: считать оба и сравнивать

void app_init(void)
{
    int rc = nirs_nn_setup();
    if (rc != 0) { error_blink(rc); }   // код ошибки см. выше по файлу
    nirs_nn_init(&g_nn, 1000.0f);

    nirs_cfg_t cfg;
    nirs_defaults(&cfg, 1000.0f);
    nirs_init(&g_ref, &cfg);
}

// вызывается ровно раз на кадр, 1 кГц
void on_adc_frame(const uint16_t* adc)
{
    static const float k = 3.30f * 2.0f / 4095.0f;
    float v[NIRS_NN_CH];
    for (int i = 0; i < NIRS_NN_CH; ++i) v[i] = adc[i] * k;

    float y_nn  = nirs_nn_update(&g_nn, v);      // выход сети
    float y_ref = nirs_update(&g_ref, v);        // выход алгоритма

    dac_write((uint16_t)(y_nn * 4095.0f));

    // валидность данных берётся у ядра: пока датчик не на мышце —
    // выходу верить нельзя (см. README §6b)
    int valid = nirs_tracking(&g_ref);

    // в отладке полезно видеть расхождение: если оно растёт, сеть
    // столкнулась с условиями, которых не было в обучающих данных
    static float err = 0.0f;
    err += 0.001f * (fabsf(y_nn - y_ref) - err);
    if ((g_nn.n % 1000) == 0)
        log_info("расхождение сеть/алгоритм: %.3f, валидность: %d", err, valid);
}

#endif

// ============================================================================
//  Оценка ресурсов (GRU, 16 состояний, вход 8), Cortex-M33 @160 МГц
//
//    веса                ~1.2 тыс. float32   ≈ 4.8 КБ flash
//    файл .tflite                            = 8624 байта flash
//    арена TFLM                              единицы КБ RAM (смотрите лог)
//    состояние runtime                       ≈ 100 байт RAM
//    один Invoke()                           ≈ 3 тыс. умножений, 15 узлов
//
//  При NIRS_NN_DECIM = 10 сеть вызывается 100 раз в секунду — порядка
//  0.3 млн умножений в секунду, менее 1 % загрузки. Ядро на полной частоте
//  добавляет 8 logf на выборку.
//
//  int8-вариант вдвое меньше по flash, но состояние передаётся через
//  квантованный тензор, и ошибка накапливается по рекуррентной петле.
//  При таком размере модели выигрыш не стоит риска: на M33 с FPU
//  оставайтесь на float32.
// ============================================================================
