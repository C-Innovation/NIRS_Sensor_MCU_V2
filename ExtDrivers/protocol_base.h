/*
 * protocol_base.h
 *
 *  Created on: 24 мар. 2026 г.
 *      Author: ivanov.y
 */

#ifndef INC_PROTOCOL_BASE_H_
#define INC_PROTOCOL_BASE_H_

#include "ring_buffer.h"
static const char* TAG2 = "protocol_base.cpp";

typedef struct
{
	uint8_t data[256];
	size_t len;
}Buffer_t;
// ==============================
// Сериализатор
// ==============================
class Serializer {
public:
    // Заголовок — 4 байта, передаётся в конструкторе.
    Serializer(uint32_t header) : header_(header) {}

    // Упаковка данных в буфер out_buf размером out_buf_size.
    // Возвращает размер пакета (0, если out_buf слишком мал).
    size_t serialize(const uint8_t* data, size_t data_len,
                     uint8_t* out_buf, size_t out_buf_size) const {
        size_t packet_len = 4 + 2 + data_len + 2;
        if (out_buf_size < packet_len) return 0;

        // Заголовок (4 байта, little-endian)
        out_buf[0] = static_cast<uint8_t>(header_ >> 0);
        out_buf[1] = static_cast<uint8_t>(header_ >> 8);
        out_buf[2] = static_cast<uint8_t>(header_ >> 16);
        out_buf[3] = static_cast<uint8_t>(header_ >> 24);

        // Длина данных (2 байта, little-endian)
        uint16_t len16 = static_cast<uint16_t>(data_len);
        out_buf[4] = static_cast<uint8_t>(len16 >> 0);
        out_buf[5] = static_cast<uint8_t>(len16 >> 8);

        // Данные
        memcpy(out_buf + 6, data, data_len);

        // Контрольная сумма (16-битная сумма байт)
        uint16_t checksum = compute_checksum(out_buf, packet_len - 2);
        out_buf[packet_len - 2] = static_cast<uint8_t>(checksum >> 0);
        out_buf[packet_len - 1] = static_cast<uint8_t>(checksum >> 8);

        return packet_len;
    }

private:
    static uint16_t compute_checksum(const uint8_t* data, size_t len) {
//        uint16_t sum = 0;
//        for (size_t i = 0; i < len; ++i) {
//            sum += data[i];
//        }
//        return sum;
    	uint16_t crc = 0x0000;
		for (size_t i = 0; i < len; i++) {
			crc ^= (uint16_t)data[i] << 8;
			for (int bit = 0; bit < 8; bit++) {
				if (crc & 0x8000) {
					crc = (crc << 1) ^ 0x8005;
				} else {
					crc <<= 1;
				}
			}
		}
		return crc;
    }

    uint32_t header_;
};

// ==============================
// Десериализатор
// ==============================
class Deserializer {
public:
    // Тип функции обратного вызова: поле данных и его размер.
    using PacketCallback = void (*)(const uint8_t* data, size_t len);

    Deserializer(RingBuffer& buffer, uint32_t expected_header, PacketCallback cb)
        : buffer_(buffer)
        , expected_header_(expected_header)
        , callback_(cb)
    {}

    // Обработать все возможные пакеты в буфере.
    Buffer_t process() {
    	Buffer_t data_buf;
    	data_buf.len = 0;
//        while (buffer_.available() >= 8) { // Минимум: заголовок(4) + длина(2) + КС(2)
    	if(buffer_.available() < 8)
    			return data_buf;
            // Ищем заголовок в буфере
            size_t header_pos = find_header();
            if (header_pos == static_cast<size_t>(-1)) {
                // Заголовок не найден, удаляем все байты, кроме последних 3
                // (на случай, если начало заголовка попало на границу)
                size_t avail = buffer_.available();
                if (avail > 3) {
                    buffer_.remove(avail - 3);
                }
                return data_buf;
            }

            // Удаляем байты до заголовка
            if (header_pos > 0) {
                buffer_.remove(header_pos);
            }

            // Теперь заголовок в начале буфера. Проверим, есть ли полный пакет.
            // Читаем длину поля данных из байтов 4 и 5.
            uint16_t data_len = static_cast<uint16_t>(buffer_.peek(4)) |
                                (static_cast<uint16_t>(buffer_.peek(5)) << 8);

            size_t total_packet_len = 4 + 2 + data_len + 2;

            // Проверка на разумный размер (например, не больше половины буфера)
            const size_t MAX_PACKET_SIZE = 1024; // можно настроить
            if (data_len > MAX_PACKET_SIZE) {
                // Некорректная длина, пропускаем этот "заголовок"
                buffer_.remove(1);
                return data_buf;
            }

            if (buffer_.available() < total_packet_len) {
                // Неполный пакет, ждём новых данных
            	return data_buf;
            }

            // Вычисляем контрольную сумму
            uint16_t calc_crc = compute_checksum_from_buffer(total_packet_len - 2);
            uint16_t recv_crc = static_cast<uint16_t>(buffer_.peek(total_packet_len - 2)) |
                                (static_cast<uint16_t>(buffer_.peek(total_packet_len - 1)) << 8);

            if (calc_crc != recv_crc) {
                // Контрольная сумма не сошлась, пропускаем этот заголовок
                buffer_.remove(1);
                return data_buf;
            }

            // Пакет корректен, извлекаем поле данных
            // Создаём временный буфер для данных (можно передать указатель на стековый массив в callback,
            // но для безопасности лучше выделить память, если данные большие. Здесь используем небольшой
            // стековый массив, но в реальном проекте можно передать callback'у ссылку на RingBuffer
            // и смещение, чтобы избежать копирования. Для простоты копируем.
             // Максимальный размер поля данных (можно настроить)

            if (data_len > sizeof(data_buf)) {
                // Слишком большой пакет — не обрабатываем (можно пропустить)
                buffer_.remove(1);
                return data_buf;
            }

            // Копируем поле данных из буфера (начиная с позиции 6)
            for (size_t i = 0; i < data_len; ++i) {
                data_buf.data[i] = buffer_.peek(6 + i);
            }
            data_buf.len = data_len;
            // Вызываем callback
//            if (callback_) {
//                callback_(data_buf, data_len);
//            }



            // Удаляем весь пакет из буфера
            buffer_.remove(total_packet_len);
            return data_buf;
//        }
    }

private:
    // Поиск заголовка в буфере, возвращает смещение от tail или -1, если не найден.
    size_t find_header() const {
        size_t avail = buffer_.available();
        if (avail < 4) return static_cast<size_t>(-1);

        // Сканируем все возможные позиции
        for (size_t offset = 0; offset <= avail - 4; ++offset) {
            uint32_t val = static_cast<uint32_t>(buffer_.peek(offset)) |
                           (static_cast<uint32_t>(buffer_.peek(offset + 1)) << 8) |
                           (static_cast<uint32_t>(buffer_.peek(offset + 2)) << 16) |
                           (static_cast<uint32_t>(buffer_.peek(offset + 3)) << 24);
            if (val == expected_header_) {
                return offset;
            }
        }
        return static_cast<size_t>(-1);
    }

    // Вычисляет контрольную сумму по первым len байтам буфера (начиная с текущего tail).
    uint16_t compute_checksum_from_buffer(size_t len) const {
//        uint16_t sum = 0;
//        for (size_t i = 0; i < len; ++i) {
//            sum += buffer_.peek(i);
//        }
//        return sum;

		uint16_t crc = 0x0000;
		for (size_t i = 0; i < len; i++) {
			crc ^= (uint16_t)buffer_.peek(i) << 8;
			for (int bit = 0; bit < 8; bit++) {
				if (crc & 0x8000) {
					crc = (crc << 1) ^ 0x8005;
				} else {
					crc <<= 1;
				}
			}
		}
		return crc;
    }

    RingBuffer& buffer_;
    uint32_t expected_header_;
    PacketCallback callback_;
};

#endif /* INC_PROTOCOL_BASE_H_ */
