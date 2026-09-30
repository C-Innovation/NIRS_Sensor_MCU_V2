/*
 * ring_buffer.h
 *
 *  Created on: 24 мар. 2026 г.
 *      Author: ivanov.y
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_


#include <cstdint>
#include <cstring>
#include <algorithm>
// ==============================
// Кольцевой буфер
// ==============================
class RingBuffer {
public:
    explicit RingBuffer(uint32_t capacity)
        : capacity_(capacity)
        , buffer_(new uint8_t[capacity_])
        , head_(0)
        , tail_(0)
    {}

    ~RingBuffer() {
        delete[] buffer_;
    }

    // Запись данных. Возвращает количество записанных байт.
    uint32_t write(const uint8_t* data, uint32_t len) {
      uint32_t free_space = free();
        if (free_space == 0) return 0;
        uint32_t write_len = std::min(len, free_space);

        uint32_t first_part = std::min(write_len, capacity_ - head_);
        memcpy(buffer_ + head_, data, first_part);
        memcpy(buffer_, data + first_part, write_len - first_part);
        head_ = (head_ + write_len) % capacity_;
        return write_len;
    }

    // Чтение данных с удалением. Возвращает количество прочитанных байт.
    uint32_t read(uint8_t* dest, uint32_t len) {
      uint32_t avail = available();
        if (avail == 0) return 0;
        uint32_t read_len = std::min(len, avail);

        uint32_t first_part = std::min(read_len, capacity_ - tail_);
        memcpy(dest, buffer_ + tail_, first_part);
        memcpy(dest + first_part, buffer_, read_len - first_part);
        tail_ = (tail_ + read_len) % capacity_;
        return read_len;
    }

    // Просмотр байта на позиции offset (относительно tail) без удаления.
    // Возвращает байт или 0, если offset >= available().
    uint8_t peek(uint32_t offset) const {
        if (offset >= available()) return 0;
        uint32_t pos = (tail_ + offset) % capacity_;
        return buffer_[pos];
    }

    // Удаляет len байт из начала.
    bool remove(uint32_t len) {
        if (len > available()) return false;
        tail_ = (tail_ + len) % capacity_;
        return true;
    }

    // Количество доступных для чтения байт.
    uint32_t available() const {
        if (head_ >= tail_)
            return head_ - tail_;
        else
            return capacity_ - tail_ + head_;
    }

    // Количество свободных байт.
    uint32_t free() const {
        return capacity_ - available() - 1; // -1 чтобы отличать полный от пустого
    }

    bool is_empty() const { return head_ == tail_; }
    bool is_full()  const { return (head_ + 1) % capacity_ == tail_; }

private:
    uint32_t capacity_;
    uint8_t* buffer_;
    uint32_t head_;
    uint32_t tail_;
};



#endif /* INC_RING_BUFFER_H_ */
