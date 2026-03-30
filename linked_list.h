#ifndef LINKED_LIST_H
#define LINKED_LIST_H

#include <cstdint>
#include <cstddef>
#include <array>

// Template-based static linked list

template <typename T, std::size_t S>
class LinkedList {
private:
    std::array<bool, S> allocated{};
    std::array<T, S> value;
    std::array<int16_t, S> next{};
    int16_t head = -1;

    int16_t allocate(const T& val) {
        for (std::size_t i = 0; i < S; ++i) {
            if (!allocated[i]) {
                allocated[i] = true;
                value[i] = val;
                next[i] = -1;
                return static_cast<int16_t>(i);
            }
        }
        return -1; // Full
    }

    void deallocate(int16_t i) {
        if (i >= 0 && i < static_cast<int16_t>(S)) {
            allocated[i] = false;
            next[i] = -1;
        }
    }

public:
    T null_element{};
    void init() {
        allocated.fill(false);
        next.fill(-1);
        head = -1;
    }

    bool empty() const {
        return head == -1;
    }

    bool full() const {
        for (bool a : allocated)
            if (!a) return false;
        return true;
    }

    std::size_t length() const {
        std::size_t count = 0;
        int16_t it = head;
        while (it != -1) {
            ++count;
            it = next[it];
        }
        return count;
    }

    T& get(std::size_t index) {
        int16_t it = head;
        std::size_t count = 0;
        while (it != -1) {
            if (count == index) return value[it];
            it = next[it];
            ++count;
        }
        return null_element;
    }

    const T& get(std::size_t index) const {
        int16_t it = head;
        std::size_t count = 0;
        while (it != -1) {
            if (count == index) return value[it];
            it = next[it];
            ++count;
        }
        return null_element;
    }

    int16_t prepend(const T& val) {
        int16_t i = allocate(val);
        if (i == -1) return -1; // List full
        if (!empty()) next[i] = head;
        head = i;
        return i;
    }

    int16_t append(const T& val) {
        int16_t i = allocate(val);
        if (i == -1) return -1; // List full

        if (empty()) {
            head = i;
        } else {
            int16_t it = head;
            while (next[it] != -1) {
                it = next[it];
            }
            next[it] = i;
        }

        return i;
    }

    void removeFromHead() {
        if (empty()) return;
        int16_t oldHead = head;
        head = next[head];
        deallocate(oldHead);
    }

    void removeFromTail() {
        if (empty()) return;

        if (next[head] == -1) {
            deallocate(head);
            head = -1;
            return;
        }

        int16_t prev = head;
        int16_t it = next[head];

        while (next[it] != -1) {
            prev = it;
            it = next[it];
        }

        next[prev] = -1;
        deallocate(it);
    }

    void removeFromIndex(std::size_t index) {
        if (empty()) return;

        // Special case: remove head
        if (index == 0) {
            removeFromHead();
            return;
        }

        std::size_t count = 1;
        int16_t prev = head;
        int16_t it = next[head];

        while (it != -1 && count < index) {
            prev = it;
            it = next[it];
            ++count;
        }

        if (it == -1 || !allocated[it]) return;

        next[prev] = next[it];
        deallocate(it);
    }

    void insertBefore(int16_t i, const T& val) {
        if (i == head) {
            prepend(val);
            return;
        }

        int16_t prev = head;
        int16_t it = next[head];

        while (it != -1 && it != i) {
            prev = it;
            it = next[it];
        }

        if (it == -1) return; // Index not found

        int16_t newI = allocate(val);
        if (newI == -1) return; // Full

        next[newI] = it;
        next[prev] = newI;
    }

    void insertAfter(int16_t i, const T& val) {
        if (i < 0 || i >= static_cast<int16_t>(S) || !allocated[i]) return;
        int16_t newI = allocate(val);
        if (newI == -1) return;
        next[newI] = next[i];
        next[i] = newI;
    }

    void swapNodes(std::size_t index1, std::size_t index2) {
        if (index1 == index2) return;

        int16_t it1 = head, it2 = head;
        std::size_t count1 = 0, count2 = 0;

        while (it1 != -1 && count1 < index1) {
            it1 = next[it1];
            ++count1;
        }
        while (it2 != -1 && count2 < index2) {
            it2 = next[it2];
            ++count2;
        }

        std::swap(value[it1], value[it2]);
    }

    // Iterator support
    class Iterator {
    private:
        LinkedList* list;
        int16_t index;
    public:
        Iterator(LinkedList* lst, int16_t idx) : list(lst), index(idx) {}

        T& operator*() const { return list->value[index]; }
        T* operator->() const { return &list->value[index]; }

        Iterator& operator++() {
            index = list->next[index];
            return *this;
        }

        bool operator!=(const Iterator& other) const {
            return index != other.index;
        }
    };

    Iterator begin() { return Iterator(this, head); }
    Iterator end() { return Iterator(this, -1); }
};


#endif
