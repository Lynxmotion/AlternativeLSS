//
// Created by guru on 11/19/19.
//

#pragma once

#include "LynxmotionLSS-Config.h"

#include <functional>
#include <memory>
#include <cassert>

typedef enum { Unresolved, Resolved, Rejected } PromiseState;

template <class T, class E=T>
class LssPromise {
private:
    class Data {
    public:
        // temp storage of result or error
        std::shared_ptr<T> result;
        std::shared_ptr<E> error;
        PromiseState state;

        // attached functions
        std::function<void(T&)> _then;
        std::function<void(T&)> _otherwise;

        inline Data() : result(nullptr), error(nullptr), state(Unresolved) {}

        // no copy constructors needed
        Data(const Data&) = delete;
        Data& operator=(const Data&) = delete;
    };
    std::shared_ptr<Data> data;

public:
    LssPromise() : data(std::make_shared<Data>()) {}
    LssPromise(const LssPromise& copy) = default;
    LssPromise(LssPromise&& mv) noexcept = default;
    LssPromise& operator=(const LssPromise& copy) = default;
    LssPromise& operator=(LssPromise&& mv) noexcept = default;

    inline bool operator ==(const LssPromise& rhs) const { return data == rhs.data; }
    inline bool operator !=(const LssPromise& rhs) const { return data != rhs.data; }

    inline LssPromise& then(std::function<void(T&)>&& f) {
        data->_then = f;
        if(data->state == Resolved) {
            assert(data->result);
            data->_then(*data->result);    // fire immediately
        }
        return *this;
    }

    inline LssPromise& otherwise(std::function<void(T&)>&& f) {
        data->_otherwise = f;
        if(data->state == Rejected) {
            assert(data->error);
            data->_otherwise(*data->error);    // fire immediately
        }
        return *this;
    }

    inline LssPromise& regardless(std::function<void(T&)>&& f) {
        data->_then = f;
        data->_otherwise = f;
        if(data->state != Unresolved) {
            assert(data->result);
            data->_then(*data->result);    // fire immediately
        }
        return *this;
    }

    inline void resolve(T& result) {
        data->state = Resolved;
        if(data->_then)
            data->_then(result);
        else
            assert(false); // no handler yet and cannot store a reference (use shared ptrs)
    }

    inline void reject(E& err) {
        data->state = Rejected;
        if(data->_otherwise)
            data->_otherwise(err);
        else
            assert(false); // no handler yet and cannot store a reference (use shared ptrs)
    }


    inline void resolve(std::shared_ptr<T> result) {
        data->state = Resolved;
        if(data->_then)
            data->_then(*result);
        else
            data->result = result; // no handler yet, store for later firing
    }

    inline void reject(std::shared_ptr<E> err) {
        data->state = Rejected;
        if(data->_otherwise)
            data->_otherwise(*err);
        else
            data->error = err; // no handler yet, store for later firing
    }

};

