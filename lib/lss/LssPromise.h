//
// Created by guru on 11/19/19.
//

#pragma once

#include "LynxmotionLSS-Config.h"

#include <functional>
#include <memory>

typedef enum { Unresolved, Resolved, Rejected } PromiseState;

template <class T, class E=T>
class LssPromise {
private:
    class Data {
    public:
        T* result;
        E* error;
        PromiseState state;

        // attached functions
        std::function<void(T&)> _then;
        std::function<void(T&)> _otherwise;

        inline Data() : result(nullptr), error(nullptr), state(Unresolved) {}
        //inline Data(T& _set) : result(new T(_set)), error(nullptr), state(Unresolved) {}

        // no copy constructors needed
        Data(const Data&) = delete;
        Data& operator=(const Data&) = delete;

        ~Data() {
            if(result)
                delete result;
            if(error)
                delete error;
        }
    };
    std::shared_ptr<Data> data;

public:
    LssPromise() : data(std::make_shared<Data>()) {}
    LssPromise(const LssPromise& copy) = default;
    LssPromise(LssPromise&& mv) = default;
    LssPromise& operator=(const LssPromise& copy) = default;
    LssPromise& operator=(LssPromise&& mv) = default;

    inline bool operator ==(const LssPromise& rhs) const { return data == rhs.data; }
    inline bool operator !=(const LssPromise& rhs) const { return data != rhs.data; }

    inline LssPromise& then(std::function<void(T&)>&& f) {
        data->_then = f;
        if(data->state == Resolved)
            data->_then(*data->result);    // fire immediately
        return *this;
    }

    inline LssPromise& otherwise(std::function<void(T&)>&& f) {
        data->_otherwise = f;
        if(data->state == Rejected)
            data->_otherwise(*data->result);    // fire immediately
        return *this;
    }

    inline LssPromise& regardless(std::function<void(T&)>&& f) {
        data->_then = f;
        data->_otherwise = f;
        if(data->state != Unresolved)
            data->_then(*data->result);    // fire immediately
        return *this;
    }

    inline void resolve(T& result) {
        data->state = Resolved;
        if(data->_then)
            data->_then(result);
    }

    inline void reject(E& err) {
        data->state = Rejected;
        if(data->_otherwise)
            data->_otherwise(err);
    }

};

