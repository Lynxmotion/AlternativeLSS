//
// Created by guru on 11/19/19.
//

#pragma once

#include "LynxmotionLSS-Config.h"

#include <functional>
#include <memory>

typedef enum { Unresolved, Resolved, Rejected } PromiseState;

template <class T>
class LssPromise {
private:
    class Data {
    public:
        T set;
        PromiseState state;

        // attached functions
        std::function<void(T&)> _then;
        std::function<void(T&)> _otherwise;

        inline Data() : state(Unresolved) {}
        inline Data(T& _set) : set(_set), state(Unresolved) {}

        // no copy constructors needed
        Data(const Data&) = delete;
        Data& operator=(const Data&) = delete;
    };
    std::shared_ptr<Data> data;

public:
    LssPromise() : data(std::make_shared<Data>()) {}
    LssPromise(T& set) : data(std::make_shared<Data>(set)) {}
    LssPromise(const LssPromise& copy) = default;
    LssPromise(LssPromise&& mv) = default;
    LssPromise& operator=(const LssPromise& copy) = default;
    LssPromise& operator=(LssPromise&& mv) = default;

    inline bool operator ==(const LssPromise& rhs) const { return data == rhs.data; }
    inline bool operator !=(const LssPromise& rhs) const { return data != rhs.data; }

    inline T* operator->() { return &data->set; }

    inline LssPromise& then(std::function<void(T&)>&& f) {
        data->_then = f;
        if(data->state == Resolved)
            data->_then(data->set);    // fire immediately
        return *this;
    }

    inline LssPromise& otherwise(std::function<void(T&)>&& f) {
        data->_otherwise = f;
        if(data->state == Rejected)
            data->_otherwise(data->set);    // fire immediately
        return *this;
    }

    inline LssPromise& regardless(std::function<void(T&)>&& f) {
        data->_then = f;
        data->_otherwise = f;
        if(data->state != Unresolved)
            data->_then(data->set);    // fire immediately
        return *this;
    }

    inline void resolve() const {
        data->state = Resolved;
        if(data->_then)
            data->_then(data->set);
    }

    inline void reject() const {
        data->state = Rejected;
        if(data->_otherwise)
            data->_otherwise(data->set);
    }

};

