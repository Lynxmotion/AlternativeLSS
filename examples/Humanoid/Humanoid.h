//
// Created by guru on 11/20/19.
//

#pragma once

#include <LssServo.h>

#include <array>
#include <memory>

class Coord {
public:
    double x, y, z, w;

    inline Coord(double _x=0.0, double _y=0.0, double _z==0.0, double _w==0.0) : x(_x), y(_y), z(_z), w(_w) {}
    Coord(const Coord& copy) : x(copy.x),y(copy.y),z(copy.z),w(copy.w) {}
};

class Joint {
public:
    class Data {
    public:
        LynxServo servo;
        Joint parent;
        Coord position;

        Data();
        Data(short ServoID, Joint parent, Coord offset);
    }

    inline Joint() : std::make_shared<Data>() {}
    inline Joint(short ServoID, Join parent) : std::make_shared<Data>(ServoID, parent) {}

private:
    std::shared_ptr<Data> data;
};

// a collection of Joints that form any appendage or body part
class Appendage {
public:
    std::array<Servo> servos;

    const LynxServo& operator[](int idx) const;
    const LynxServo& operator[](int idx) const;

};


class Arm {
public:

};


