#ifndef RAF_h
#define RAF_h

#include <stdlib.h>
#include <cassert>

template<typename T>
class MovingAverage
{
public:
	MovingAverage(unsigned char _size, T init_value=0)
	{
		size = _size;  
		history = (T*)malloc(size*sizeof(T));
		for(int i=0; i<size; i++)
		  history[i] = init_value;
		p = 0;
		acc = init_value*size;
	}

	MovingAverage(const MovingAverage& copy) : size(copy.size), p(copy.p), acc(copy.acc)
	{
		history = (T*)malloc(size*sizeof(T));
		for(int i=0; i<size; i++)
		  history[i] = copy.history[i];
	}

	MovingAverage& operator=(const MovingAverage& copy)
	{
		size=copy.size;
		p=copy.p;
		acc=copy.acc;
		history = (T*)malloc(size*sizeof(T));
		for(int i=0; i<size; i++)
		  history[i] = copy.history[i];
		return *this;
	}

	~MovingAverage()
	{
		free(history);
	}

	void clear(T init_value)
	{
		for(int i=0; i<size; i++)
		  history[i] = init_value;
		acc = init_value * size;
		p = 0;
	}

	inline void clear() { clear(0); }

	void add(T value)
	{
	    assert(size>0);
		if(++p >= size)
			p = 0;
		acc -= history[p];
		history[p] = value;
		acc += value;
	}
	
	T average() const
	{
		return (size>0) ? acc / size : 0;
	}

    inline T sum() const { return acc; }

    T recent() const
	{
		return history[p];
	}

	void minmax(T& min_out, T& max_out) const
	{
        if(size<=0) return;
        min_out = max_out = history[0];
		for(int i=1; i<size; i++)
        {
            if(min_out > history[i])
                min_out = history[i];
            if(max_out < history[i])
                max_out = history[i];
        }
    }

    // return the range of values, or abs(min-max)
    T spread() const
    {
        T _min, _max;
        minmax(_min, _max);
        return abs(_min - _max);
    }

private:
	T* history;
	unsigned char size;
	T acc;
	unsigned char p;
};




class raf_timer_t : public MovingAverage<clock_t>
{
protected:
    clock_t tsbegin;

public:
	raf_timer_t(unsigned char _size=10)
        : MovingAverage<clock_t>(_size, 0)
	{
    }

    inline void enter()
    {
        tsbegin = clock();
    }

    inline void exit()
    {
        add( clock() - tsbegin );
        tsbegin=0;
    }

};
#endif

