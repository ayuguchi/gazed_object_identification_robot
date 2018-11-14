#ifndef COMBI_DARKNET_OPENFACE_CACHED_VALUE_H
#define COMBI_DARKNET_OPENFACE_CACHED_VALUE_H

template<typename T>
class CachedValue
{
private:
    T cache;
    T invalid_value;
    bool initialized = false;
public:
    CachedValue(const T& invalid_value):invalid_value(invalid_value){}
    T update(const T& current_value)
    {
        if(!this->initialized)
        {
            this->cache = current_value;
            this->initialized = true;
        }
        if(current_value != this->invalid_value)
        {
            this->cache = current_value;
        }
        return this->cache;
    }
    T value() const
    {
        return this->cache;
    }
};

#endif
