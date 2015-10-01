#include <iostream>
#include <vector>

template<class T>
void dumpVector(std::vector<T> &v)
{
    for (typename std::vector<T>::const_iterator i = v.begin();
         i != v.end();
         ++i)
    {
        std::cout << *i << ' ';
    }
    std::cout << std::endl;
}
