#ifndef __GESTOOS_SUPPORT_H__
#define __GESTOOS_SUPPORT_H__

#include <iostream>
#include <map>
#include <string>
#include <vector>

typedef std::map<int,std::string> idNameMap;

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

void initializeMap(const std::vector<int> &labels,
                   const std::vector<std::string> &names,
                   idNameMap &nameContainer);

#endif // __GESTOOS_SUPPORT_H__
