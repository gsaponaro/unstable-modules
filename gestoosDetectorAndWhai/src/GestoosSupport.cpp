#include <utility>

#include "GestoosSupport.h"

void initializeMap(const std::vector<int> &labels,
                   const std::vector<std::string> &names,
                   idNameMap &nameContainer)
{
    for (std::vector<int>::const_iterator i = labels.begin();
         i != labels.end();
         ++i)
    {
        int idx = i - labels.begin(); // 0, 1, 2, ...
        //std::cout << "adding new entry: " << *i << " " << names[idx] << std::endl;
        nameContainer.insert(make_pair(*i,names[idx]));
    }
}
