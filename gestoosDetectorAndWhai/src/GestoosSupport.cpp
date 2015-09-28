#include <iostream>
#include <vector>

void dumpDetectorLabels(std::vector<int> &labels)
{
    std::cout << "gesture detector labels: ";
    for (std::vector<int>::const_iterator i = labels.begin();
         i != labels.end();
         ++i)
    {
        std::cout << *i << ' ';
    }
    std::cout << std::endl;
}
