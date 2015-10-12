/* 
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * This program uses Gestoos SDK by Fezoo Labs:
 *     http://www.gestoos.com/
 *     http://www.fezoo.cat/
 *
 */

#include "WhaiHandTrackerSupport.h"

int posMaxElement(std::vector<double> &v)
{
    if (v.empty())
        return -1;

    // http://stackoverflow.com/a/10159170/1638888
    std::vector<double>::iterator biggest = std::max_element(std::begin(v), std::end(v));
    return std::distance(std::begin(v), biggest);
}
