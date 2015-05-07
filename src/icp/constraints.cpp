#include <algorithm>
#include "constraints.hpp"
#include "instanciate.hpp"


namespace icp
{

int FixTranslationConstraint::numFixedAxes() const 
{
    return std::count(std::begin(fixedAxes_), std::end(fixedAxes_), true);
}

INSTANCIATE_CONSTRAINTS;

}  // namespace icp
