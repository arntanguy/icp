//  This file is part of the Icp Library,
//
//  Copyright (C) 2014-2015 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

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
