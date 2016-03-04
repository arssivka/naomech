//
// Created by arssivka on 3/1/16.
//

#ifndef NAOMECH_TYPEDEFINITION_H
#define NAOMECH_TYPEDEFINITION_H

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

typedef std::vector<int> IntegerKeyVector;
typedef std::vector<std::string> StringKeyVector;
typedef std::vector<double> ValuesVector;
typedef boost::shared_ptr<std::vector<double> > ValuesVectorPtr;

#endif //NAOMECH_TYPEDEFINITION_H
