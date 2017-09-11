#include "Parameter.h"

namespace ORB_SLAM2
{
    // static variables
    std::map<ParameterGroup, std::map<std::string, ParameterBase*> > ParameterBase::parametersMap;
    ParameterManager::ParameterPairMap ParameterManager::pangolinParams;
}
