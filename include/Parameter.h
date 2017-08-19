#ifndef PARAMETER_H
#define PARAMETER_H

#include <memory>
#include <vector>
#include <pangolin/pangolin.h>

namespace ORB_SLAM2 {
class ParameterBase {
 public:
  enum ParamType{
     MINMAX,
     BOOL
  };

  virtual ~ParameterBase(){};
  /*! TODO: This shouldn't all be returned as double
   *  \todo This shouldn't all be returned as double
   */
  virtual const double getValue() const = 0;
  virtual void setValue(const double& value) = 0;
  virtual const double getMinValue() const = 0;
  virtual const double getMaxValue() const = 0;
  virtual const double getParamType() const = 0;
  virtual const std::string& getName() const = 0;

 protected:
  static std::vector<ParameterBase*> parameters;
};


template <typename T>
class Parameter : public ParameterBase {
 public:

  Parameter(const std::string& name, const T& value)
      : paramType(ParamType::BOOL)
      , minValue(0)
      , maxValue(0)
      , mValue(value)
      , mName(name)
  {
    parameters.push_back(this);
  }

  Parameter(const std::string& name, const T& value, const T&minValue, const T& maxValue)
      : paramType(ParamType::MINMAX)
      , minValue(minValue)
      , maxValue(maxValue)
      , mValue(value)
      , mName(name)
  {
    parameters.push_back(this);
  }

  virtual ~Parameter(){};

  virtual const double getValue() const override { return mValue; }; // cannot return a const ref because implicitly casted
  virtual void setValue(const double& value) override { mValue = static_cast<T>(value); };
  virtual const double getMinValue() const override { return minValue; };
  virtual const double getMaxValue() const override { return maxValue; };
  virtual const double getParamType() const override { return paramType; };
  virtual const std::string& getName() const override { return mName; };

  const ParamType paramType;
  T minValue;
  T maxValue;

 protected:
  T mValue;
  std::string mName;
};


class ParameterManager : public ParameterBase {
  public:
  typedef std::map<std::string, std::pair<ParameterBase*, pangolin::Var<double>*> > ParameterPairMap;

  static ParameterPairMap& createPangolinEntries(const std::string& panel_name)
  {
    for(auto& param : parameters)
    {
      if (param->getParamType() == ParamType::BOOL)
      {
        pangolinParams[param->getName()] = std::make_pair(param, (new pangolin::Var<double>(
                panel_name + "." + param->getName(), param->getValue())));
      }
      else if (param->getParamType() == ParamType::MINMAX)
      {
        pangolinParams[param->getName()] = std::make_pair(param, (new pangolin::Var<double>(
                panel_name + "." + param->getName(), param->getValue(), param->getMinValue(), param->getMaxValue())));
      }
    }
    return pangolinParams;
  }

  static void updateParameters()
  {
    for(ParameterPairMap::iterator it = pangolinParams.begin(); it != pangolinParams.end(); it++)
    {
      auto& pango_var = it->second.second;
      auto& param = it->second.first;
      if(pango_var->Get() != param->getValue())
      {
        param->setValue(pango_var->Get());
        // std::cout << "param value of " << param->getName() <<" is: " << param->getValue() << std::endl;
      }
    }
  }

  private:
  static ParameterPairMap pangolinParams;
};
}

#endif  // PARAMETER_H
