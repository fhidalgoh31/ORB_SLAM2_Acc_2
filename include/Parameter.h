#ifndef PARAMETER_H
#define PARAMETER_H

#include <memory>
#include <vector>
#include <pangolin/pangolin.h>
#include <boost/variant.hpp>

namespace ORB_SLAM2 {

// For new types of parameters add type here, a setValue below and all the stuff in ParameterManager
typedef boost::variant<int, double, bool> ParameterTypes;

class ParameterBase {
 protected:
  enum ParameterCategory{
     MINMAX,
     BOOL,
     UNDEFINED
  };

  static std::vector<ParameterBase*> parameters;

 public:
  virtual ~ParameterBase(){};
  virtual const ParameterTypes getVariant() const { return 0.0; };
  virtual void setValue(const int& value){} ;
  virtual void setValue(const double& value){} ;
  virtual void setValue(const bool& value){} ;
  virtual const int getMinValue() const { return 0.0; };
  virtual const int getMaxValue() const { return 0.0; };
  virtual const ParameterCategory getCategory() const { return ParameterCategory::UNDEFINED; };
  virtual const std::string getName() const { return "undefined"; };
};


template <typename T>
class Parameter : public ParameterBase {
 public:

  Parameter(const std::string& name, const T& value)
      : mCategory(ParameterCategory::BOOL)
      , minValue(0)
      , maxValue(0)
      , mValue(value)
      , mName(name)
  {
    parameters.push_back(this);
  }

  Parameter(const std::string& name, const T& value, const int& minValue, const int& maxValue)
      : mCategory(ParameterCategory::MINMAX)
      , minValue(minValue)
      , maxValue(maxValue)
      , mValue(value)
      , mName(name)
  {
    parameters.push_back(this);
  }

  virtual ~Parameter(){};

  virtual const T& getValue() const { return mValue; };
  virtual const ParameterTypes getVariant() const override { return mValue; }; // cannot return a const ref because implicitly casted
  virtual void setValue(const T& value) override { mValue = value; };
  virtual const int getMinValue() const override { return minValue; };
  virtual const int getMaxValue() const override { return maxValue; };
  virtual const ParameterCategory getCategory() const override { return mCategory; };
  virtual const std::string getName() const override { return mName; };

  const ParameterCategory mCategory;
  int minValue;
  int maxValue;

 protected:
  T mValue;
  std::string mName;
};


class ParameterManager : public ParameterBase {
  public:
  typedef boost::variant<pangolin::Var<int>*, pangolin::Var<double>*, pangolin::Var<bool>*> PangolinVariants;
  typedef std::map<std::string, std::pair<ParameterBase*, PangolinVariants>> ParameterPairMap;

  static ParameterPairMap& createPangolinEntries(const std::string& panel_name)
  {
    for(const auto& param : parameters)
    {
      switch (param->getVariant().which()) {
        case 0: // int
          createPangolinEntry<int>(param, panel_name);
          break;
        case 1: // double
          createPangolinEntry<double>(param, panel_name);
          break;
        case 2: // bool
          createPangolinEntry<bool>(param, panel_name);
          break;
      }
    }
    return pangolinParams;
  }

  static void updateParameters()
  {
    for(ParameterPairMap::iterator it = pangolinParams.begin(); it != pangolinParams.end(); it++)
    {
      auto& pango_var_variant = it->second.second;
      auto& param = it->second.first;

      const int& type = param->getVariant().which();

      if (type == 0) // int
      { 
        auto& pango_var =  boost::get<pangolin::Var<int>* >(pango_var_variant);
        auto& value = boost::get<int>(param->getVariant());
        if(pango_var->Get() != value)
        {
          param->setValue(pango_var->Get());
          std::cout << "param value of " << param->getName() <<" is: " << boost::get<int>(param->getVariant()) << std::endl;
        }
      }
      else if (type == 1) // double
      {
        auto& pango_var =  boost::get<pangolin::Var<double>* >(pango_var_variant);
        auto& value = boost::get<double>(param->getVariant());
        if(pango_var->Get() != value)
        {
          param->setValue(pango_var->Get());
          std::cout << "param value of " << param->getName() <<" is: " << boost::get<double>(param->getVariant()) << std::endl;
        }
      }
      else if (type == 2) // bool
      {
        auto& pango_var =  boost::get<pangolin::Var<bool>* >(pango_var_variant);
        auto& value = boost::get<bool>(param->getVariant());
        if(pango_var->Get() != value)
        {
          param->setValue(pango_var->Get());
          std::cout << "param value of " << param->getName() <<" is: " << boost::get<bool>(param->getVariant()) << std::endl;
        }
      }
    }
  }

  private:
  template<typename T>
  static void createPangolinEntry(ParameterBase* param, const std::string& panel_name)
  {
    if (param->getCategory() == ParameterCategory::BOOL)
    {
      pangolinParams[param->getName()] = std::make_pair(param, (new pangolin::Var<T>(
              panel_name + "." + param->getName(), boost::get<T>(param->getVariant()))));
    }
    else if (param->getCategory() == ParameterCategory::MINMAX)
    {
      pangolinParams[param->getName()] = std::make_pair(param, (new pangolin::Var<T>(
              panel_name + "." + param->getName(), boost::get<T>(param->getVariant()),
              param->getMinValue(),param->getMaxValue())));
    }
  }

  static ParameterPairMap pangolinParams;
};
}

#endif  // PARAMETER_H
