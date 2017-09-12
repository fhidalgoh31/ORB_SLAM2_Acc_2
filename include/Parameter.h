#ifndef PARAMETER_H
#define PARAMETER_H

#include <vector>
#include <pangolin/pangolin.h>
#include <boost/variant.hpp>
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>

namespace ORB_SLAM2 {

typedef boost::variant<bool, int, float, double> ParameterVariant;

enum class ParameterGroup
{
    GENERAL,
    VISUAL,
    ORBEXTRACTOR,
    INITIALIZATION,
    TRACKING,
    RELOCALIZATION,
    LOCAL_MAPPING,
    LOOP_CLOSING,
    UNDEFINED
};

class ParameterBase
{
protected:
    enum ParameterCategory
    {
        MINMAX,
        BOOL,
        TEXTINPUT,
        UNDEFINED
    };

    static std::map<ParameterGroup, std::map<std::string, ParameterBase*> > parametersMap;

public:
    virtual ~ParameterBase(){};
    virtual const ParameterVariant getVariant() const { return 0.0; };
    virtual void setValueInternal(const bool& value){} ;
    virtual void setValueInternal(const int& value){} ;
    virtual void setValueInternal(const float& value){} ;
    virtual void setValueInternal(const double& value){} ;
    virtual const ParameterVariant getMinValue() const { return 0.0; };
    virtual const ParameterVariant getMaxValue() const { return 0.0; };
    virtual const ParameterCategory getCategory() const { return ParameterCategory::UNDEFINED; };
    virtual const std::string getName() const { return "undefined"; };
    virtual const ParameterGroup getGroup() const { return ParameterGroup::UNDEFINED; };
};


//TODO : add comments!
template <typename T>
class Parameter : public ParameterBase
{
public:

    Parameter(){};

    // if toggle is set to false this will create a button, otherwise a switch
    Parameter(const std::string& name, const bool& value, const bool& toggle,
              const ParameterGroup& group)
        : mCategory(ParameterCategory::BOOL)
        , mMinValue(0)
        , mMaxValue(toggle)
        , mValue(value)
        , mName(name)
        , mGroup(group)
    {
        std::map<std::string, ParameterBase*>::iterator found_it = parametersMap[group].find(name);
        if(found_it != parametersMap[group].end())
        {
            DLOG(WARNING) << "Duplicate parameter: " << name;
        }
        parametersMap[group][name] = this;
    }

    Parameter(const std::string& name, const T& value, const T& minValue,
              const T& maxValue, const ParameterGroup& group)
        : mCategory(ParameterCategory::MINMAX)
        , mMinValue(minValue)
        , mMaxValue(maxValue)
        , mValue(value)
        , mName(name)
        , mGroup(group)
    {
        std::map<std::string, ParameterBase*>::iterator found_it = parametersMap[group].find(name);
        if(found_it != parametersMap[group].end())
        {
            DLOG(WARNING) << "Duplicate parameter: " << name;
        }
        parametersMap[group][name] = this;
    }

    Parameter(const std::string& name, const T& value, const ParameterGroup& group)
        : mCategory(ParameterCategory::TEXTINPUT)
        , mMinValue(0)
        , mMaxValue(0)
        , mValue(value)
        , mName(name)
        , mGroup(group)
    {
        std::map<std::string, ParameterBase*>::iterator found_it = parametersMap[group].find(name);
        if(found_it != parametersMap[group].end())
        {
            DLOG(WARNING) << "Duplicate parameter: " << name;
        }
        parametersMap[group][name] = this;
    }

    virtual ~Parameter()
    {
        DLOG(WARNING) << "Parameter being deleted: " << mName;
        parametersMap[mGroup][mName] = nullptr;
    };

    const T& getValue() const { return mValue; };
    virtual void setValue(const T& value) { mValue = value; mChangedExternally = true; };
    const bool checkAndResetIfChanged()
    {
        if(mChangedThroughPangolin)
        {
            mChangedThroughPangolin = false;
            return true;
        }
        else
        {
            return false;
        }
    };

    const T operator()()
    {
        return getValue();
    }

    const ParameterCategory mCategory;
    const T mMinValue;
    const T mMaxValue;

protected:
    friend class ParameterManager;

    virtual const ParameterVariant getMinValue() const override { return mMinValue; };
    virtual const ParameterVariant getMaxValue() const override { return mMaxValue; };
    virtual const ParameterCategory getCategory() const override { return mCategory; };
    virtual const std::string getName() const override { return mName; };
    virtual const ParameterGroup getGroup() const override { return mGroup; };
    virtual const ParameterVariant getVariant() const override { return mValue; }; // cannot return a const ref because implicitly casted
    virtual void setValueInternal(const T& value) override { mValue = value; };

    T mValue;
    std::string mName;
    ParameterGroup mGroup;
    bool mChangedThroughPangolin;
    bool mChangedExternally;
};


class ParameterManager : public ParameterBase
{
public:

    typedef boost::variant<pangolin::Var<bool>*, pangolin::Var<int>*, pangolin::Var<float>*, pangolin::Var<double>*, pangolin::Var<std::string>* > PangolinVariants;
    typedef std::map<ParameterGroup, std::map<std::string, std::pair<ParameterBase*, PangolinVariants>>> ParameterPairMap;

    static void createPangolinEntries(const std::string& panel_name, ParameterGroup target_group)
    {
        for(std::map<std::string, ParameterBase*>::iterator it = parametersMap[target_group].begin(); it != parametersMap[target_group].end(); it++)
        {
            auto& param = it->second;
            switch (param->getVariant().which())
            {
                case 0: // bool
                    createPangolinEntry<bool>(param, panel_name);
                    break;
                case 1: // int
                    createPangolinEntry<int>(param, panel_name);
                    break;
                case 2: // float
                    createPangolinEntry<float>(param, panel_name);
                    break;
                case 3: // double
                    createPangolinEntry<double>(param, panel_name);
                    break;
            }
        }
    }

    static void updateParameters()
    {
        for(ParameterPairMap::iterator it_groups = pangolinParams.begin(); it_groups != pangolinParams.end(); it_groups++)
        {
            for(std::map<std::string, std::pair<ParameterBase*, PangolinVariants>>::iterator it = it_groups->second.begin(); it != it_groups->second.end(); it++)
            {
                auto& pango_var_variant = it->second.second;
                auto& param = it->second.first;

                const int& type = param->getVariant().which();

                if (type == 0) // bool
                {
                    updateValue<bool>(param, pango_var_variant);
                }
                else if (type == 1) // int
                {
                    updateValue<int>(param, pango_var_variant);
                }
                else if (type == 2) // float
                {
                    updateValue<float>(param, pango_var_variant);
                }
                else if (type == 3) // double
                {
                    updateValue<double>(param, pango_var_variant);
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
            pangolinParams[param->getGroup()][param->getName()] = std::make_pair(param, (new pangolin::Var<T>(
                            panel_name + "." + param->getName(), boost::get<T>(param->getVariant()),
                            boost::get<T>(param->getMaxValue()))));
        }
        else if (param->getCategory() == ParameterCategory::MINMAX)
        {
            pangolinParams[param->getGroup()][param->getName()] = std::make_pair(param, (new pangolin::Var<T>(
                            panel_name + "." + param->getName(), boost::get<T>(param->getVariant()),
                            boost::get<T>(param->getMinValue()),boost::get<T>(param->getMaxValue()))));
        }
        else if (param->getCategory() == ParameterCategory::TEXTINPUT)
        {
            pangolinParams[param->getGroup()][param->getName()] = std::make_pair(param, new pangolin::Var<std::string>(
                        panel_name + "." + param->getName(), std::to_string(boost::get<T>(param->getVariant()))));
        }
    }

    template<typename T>
    static void updateValue(ParameterBase* param, PangolinVariants& pango_var_variant)
    {
        // If the ParameterCategory is TEXTINPUT the pangolin::var needs to be cast
        // from a string to the type value holds in the parameter, this is done here
        // through a boost::lexical_cast. All other cases are managed by the else block
        auto& param_value = boost::get<T>(param->getVariant());

        if (param->getCategory() == ParameterCategory::TEXTINPUT)
        {
            auto& pango_var = boost::get<pangolin::Var<std::string>* >(pango_var_variant);
            T pango_var_value = boost::lexical_cast<T>(pango_var->Get());

            if(static_cast<Parameter<T>* >(param)->mChangedExternally)
            {
                pango_var->operator=(std::to_string(param_value));
                static_cast<Parameter<T>* >(param)->mChangedExternally = false;
            }
            if(pango_var_value != param_value)
            {
                param->setValueInternal(pango_var_value);
                static_cast<Parameter<T>* >(param)->mChangedThroughPangolin = true;
                DLOG(INFO) << "Parameter value of " << param->getName() <<" is: " << param_value;
            }
        }
        else
        {
            auto& pango_var = boost::get<pangolin::Var<T>* >(pango_var_variant);

            if(pango_var->Get() != param_value)
            {
                if(static_cast<Parameter<T>* >(param)->mChangedExternally)
                {
                    pango_var->operator=(param_value);
                    static_cast<Parameter<T>* >(param)->mChangedExternally = false;
                    DLOG(INFO) << "Parameter value of " << param->getName() <<" is: " << param_value;
                }
                else
                {
                    param->setValueInternal(pango_var->Get());
                    static_cast<Parameter<T>* >(param)->mChangedThroughPangolin = true;
                    DLOG(INFO) << "Parameter value of " << param->getName() <<" is: " << param_value;
                }
            }
        }
    }

    static ParameterPairMap pangolinParams;
};
}

#endif  // PARAMETER_H
