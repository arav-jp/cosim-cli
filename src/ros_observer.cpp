/*
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "ros_observer.hpp"

//#include <cosim/error.hpp>
#include <cosim/log/logger.hpp>

#include <boost/date_time/local_time/local_time.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <algorithm>
#include <codecvt>
#include <fstream>
#include <locale>
#include <map>
#include <mutex>
#include <sstream>
#include <vector>

namespace cosim
{

class ros_observer::slave_value_writer
{
public:
    slave_value_writer(observable* observable, bool timeStampedFileNames = true)
        : observable_(observable)
        , timeStampedFileNames_(timeStampedFileNames)
    {
        initialize_default();

        msg_pub = nh.advertise<cosim_msgs::cosim>("cosim_output", 1);
    }

    slave_value_writer(observable* observable, size_t decimationFactor,
        const std::vector<variable_description>& variables, bool timeStampedFileNames = true)
        : observable_(observable)
        , decimationFactor_(decimationFactor)
        , timeStampedFileNames_(timeStampedFileNames)
    {
        initialize_config(variables);

        msg_pub = nh.advertise<cosim_msgs::cosim>("cosim_output", 1);
    }

    void observe(step_number timeStep, time_point currentTime)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (recording_) {
            if (timeStep % decimationFactor_ == 0) {

                if (!realVars_.empty()) realSamples_[timeStep].reserve(realVars_.size());
                if (!intVars_.empty()) intSamples_[timeStep].reserve(intVars_.size());
                if (!boolVars_.empty()) boolSamples_[timeStep].reserve(boolVars_.size());
                if (!stringVars_.empty()) stringSamples_[timeStep].reserve(stringVars_.size());

                cosim_msgs::cosim cosim_output;
                cosim_output.header.stamp = ros::Time::now();

                timeSamples_[timeStep] = to_double_time_point(currentTime);
                std::cout<<"t: "<<timeSamples_[timeStep]<<" ";
                for (const auto& vd : realVars_) {
                    realSamples_[timeStep].push_back(observable_->get_real(vd.reference));
                    std::cout << vd.name << ": "<<observable_->get_real(vd.reference) << " ";

                    cosim_output.keys.push_back(vd.name);
                    cosim_output.values.push_back(observable_->get_real(vd.reference));
                }
                for (const auto& vd : intVars_) {
                    intSamples_[timeStep].push_back(observable_->get_integer(vd.reference));
                }
                for (const auto& vd : boolVars_) {
                    boolSamples_[timeStep].push_back(observable_->get_boolean(vd.reference));
                }
                for (const auto& vd : stringVars_) {
                    stringSamples_[timeStep].push_back(observable_->get_string(vd.reference));
                }

                std::cout<<std::endl;
                msg_pub.publish(cosim_output);
            }
        }

        ros::spinOnce();
    }

    void start_recording()
    {
        recording_ = true;
    }

    void stop_recording()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        recording_ = false;
        realSamples_.clear();
        intSamples_.clear();
        boolSamples_.clear();
        stringSamples_.clear();
        timeSamples_.clear();
    }

    ~slave_value_writer()
    {
    }

private:

    void initialize_variable(const variable_description& vd)
    {
        observable_->expose_for_getting(vd.type, vd.reference);

        switch (vd.type) {
            case variable_type::real:
                realVars_.push_back(vd);
                break;
            case variable_type::integer:
                intVars_.push_back(vd);
                break;
            case variable_type::string:
                stringVars_.push_back(vd);
                break;
            case variable_type::boolean:
                boolVars_.push_back(vd);
                break;
            case variable_type::enumeration:
                break;
                //COSIM_PANIC();
        }
    }

    /** Default constructor initialization, all variables are logged. */
    void initialize_default()
    {
        for (const auto& vd : observable_->model_description().variables) {
            if (vd.causality != variable_causality::local) {
                initialize_variable(vd);
            }
        }
    }

    /** External config initialization, only configured variables are logged. */
    void initialize_config(const std::vector<variable_description>& variables)
    {
        for (const auto& vd : variables) {
            initialize_variable(vd);
        }
    }

    std::map<step_number, std::vector<double>> realSamples_;
    std::map<step_number, std::vector<int>> intSamples_;
    std::map<step_number, std::vector<bool>> boolSamples_;
    std::map<step_number, std::vector<std::string_view>> stringSamples_;
    std::map<step_number, double> timeSamples_;
    std::vector<variable_description> realVars_;
    std::vector<variable_description> intVars_;
    std::vector<variable_description> boolVars_;
    std::vector<variable_description> stringVars_;
    observable* observable_;
    size_t decimationFactor_ = 1;
    std::atomic<bool> recording_ = true;
    std::mutex mutex_;
    bool timeStampedFileNames_ = true;

    ros::NodeHandle nh;
    //ros publisher
    ros::Publisher msg_pub;
};

ros_observer::ros_observer()
{
}

ros_observer::ros_observer(const cosim::filesystem::path& configPath)
    : configPath_(configPath)
    , logFromConfig_(true)
{
    boost::property_tree::read_xml(configPath_.string(), ptree_,
        boost::property_tree::xml_parser::no_comments | boost::property_tree::xml_parser::trim_whitespace);
}

namespace
{

template<typename T>
T get_attribute(const boost::property_tree::ptree& tree, const std::string& key)
{
    return tree.get<T>("<xmlattr>." + key);
}

template<typename T>
T get_attribute(const boost::property_tree::ptree& tree, const std::string& key, T& defaultValue)
{
    return tree.get<T>("<xmlattr>." + key, defaultValue);
}
} // namespace

void ros_observer::simulator_added(
    simulator_index index,
    observable* simulator,
    time_point /*currentTime*/)
{
    simulators_[index] = simulator;

    if (logFromConfig_) {
        // Read all configured model names from the XML. If simulator name is not in the list, do nothing.
        std::vector<std::string> modelNames;
        for (const auto& simulatorChild : ptree_.get_child("simulators")) {
            if (simulatorChild.first == "simulator") {
                modelNames.push_back(get_attribute<std::string>(simulatorChild.second, "name"));
            }
        }
        if (std::find(modelNames.begin(), modelNames.end(), simulator->name()) != modelNames.end()) {
            auto config = parse_config(simulator->name());

            valueWriters_[index] = std::make_unique<slave_value_writer>(
                simulator,
                config.decimationFactor,
                config.variables,
                config.timeStampedFileNames);
        } else {
            return;
        }
    } else {
        valueWriters_[index] = std::make_unique<slave_value_writer>(simulator);
    }
}

void ros_observer::simulator_removed(simulator_index index, time_point /*currentTime*/)
{
    valueWriters_.erase(index);
}

void ros_observer::variables_connected(variable_id /*output*/, variable_id /*input*/, time_point /*currentTime*/)
{
}

void ros_observer::variable_disconnected(variable_id /*input*/, time_point /*currentTime*/)
{
}

void ros_observer::simulation_initialized(step_number firstStep, time_point startTime)
{
    if (recording_) {
        for (const auto& entry : valueWriters_) {
            entry.second->observe(firstStep, startTime);
        }
    }
}

void ros_observer::step_complete(step_number /*lastStep*/, duration /*lastStepSize*/, time_point /*currentTime*/)
{
}

void ros_observer::simulator_step_complete(simulator_index index, step_number lastStep, duration /*lastStepSize*/, time_point currentTime)
{
    if (auto writer = valueWriters_.find(index); writer != valueWriters_.end() && recording_) {
        writer->second->observe(lastStep, currentTime);
    }
}

bool ros_observer::is_recording()
{
    return recording_;
}

void ros_observer::start_recording()
{
    if (recording_) {
        throw std::runtime_error("File observer is already recording");
    } else {
        for (const auto& entry : valueWriters_) {
            entry.second->start_recording();
        }
        recording_ = true;
    }
}

void ros_observer::stop_recording()
{
    if (!recording_) {
        throw std::runtime_error("File observer has already stopped recording");
    } else {
        for (const auto& entry : valueWriters_) {
            entry.second->stop_recording();
        }
        recording_ = false;
    }
}

cosim::observable* find_simulator(
    const std::unordered_map<simulator_index, observable*>& simulators,
    const std::string& simulatorName)
{
    for (const auto& entry : simulators) {
        if (entry.second->name() == simulatorName) {
            return entry.second;
        }
    }
    throw std::invalid_argument("Can't find simulator with name: " + simulatorName);
}

ros_observer::simulator_logging_config ros_observer::parse_config(const std::string& simulatorName)
{
    auto simulators = ptree_.get_child("simulators");
    bool timeStampedFileNames = simulators.get<bool>("<xmlattr>.timeStampedFileNames", true);
    for (const auto& childElement : simulators) {
        if (childElement.first == "simulator") {
            auto simulatorElement = childElement.second;
            auto modelName = get_attribute<std::string>(simulatorElement, "name");
            if (modelName == simulatorName) {
                simulator_logging_config config;
                config.timeStampedFileNames = timeStampedFileNames;
                config.decimationFactor = get_attribute<size_t>(simulatorElement, "decimationFactor", defaultDecimationFactor_);

                const auto& simulator = find_simulator(simulators_, modelName);
                if (simulatorElement.count("variable") == 0) {

                    for (const auto& vd : simulator->model_description().variables) {
                        switch (vd.type) {
                            case variable_type::real:
                            case variable_type::integer:
                            case variable_type::boolean:
                            case variable_type::string:
                                config.variables.push_back(vd);
                                break;
                            default:
                                break;
                        }
                    }
                } else {
                    for (const auto& [variableElementName, variableElement] : simulatorElement) {
                        if (variableElementName == "variable") {
                            const auto name = get_attribute<std::string>(variableElement, "name");
                            const auto variableDescription =
                                find_variable(simulator->model_description(), name);

                            if (!variableDescription) {
                                throw std::runtime_error("Can't find variable descriptor with name " + name + " for model with name " + simulator->model_description().name);
                            }

                            switch (variableDescription->type) {
                                case variable_type::real:
                                case variable_type::integer:
                                case variable_type::boolean:
                                case variable_type::string:
                                    config.variables.push_back(*variableDescription);
                                    BOOST_LOG_SEV(log::logger(), log::info) << "Logging variable: " << modelName << ":" << name;
                                    break;
                                default:
                                    break;
                                    //COSIM_PANIC_M("Variable type not supported.");
                            }
                        }
                    }
                }
                return config;
            }
        }
    }
    return simulator_logging_config();
}

ros_observer::~ros_observer() = default;


} // namespace cosim