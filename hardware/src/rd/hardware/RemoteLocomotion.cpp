//
// Created by arssivka on 2/29/16.
//

#include <xmlrpc-c/girerr.hpp>
#include <rd/hardware/TypeDefinition.h>
#include "rd/hardware/RemoteLocomotion.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::RemoteLocomotion::RemoteLocomotion(boost::shared_ptr<Locomotion> locomotion)
        : RemoteModule("locomotion") {
    this->addMethod(boost::shared_ptr<RemoteMethod>(new ParameterKeysMethod(locomotion)));
    this->addMethod(boost::shared_ptr<RemoteMethod>(new OdometryKeysMethod(locomotion)));
    this->addMethod(boost::shared_ptr<RemoteMethod>(new JointKeysMethod(locomotion)));
    this->addMethod(boost::shared_ptr<RemoteMethod>(new ParametersMethod(locomotion)));
    this->addMethod(boost::shared_ptr<RemoteMethod>(new OdometryMethod(locomotion)));
    this->addMethod(boost::shared_ptr<RemoteMethod>(new GaitParametersMethod(locomotion)));
    this->addMethod(boost::shared_ptr<RemoteMethod>(new JointsApplyMethod(locomotion)));
    this->addMethod(boost::shared_ptr<RemoteMethod>(new JointsPositionsMethod(locomotion)));
    this->addMethod(boost::shared_ptr<RemoteMethod>(new JointsHardnessMethod(locomotion)));
    this->addMethod(boost::shared_ptr<RemoteMethod>(new GenerateStepMethod(locomotion)));
    this->addMethod(boost::shared_ptr<RemoteMethod>(new IsDoneMethod(locomotion)));
    this->addMethod(boost::shared_ptr<RemoteMethod>(new AutoUpdateMethod(locomotion)));
    this->addMethod(boost::shared_ptr<RemoteMethod>(new AutoUpdateSleepMethod(locomotion)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::RemoteLocomotion::ParameterKeysMethod::ParameterKeysMethod(boost::shared_ptr<Locomotion> locomotion)
        : RemoteMethod("parameters.keys", "A:", "Return array of locomotion parameter keys names") {
    const StringKeyVector &keys = locomotion->getParameterKeys();
    std::vector<xmlrpc_c::value> values;
    for (StringKeyVector::const_iterator it = keys.begin();
         it != keys.end(); ++it) {
        values.push_back(xmlrpc_c::value_string(*it));
    }
    m_keys = xmlrpc_c::value_array(values);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::RemoteLocomotion::ParameterKeysMethod::execute(xmlrpc_c::paramList const& paramList,
                                                        xmlrpc_c::value* const resultP) {
    paramList.verifyEnd(0);
    *resultP = m_keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::RemoteLocomotion::ParametersMethod::ParametersMethod(boost::shared_ptr<Locomotion> locomotion)
        : RemoteMethod("parameters", "A:,S:A,n:AA", "Parameters control method"), m_locomotion(locomotion) {}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::RemoteLocomotion::ParametersMethod::execute(xmlrpc_c::paramList const& paramList,
                                                     xmlrpc_c::value* const resultP) {
    // n:AA
    if (paramList.size() == 2) {
        std::vector<xmlrpc_c::value> keys = paramList.getArray(0);
        std::vector<xmlrpc_c::value> values = paramList.getArray(1);
        // Size check
        if (keys.size() != values.size()) throw girerr::error("Keys vector and values vector doesn't have same size");
        // Empty check
        if (keys.empty()) return;
        bool integer_keys = keys[0].type() == xmlrpc_c::value::TYPE_INT;
        ValuesVector data(values.size());
        for (int i = 0; i < values.size(); ++i) data[i] = xmlrpc_c::value_double(values[i]);
        if (integer_keys) {
            IntegerKeyVector parameter_names(keys.size());
            for (int i = 0; i < keys.size(); ++i) parameter_names[i] = xmlrpc_c::value_int(keys[i]);
            m_locomotion->setSpeedParameters(parameter_names, data);
        } else {
            StringKeyVector parameter_names(keys.size());
            for (int i = 0; i < keys.size(); ++i) parameter_names[i] = xmlrpc_c::value_string(keys[i]);
            m_locomotion->setSpeedParameters(parameter_names, data);
        }
        *resultP = xmlrpc_c::value_nil();
        return;
    }
    // S:
    SensorData<double>::Ptr data;
    if (paramList.size() == 0) {
        data = m_locomotion->getPositions();
    } else if (paramList.size() == 1) {
        // Empty check
        std::vector<xmlrpc_c::value> keys = paramList.getArray(0);
        if (keys.empty()) return;
        bool integer_keys = keys[0].type() == xmlrpc_c::value::TYPE_INT;
        if (integer_keys) {
            IntegerKeyVector parameter_names(keys.size());
            for (int i = 0; i < keys.size(); ++i) parameter_names[i] = xmlrpc_c::value_int(keys[i]);
            data = m_locomotion->getSpeedParameters(parameter_names);
        } else {
            StringKeyVector parameter_names(keys.size());
            for (int i = 0; i < keys.size(); ++i) parameter_names[i] = xmlrpc_c::value_string(keys[i]);
            data = m_locomotion->getSpeedParameters(parameter_names);
        }
    } else throw girerr::error("Unknown signature for parameters function");

    // TODO Check for memory leaks
    // Some optimisation by using C library of xmlrpc-c
    xmlrpc_env env;
    xmlrpc_env_init(&env);
    xmlrpc_value *elem;
    xmlrpc_value *values;
    xmlrpc_value *result;
    // Init result array
    values = xmlrpc_array_new(&env);
    for (int i = 0; i < data->data.size(); ++i) {
        elem = xmlrpc_double_new(&env, data->data[i]);
        xmlrpc_array_append_item(&env, values, elem);
        xmlrpc_DECREF(elem);
    }
    // XMLRPC-C timestamp
    elem = xmlrpc_double_new(&env, data->timestamp);
    // Create result struct
    result = xmlrpc_struct_new(&env);
    xmlrpc_struct_set_value(&env, result, "data", values);
    xmlrpc_struct_set_value(&env, result, "timestamp", elem);
    // Apply result
    resultP->instantiate(result);
    // Clean this shit!
    xmlrpc_DECREF(elem);
    xmlrpc_DECREF(values);
    xmlrpc_DECREF(result);
    xmlrpc_env_clean(&env);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::RemoteLocomotion::OdometryMethod::OdometryMethod(boost::shared_ptr<Locomotion> locomotion)
        : RemoteMethod("odometry", "S:,S:b", "Getting odometry data or reset"), m_locomotion(locomotion) {}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::RemoteLocomotion::OdometryMethod::execute(xmlrpc_c::paramList const& paramList,
                                                   xmlrpc_c::value* const resultP) {
    if (paramList.size() <= 1) {
        const SensorData<double>::Ptr& data = m_locomotion->getOdometry();

        if (paramList.size() == 1) {
            bool reset = paramList.getBoolean(1);
            if (reset) {
                m_locomotion->resetOdometry();
            }
        }

        // TODO Check for memory leaks
        // Some optimisation by using C library of xmlrpc-c
        xmlrpc_env env;
        xmlrpc_env_init(&env);

        xmlrpc_value* elem;
        xmlrpc_value* values;
        xmlrpc_value* result;

        // Init result array
        values = xmlrpc_array_new(&env);
        for (int i = 0; i < data->data.size(); ++i) {
            elem = xmlrpc_double_new(&env, data->data[i]);
            xmlrpc_array_append_item(&env, values, elem);
            xmlrpc_DECREF(elem);
        }
        // XMLRPC-C timestamp
        elem = xmlrpc_double_new(&env, data->timestamp);
        // Create result struct
        result = xmlrpc_struct_new(&env);
        xmlrpc_struct_set_value(&env, result, "data", values);
        xmlrpc_struct_set_value(&env, result, "timestamp", elem);
        // Apply result
        resultP->instantiate(result);
        // Clean this shit!
        xmlrpc_DECREF(elem);
        xmlrpc_DECREF(values);
        xmlrpc_DECREF(result);
        xmlrpc_env_clean(&env);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::RemoteLocomotion::OdometryKeysMethod::OdometryKeysMethod(boost::shared_ptr<Locomotion> locomotion)
        : RemoteMethod("odometry.keys", "A:", "Return array of locomotion odometry key names") {
    const StringKeyVector &keys = locomotion->getOdometryKeys();
    std::vector<xmlrpc_c::value> values;
    for (StringKeyVector::const_iterator it = keys.begin();
         it != keys.end(); ++it) {
        values.push_back(xmlrpc_c::value_string(*it));
    }
    m_keys = xmlrpc_c::value_array(values);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::RemoteLocomotion::OdometryKeysMethod::execute(xmlrpc_c::paramList const& paramList,
                                                       xmlrpc_c::value* const resultP) {
    paramList.verifyEnd(0);
    *resultP = m_keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::RemoteLocomotion::GaitParametersMethod::GaitParametersMethod(boost::shared_ptr<Locomotion> locomotion)
        : RemoteMethod("gait", "n:AAAAAAAA", "Update gait parameters"), m_locomotion(locomotion) {}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::RemoteLocomotion::GaitParametersMethod::execute(xmlrpc_c::paramList const& paramList,
                                                         xmlrpc_c::value* const resultP) {
    const int config_len[] = {
        WP::LEN_STANCE_CONFIG, WP::LEN_STEP_CONFIG, WP::LEN_ZMP_CONFIG,
                WP::LEN_HACK_CONFIG, WP::LEN_STEP_CONFIG, WP::LEN_STIFF_CONFIG,
                WP::LEN_ODO_CONFIG, WP::LEN_ARM_CONFIG };
    paramList.verifyEnd(8);
    xmlrpc_c::carray config_vector[8];

    for (int i = 0; i < 8; ++i) {
        config_vector[i] = paramList.getArray(i);
        if (config_vector[i].size() != config_len[i]) throw girerr::error("Unknown signature for gait parameters function");
    }

    boost::scoped_array<double> config_array[8];
    for (int i = 0; i < 8; ++i) {
        config_array[i].reset(new double[config_len[i]]);
        for (int j = 0; j != config_vector[i].size(); ++j) {
            config_array[i][j] = (xmlrpc_c::value_double) config_vector[i][j];
        }
    }

    m_locomotion->setGaitParameters(config_array[0].get(), config_array[1].get(), config_array[2].get(),
                                    config_array[3].get(), config_array[4].get(), config_array[5].get(),
                                    config_array[6].get(), config_array[7].get());
    *resultP = xmlrpc_c::value_nil();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::RemoteLocomotion::JointKeysMethod::JointKeysMethod(boost::shared_ptr<Locomotion> locomotion)
        : RemoteMethod("joints.keys", "A:", "Return array of locomotion joint keys names") {
    const StringKeyVector &keys = locomotion->getJointKeys();
    std::vector<xmlrpc_c::value> values;
    for (StringKeyVector::const_iterator it = keys.begin();
         it != keys.end(); ++it) {
        values.push_back(xmlrpc_c::value_string(*it));
    }
    m_keys = xmlrpc_c::value_array(values);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::RemoteLocomotion::JointKeysMethod::execute(xmlrpc_c::paramList const& paramList,
                                                    xmlrpc_c::value* const resultP) {
    paramList.verifyEnd(0);
    *resultP = m_keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::RemoteLocomotion::JointsApplyMethod::JointsApplyMethod(boost::shared_ptr<Locomotion> locomotion) :
        RemoteMethod("joints.apply", "n:", "Apply joints"), m_locomotion(locomotion) {}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::RemoteLocomotion::JointsApplyMethod::execute(xmlrpc_c::paramList const& paramList,
                                                      xmlrpc_c::value* const resultP) {
    paramList.verifyEnd(0);
    m_locomotion->applyPositions();
    *resultP = xmlrpc_c::value_nil();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::RemoteLocomotion::JointsPositionsMethod::JointsPositionsMethod(boost::shared_ptr<Locomotion> locomotion)
        : RemoteMethod("joints.positions", "S:", "Get joint positions"), m_locomotion(locomotion) {

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::RemoteLocomotion::JointsPositionsMethod::execute(xmlrpc_c::paramList const& paramList, xmlrpc_c::value* const resultP) {
    paramList.verifyEnd(0);
    const SensorData<double>::Ptr& data = m_locomotion->getPositions();

    // TODO Check for memory leaks
    // Some optimisation by using C library of xmlrpc-c
    xmlrpc_env env;
    xmlrpc_env_init(&env);

    xmlrpc_value* elem;
    xmlrpc_value* values;
    xmlrpc_value* result;

    // Init result array
    values = xmlrpc_array_new(&env);
    for (int i = 0; i < data->data.size(); ++i) {
        elem = xmlrpc_double_new(&env, data->data[i]);
        xmlrpc_array_append_item(&env, values, elem);
        xmlrpc_DECREF(elem);
    }
    // XMLRPC-C timestamp
    elem = xmlrpc_double_new(&env, data->timestamp);
    // Create result struct
    result = xmlrpc_struct_new(&env);
    xmlrpc_struct_set_value(&env, result, "data", values);
    xmlrpc_struct_set_value(&env, result, "timestamp", elem);
    // Apply result
    resultP->instantiate(result);
    // Clean this shit!
    xmlrpc_DECREF(elem);
    xmlrpc_DECREF(values);
    xmlrpc_DECREF(result);
    xmlrpc_env_clean(&env);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::RemoteLocomotion::JointsHardnessMethod::JointsHardnessMethod(boost::shared_ptr<Locomotion> locomotion)
        : RemoteMethod("joints.hardness", "S:", "Get joint hardness"), m_locomotion(locomotion) {}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::RemoteLocomotion::JointsHardnessMethod::execute(xmlrpc_c::paramList const& paramList,
                                                         xmlrpc_c::value* const resultP) {
    paramList.verifyEnd(0);
    const SensorData<double>::Ptr& data = m_locomotion->getHardness();

    // TODO Check for memory leaks
    // Some optimisation by using C library of xmlrpc-c
    xmlrpc_env env;
    xmlrpc_env_init(&env);

    xmlrpc_value* elem;
    xmlrpc_value* values;
    xmlrpc_value* result;

    // Init result array
    values = xmlrpc_array_new(&env);
    for (int i = 0; i < data->data.size(); ++i) {
        elem = xmlrpc_double_new(&env, data->data[i]);
        xmlrpc_array_append_item(&env, values, elem);
        xmlrpc_DECREF(elem);
    }
    // XMLRPC-C timestamp
    elem = xmlrpc_double_new(&env, data->timestamp);
    // Create result struct
    result = xmlrpc_struct_new(&env);
    xmlrpc_struct_set_value(&env, result, "data", values);
    xmlrpc_struct_set_value(&env, result, "timestamp", elem);
    // Apply result
    resultP->instantiate(result);
    // Clean this shit!
    xmlrpc_DECREF(elem);
    xmlrpc_DECREF(values);
    xmlrpc_DECREF(result);
    xmlrpc_env_clean(&env);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::RemoteLocomotion::IsDoneMethod::IsDoneMethod(boost::shared_ptr<Locomotion> locomotion)
        : RemoteMethod("is_done", "b:", "Is steps generator's job done"), m_locomotion(locomotion) {}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::RemoteLocomotion::IsDoneMethod::execute(xmlrpc_c::paramList const& paramList, xmlrpc_c::value* const resultP) {
    paramList.verifyEnd(0);
    *resultP = xmlrpc_c::value_boolean(m_locomotion->isDone());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::RemoteLocomotion::GenerateStepMethod::GenerateStepMethod(boost::shared_ptr<Locomotion> locomotion)
        : RemoteMethod("generate", "n:", "Generate step"), m_locomotion(locomotion) {}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::RemoteLocomotion::GenerateStepMethod::execute(xmlrpc_c::paramList const& paramList,
                                                       xmlrpc_c::value* const resultP) {
    paramList.verifyEnd(0);
    m_locomotion->generateStep();
    *resultP = xmlrpc_c::value_nil();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::RemoteLocomotion::AutoUpdateSleepMethod::AutoUpdateSleepMethod(boost::shared_ptr<Locomotion> locomotion)
        : RemoteMethod("autoapply.sleep", "d:,n:d", "Function works with sleep time parameters"),
          m_locomotion(locomotion) {}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::RemoteLocomotion::AutoUpdateSleepMethod::execute(xmlrpc_c::paramList const& paramList,
                                                          xmlrpc_c::value* const resultP) {
    int param_size = paramList.size();
    if (param_size == 1) {
        m_locomotion->setAutoApplyUSleepTime((useconds_t) paramList.getDouble(0));
        *resultP = xmlrpc_c::value_nil();
    } else {
        paramList.verifyEnd(0);
        *resultP = xmlrpc_c::value_double(m_locomotion->getAutoApplyUSleepTime());
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::RemoteLocomotion::AutoUpdateMethod::AutoUpdateMethod(boost::shared_ptr<Locomotion> locomotion)
        : RemoteMethod("autoapply.enable", "b:,n:b", "Function works with auto apply method's status"),
          m_locomotion(locomotion) {}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::RemoteLocomotion::AutoUpdateMethod::execute(xmlrpc_c::paramList const& paramList,
                                                     xmlrpc_c::value* const resultP) {
    int param_size = paramList.size();
    if (param_size == 1) {
        m_locomotion->setAutoApply(paramList.getBoolean(0));
        *resultP = xmlrpc_c::value_nil();
    } else {
        paramList.verifyEnd(0);
        *resultP = xmlrpc_c::value_boolean(m_locomotion->isAutoApplyEnabled());
    }
}
