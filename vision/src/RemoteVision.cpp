//
// Created by nikitas on 4/1/16.
//

#include "RemoteVision.h"

namespace rd {


    RemoteVision::RemoteVision(boost::shared_ptr<Vision> vision, boost::shared_ptr<Camera> camera) :
            RemoteModule("vision") {
        this->addMethod(boost::shared_ptr<RemoteMethod>(new ballDetect(vision)));
        this->addMethod(boost::shared_ptr<RemoteMethod>(new lineDetect(vision)));
        this->addMethod(boost::shared_ptr<RemoteMethod>(new updateFrame(vision, camera)));
    }


    RemoteVision::ballDetect::ballDetect(boost::shared_ptr<Vision> vision) :
            m_vision(vision), RemoteMethod("ballDetect", "S:", "detect ball") { }


    void RemoteVision::ballDetect::execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP) {
        paramList.verifyEnd(0);

        cv::Rect ball = m_vision->ballDetect();

        xmlrpc_env env;
        xmlrpc_env_init(&env);

        xmlrpc_value *elem;
        xmlrpc_value *result;

        result = xmlrpc_struct_new(&env);

        elem = xmlrpc_int_new(&env, ball.x);
        xmlrpc_struct_set_value(&env, result, "x", elem);
        xmlrpc_DECREF(elem);

        elem = xmlrpc_int_new(&env, ball.y);
        xmlrpc_struct_set_value(&env, result, "y", elem);
        xmlrpc_DECREF(elem);

        elem = xmlrpc_int_new(&env, ball.width);
        xmlrpc_struct_set_value(&env, result, "width", elem);
        xmlrpc_DECREF(elem);

        elem = xmlrpc_int_new(&env, ball.height);
        xmlrpc_struct_set_value(&env, result, "height", elem);
        xmlrpc_DECREF(elem);

        resultP->instantiate(result);
        xmlrpc_DECREF(result);
        xmlrpc_env_clean(&env);
    }


    RemoteVision::lineDetect::lineDetect(boost::shared_ptr<Vision> vision)
            : m_vision(vision), RemoteMethod("lineDetect", "A:", "detect lines") { }


    void RemoteVision::lineDetect::execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP) {
        paramList.verifyEnd(0);

        std::vector<cv::Vec4i> lines = m_vision->lineDetect();

        xmlrpc_env env;
        xmlrpc_env_init(&env);

        xmlrpc_value *array = xmlrpc_array_new(&env);
        for (int i = 0; i < lines.size(); ++i) {
            const cv::Vec4i line = lines[i];
            xmlrpc_value *vec4i = xmlrpc_struct_new(&env);

            const char *names[] = {"x1", "y1", "x2", "y2"};
            for (int j = 0; j < 4; ++j) {
                xmlrpc_value *elem = xmlrpc_int_new(&env, line[j]);
                xmlrpc_struct_set_value(&env, vec4i, names[j], elem);
                xmlrpc_DECREF(elem);
            }
            xmlrpc_array_append_item(&env, array, vec4i);
            xmlrpc_DECREF(vec4i);
        }
        resultP->instantiate(array);
        xmlrpc_DECREF(array);
        xmlrpc_env_clean(&env);
    }


    RemoteVision::updateFrame::updateFrame(boost::shared_ptr<Vision> vision, boost::shared_ptr<Camera> camera)
            : m_vision(vision), m_camera(camera), RemoteMethod("updateFrame", "n:", "load next frame") { }

    void RemoteVision::updateFrame::execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP) {
        paramList.verifyEnd(0);

        m_vision->setFrame(m_camera->getCVImage());
        *resultP = xmlrpc_c::value_nil();
    }

}