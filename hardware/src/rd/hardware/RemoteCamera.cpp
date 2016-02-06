#include <rd/hardware/RemoteCamera.h>
#include <xmlrpc-c/base_int.h>

using namespace xmlrpc_c;
using namespace boost;
using namespace std;
using namespace rd;

RemoteCamera::RemoteCamera(shared_ptr<rd::Camera> top_camera, shared_ptr<rd::Camera> bot_camera)
        : RemoteModule("cameras") {
    this->addMethod(shared_ptr<RemoteMethod>(new ImageMethod(top_camera, bot_camera)));
}
//TODO: Check if there are memeory leaks
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteCamera::ImageMethod::ImageMethod(shared_ptr<rd::Camera> top_camera, shared_ptr<rd::Camera> bot_camera)
        : RemoteMethod("image", "A:S", "Get the struct of image binary data and timestamp"), m_top_camera(top_camera),
          m_bot_camera(bot_camera) {
    xmlrpc_env_init(&m_envP);
    xmlrpc_createXmlrpcValue(&m_envP, &m_valP);
    xmlrpc_INCREF(m_valP);
    m_valP->_type = XMLRPC_TYPE_BASE64;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RemoteCamera::ImageMethod::execute(paramList const &paramList, value *const resultP) {
    bool which = paramList.getBoolean(0);
    if(which) {
        unsigned char* dbuf = m_top_camera->captureImage();
        int time = m_top_camera->getTime();
        int size = m_top_camera->getSize();
        xmlrpc_value *elem;
        xmlrpc_value *result;

        // XMLRPC-C timestamp
        elem = xmlrpc_double_new(&m_envP, time);

        m_valP->_block._size = size;
        m_valP->_block._allocated = size;
        m_valP->_block._block = dbuf;

        // Create result struct
        result = xmlrpc_struct_new(&m_envP);
        xmlrpc_struct_set_value(&m_envP, result, "data", m_valP);
        xmlrpc_struct_set_value(&m_envP, result, "timestamp", elem);
        resultP->instantiate(result);

        xmlrpc_DECREF(elem);
        xmlrpc_DECREF(result);
        //xmlrpc_env_clean(&envP);

    }
    else {
        unsigned char* dbuf = m_bot_camera->captureImage();
        int time = m_bot_camera->getTime();
        int size = m_bot_camera->getSize();
        xmlrpc_value *elem;
        xmlrpc_value *result;

        // XMLRPC-C timestamp
        elem = xmlrpc_double_new(&m_envP, time);

        m_valP->_block._size = size;
        m_valP->_block._allocated = size;
        m_valP->_block._block = dbuf;

         // Create result struct
        result = xmlrpc_struct_new(&m_envP);
        xmlrpc_struct_set_value(&m_envP, result, "data", m_valP);
        xmlrpc_struct_set_value(&m_envP, result, "timestamp", elem);

        resultP->instantiate(result);

        xmlrpc_DECREF(elem);
        xmlrpc_DECREF(result);
       // xmlrpc_env_clean(&envP);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteCamera::ImageMethod::~ImageMethod() {
    m_valP->_type = XMLRPC_TYPE_NIL;
    xmlrpc_DECREF(m_valP);
    xmlrpc_env_clean(&m_envP);
}
