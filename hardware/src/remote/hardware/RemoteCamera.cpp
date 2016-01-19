#include <rd/remote/hardware/RemoteCamera.h>
#include <xmlrpc-c/girerr.hpp>
#include <xmlrpc-c/base64.hpp>
#include <xmlrpc-c/base.h>
#include <xmlrpc-c/base_int.h>
#include <time.h>

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
        : RemoteMethod("image", "A:S", "Get the struct of image binary data and timestamp"), top_camera(top_camera), bot_camera(bot_camera) {
     xmlrpc_env_init(&envP);
     xmlrpc_createXmlrpcValue(&envP, &valP);
     xmlrpc_INCREF(valP);
     valP ->_type = XMLRPC_TYPE_BASE64;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RemoteCamera::ImageMethod::execute(paramList const &paramList, value *const resultP) {
    bool which = paramList.getBoolean(0);
    if(which) {
        unsigned char *dbuf = top_camera->captureImage();
        int time = top_camera -> getTime();
        int size = this->top_camera->getSize();
        xmlrpc_value *elem;
        xmlrpc_value *result;

        // XMLRPC-C timestamp
        elem = xmlrpc_double_new(&envP, time);

        valP->_block._size = size;
        valP->_block._allocated = size;
        valP->_block._block = dbuf;

        // Create result struct
        result = xmlrpc_struct_new(&envP);
        xmlrpc_struct_set_value(&envP, result, "data", valP);
        xmlrpc_struct_set_value(&envP, result, "timestamp", elem);

        resultP->instantiate(result);

        xmlrpc_DECREF(elem);
        xmlrpc_DECREF(result);
        xmlrpc_env_clean(&envP);

    }
    else {
        unsigned char *dbuf = bot_camera->captureImage();
        int time = bot_camera -> getTime();
        int size = this->bot_camera->getSize();
        xmlrpc_value *elem;
        xmlrpc_value *result;

        // XMLRPC-C timestamp
        elem = xmlrpc_double_new(&envP, time);

        valP->_block._size = size;
        valP->_block._allocated = size;
        valP->_block._block = dbuf;

         // Create result struct
        result = xmlrpc_struct_new(&envP);
        xmlrpc_struct_set_value(&envP, result, "data", valP);
        xmlrpc_struct_set_value(&envP, result, "timestamp", elem);

        resultP->instantiate(result);

        xmlrpc_DECREF(elem);
        xmlrpc_DECREF(result);
        xmlrpc_env_clean(&envP);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteCamera::ImageMethod::~ImageMethod() {
    valP->_type = XMLRPC_TYPE_NIL;
    xmlrpc_DECREF(valP);
    xmlrpc_env_clean(&envP);
}
