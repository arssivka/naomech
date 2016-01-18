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
//TODO: Change the creating of xmlrpc and timestamp, imagemethod must return the struct!!!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteCamera::ImageMethod::ImageMethod(shared_ptr<rd::Camera> top_camera, shared_ptr<rd::Camera> bot_camera)
        : RemoteMethod("image", "A:b", "Get the image binary data"), top_camera(top_camera), bot_camera(bot_camera) {
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
        int size = this->top_camera->getSize();
        valP->_block._size = size;
        valP->_block._allocated = size;
        valP->_block._block = dbuf;
        resultP->instantiate(valP);

    }
    else {
        unsigned char *dbuf = bot_camera->captureImage();
        int size = this->bot_camera->getSize();
        valP->_block._size = size;
        valP->_block._allocated = size;
        valP->_block._block = dbuf;
        resultP->instantiate(valP);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteCamera::ImageMethod::~ImageMethod() {
    valP->_type = XMLRPC_TYPE_NIL;
    xmlrpc_DECREF(valP);
    xmlrpc_env_clean(&envP);
}
