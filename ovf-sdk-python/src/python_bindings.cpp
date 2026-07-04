/**
 * @file python_bindings.cpp
 * @brief OpenVisionFlow Python SDK 绑定实现
 * 
 * 使用Python C API实现纯C接口绑定
 */

#include "ovf_python.h"

#include <Python.h>
#include <structmember.h>
#include <cstring>

// ============================================================
// 异常类
// ============================================================

static PyObject* OVFError;

static void set_python_error(OVFErrorCode code) {
    char buffer[256];
    ovf_error_code_to_string(code, buffer, sizeof(buffer));
    
    char error_msg[512];
    ovf_get_last_error_message(error_msg, sizeof(error_msg));
    
    PyErr_SetString(OVFError, error_msg[0] ? error_msg : buffer);
}

// ============================================================
// Image类
// ============================================================

typedef struct {
    PyObject_HEAD
    OVFImage handle;
    int width;
    int height;
    int channels;
} ImageObject;

static void Image_dealloc(ImageObject* self) {
    if (self->handle) {
        ovf_destroy_image(self->handle);
        self->handle = nullptr;
    }
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject* Image_new(PyTypeObject* type, PyObject* args, PyObject* kwds) {
    ImageObject* self;
    self = (ImageObject*)type->tp_alloc(type, 0);
    if (self) {
        self->handle = nullptr;
        self->width = 0;
        self->height = 0;
        self->channels = 0;
    }
    return (PyObject*)self;
}

static int Image_init(ImageObject* self, PyObject* args, PyObject* kwds) {
    int width, height, channels = 1;
    static char* kwlist[] = {"width", "height", "channels", nullptr};
    
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "ii|i", kwlist, &width, &height, &channels)) {
        return -1;
    }
    
    self->handle = ovf_create_image(width, height, channels);
    if (!self->handle) {
        set_python_error(ovf_get_last_error_code());
        return -1;
    }
    
    self->width = width;
    self->height = height;
    self->channels = channels;
    
    return 0;
}

static PyObject* Image_width(ImageObject* self, PyObject* Py_UNUSED(ignored)) {
    return PyLong_FromLong(self->width);
}

static PyObject* Image_height(ImageObject* self, PyObject* Py_UNUSED(ignored)) {
    return PyLong_FromLong(self->height);
}

static PyObject* Image_channels(ImageObject* self, PyObject* Py_UNUSED(ignored)) {
    return PyLong_FromLong(self->channels);
}

static PyObject* Image_data(ImageObject* self, PyObject* Py_UNUSED(ignored)) {
    if (!self->handle) {
        PyErr_SetString(PyExc_ValueError, "Image not initialized");
        return nullptr;
    }
    
    size_t size = ovf_image_data_size(self->handle);
    uint8_t* data = ovf_image_data(self->handle);
    
    return PyBytes_FromStringAndSize((char*)data, size);
}

static PyObject* Image_get_data(ImageObject* self, PyObject* args) {
    PyObject* buffer_obj = nullptr;
    
    if (!PyArg_ParseTuple(args, "|O", &buffer_obj)) {
        return nullptr;
    }
    
    if (!self->handle) {
        PyErr_SetString(PyExc_ValueError, "Image not initialized");
        return nullptr;
    }
    
    size_t size = ovf_image_data_size(self->handle);
    
    if (buffer_obj && PyByteArray_Check(buffer_obj)) {
        // 写入到提供的bytearray
        Py_ssize_t buffer_size = PyByteArray_Size(buffer_obj);
        if (buffer_size < (Py_ssize_t)size) {
            PyErr_SetString(PyExc_ValueError, "Buffer too small");
            return nullptr;
        }
        ovf_image_copy_data(self->handle, (uint8_t*)PyByteArray_AsString(buffer_obj), size);
        Py_RETURN_NONE;
    }
    
    // 返回新的bytes对象
    uint8_t* data = ovf_image_data(self->handle);
    return PyBytes_FromStringAndSize((char*)data, size);
}

static PyObject* Image_set_data(ImageObject* self, PyObject* args) {
    Py_buffer buffer;
    
    if (!PyArg_ParseTuple(args, "y*", &buffer)) {
        return nullptr;
    }
    
    if (!self->handle) {
        PyBuffer_Release(&buffer);
        PyErr_SetString(PyExc_ValueError, "Image not initialized");
        return nullptr;
    }
    
    size_t expected_size = self->width * self->height * self->channels;
    if (buffer.len < expected_size) {
        PyBuffer_Release(&buffer);
        PyErr_SetString(PyExc_ValueError, "Data size mismatch");
        return nullptr;
    }
    
    // 复制数据到图像
    uint8_t* img_data = ovf_image_data(self->handle);
    std::memcpy(img_data, buffer.buf, expected_size);
    
    PyBuffer_Release(&buffer);
    Py_RETURN_NONE;
}

static PyObject* Image_from_bytes(PyTypeObject* type, PyObject* args) {
    Py_buffer buffer;
    int width, height, channels = 1;
    
    if (!PyArg_ParseTuple(args, "y*ii|i", &buffer, &width, &height, &channels)) {
        return nullptr;
    }
    
    size_t expected_size = width * height * channels;
    if (buffer.len < expected_size) {
        PyBuffer_Release(&buffer);
        PyErr_SetString(PyExc_ValueError, "Data size mismatch");
        return nullptr;
    }
    
    OVFImage handle = ovf_create_image_from_data((uint8_t*)buffer.buf, width, height, channels, buffer.len);
    PyBuffer_Release(&buffer);
    
    if (!handle) {
        set_python_error(ovf_get_last_error_code());
        return nullptr;
    }
    
    ImageObject* self = (ImageObject*)type->tp_alloc(type, 0);
    if (!self) {
        ovf_destroy_image(handle);
        return nullptr;
    }
    
    self->handle = handle;
    self->width = width;
    self->height = height;
    self->channels = channels;
    
    return (PyObject*)self;
}

static PyObject* Image_tobytes(ImageObject* self, PyObject* Py_UNUSED(ignored)) {
    return Image_data(self, nullptr);
}

static PyObject* Image_toarray(ImageObject* self, PyObject* Py_UNUSED(ignored)) {
    // 返回 bytearray，可修改
    if (!self->handle) {
        PyErr_SetString(PyExc_ValueError, "Image not initialized");
        return nullptr;
    }
    
    size_t size = ovf_image_data_size(self->handle);
    uint8_t* data = ovf_image_data(self->handle);
    
    PyObject* arr = PyByteArray_FromStringAndSize((char*)data, size);
    return arr;
}

static PyMethodDef Image_methods[] = {
    {"width", (PyCFunction)Image_width, METH_NOARGS, "获取图像宽度"},
    {"height", (PyCFunction)Image_height, METH_NOARGS, "获取图像高度"},
    {"channels", (PyCFunction)Image_channels, METH_NOARGS, "获取图像通道数"},
    {"data", (PyCFunction)Image_data, METH_NOARGS, "获取图像数据(bytes)"},
    {"get_data", (PyCFunction)Image_get_data, METH_VARARGS, "获取图像数据"},
    {"set_data", (PyCFunction)Image_set_data, METH_VARARGS, "设置图像数据"},
    {"from_bytes", (PyCFunction)Image_from_bytes, METH_VARARGS|METH_CLASS, "从bytes创建图像"},
    {"tobytes", (PyCFunction)Image_tobytes, METH_NOARGS, "转换为bytes"},
    {"toarray", (PyCFunction)Image_toarray, METH_NOARGS, "转换为bytearray"},
    {nullptr}
};

static PyMemberDef Image_members[] = {
    {"width", T_INT, offsetof(ImageObject, width), READONLY, "图像宽度"},
    {"height", T_INT, offsetof(ImageObject, height), READONLY, "图像高度"},
    {"channels", T_INT, offsetof(ImageObject, channels), READONLY, "通道数"},
    {nullptr}
};

static PyTypeObject ImageType = {
    PyVarObject_HEAD_INIT(nullptr, 0)
    "ovf.Image",                         /* tp_name */
    sizeof(ImageObject),                 /* tp_basicsize */
    0,                                   /* tp_itemsize */
    (destructor)Image_dealloc,           /* tp_dealloc */
    0,                                   /* tp_vectorcall_offset */
    nullptr,                             /* tp_getattr */
    nullptr,                             /* tp_setattr */
    nullptr,                             /* tp_as_async */
    nullptr,                             /* tp_repr */
    nullptr,                             /* tp_as_number */
    nullptr,                             /* tp_as_sequence */
    nullptr,                             /* tp_as_mapping */
    nullptr,                             /* tp_hash */
    nullptr,                             /* tp_call */
    nullptr,                             /* tp_str */
    nullptr,                             /* tp_getattro */
    nullptr,                             /* tp_setattro */
    nullptr,                             /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, /* tp_flags */
    "OpenVisionFlow 图像对象",            /* tp_doc */
    nullptr,                             /* tp_traverse */
    nullptr,                             /* tp_clear */
    nullptr,                             /* tp_richcompare */
    0,                                   /* tp_weaklistoffset */
    nullptr,                             /* tp_iter */
    nullptr,                             /* tp_iternext */
    Image_methods,                       /* tp_methods */
    Image_members,                       /* tp_members */
    nullptr,                             /* tp_getset */
    nullptr,                             /* tp_base */
    nullptr,                             /* tp_dict */
    nullptr,                             /* tp_descr_get */
    nullptr,                             /* tp_descr_set */
    0,                                   /* tp_dictoffset */
    (initproc)Image_init,                /* tp_init */
    nullptr,                             /* tp_alloc */
    Image_new,                           /* tp_new */
};

// ============================================================
// FlowEngine类
// ============================================================

typedef struct {
    PyObject_HEAD
    OVFEngine handle;
} FlowEngineObject;

static void FlowEngine_dealloc(FlowEngineObject* self) {
    if (self->handle) {
        ovf_destroy_engine(self->handle);
        self->handle = nullptr;
    }
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject* FlowEngine_new(PyTypeObject* type, PyObject* args, PyObject* kwds) {
    FlowEngineObject* self;
    self = (FlowEngineObject*)type->tp_alloc(type, 0);
    if (self) {
        self->handle = nullptr;
    }
    return (PyObject*)self;
}

static int FlowEngine_init(FlowEngineObject* self, PyObject* args, PyObject* kwds) {
    self->handle = ovf_create_engine();
    if (!self->handle) {
        set_python_error(ovf_get_last_error_code());
        return -1;
    }
    return 0;
}

static PyObject* FlowEngine_load_flow(FlowEngineObject* self, PyObject* args) {
    const char* filepath;
    
    if (!PyArg_ParseTuple(args, "s", &filepath)) {
        return nullptr;
    }
    
    OVFErrorCode code = ovf_load_flow(self->handle, filepath);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    
    Py_RETURN_NONE;
}

static PyObject* FlowEngine_load_flow_from_json(FlowEngineObject* self, PyObject* args) {
    const char* json_content;
    
    if (!PyArg_ParseTuple(args, "s", &json_content)) {
        return nullptr;
    }
    
    OVFErrorCode code = ovf_load_flow_from_json(self->handle, json_content);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    
    Py_RETURN_NONE;
}

static PyObject* FlowEngine_save_flow(FlowEngineObject* self, PyObject* args) {
    const char* filepath;
    
    if (!PyArg_ParseTuple(args, "s", &filepath)) {
        return nullptr;
    }
    
    OVFErrorCode code = ovf_save_flow(self->handle, filepath);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    
    Py_RETURN_NONE;
}

static PyObject* FlowEngine_run(FlowEngineObject* self, PyObject* Py_UNUSED(ignored)) {
    OVFErrorCode code = ovf_run_flow(self->handle);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    
    Py_RETURN_NONE;
}

static PyObject* FlowEngine_run_node(FlowEngineObject* self, PyObject* args) {
    const char* node_id;
    
    if (!PyArg_ParseTuple(args, "s", &node_id)) {
        return nullptr;
    }
    
    OVFErrorCode code = ovf_run_node(self->handle, node_id);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    
    Py_RETURN_NONE;
}

static PyObject* FlowEngine_set_input_image(FlowEngineObject* self, PyObject* args) {
    const char* node_id, *port_id;
    ImageObject* img;
    
    if (!PyArg_ParseTuple(args, "ssO!", &node_id, &port_id, &ImageType, &img)) {
        return nullptr;
    }
    
    OVFErrorCode code = ovf_set_input_image(self->handle, node_id, port_id, img->handle);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    
    Py_RETURN_NONE;
}

static PyObject* FlowEngine_set_input_number(FlowEngineObject* self, PyObject* args) {
    const char* node_id, *port_id;
    double value;
    
    if (!PyArg_ParseTuple(args, "ssd", &node_id, &port_id, &value)) {
        return nullptr;
    }
    
    OVFErrorCode code = ovf_set_input_number(self->handle, node_id, port_id, value);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    
    Py_RETURN_NONE;
}

static PyObject* FlowEngine_set_input_string(FlowEngineObject* self, PyObject* args) {
    const char* node_id, *port_id, *value;
    
    if (!PyArg_ParseTuple(args, "sss", &node_id, &port_id, &value)) {
        return nullptr;
    }
    
    OVFErrorCode code = ovf_set_input_string(self->handle, node_id, port_id, value);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    
    Py_RETURN_NONE;
}

static PyObject* FlowEngine_get_output_image(FlowEngineObject* self, PyObject* args) {
    const char* node_id, *port_id;
    
    if (!PyArg_ParseTuple(args, "ss", &node_id, &port_id)) {
        return nullptr;
    }
    
    OVFImage img = ovf_get_output_image(self->handle, node_id, port_id);
    if (!img) {
        set_python_error(ovf_get_last_error_code());
        return nullptr;
    }
    
    ImageObject* result = (ImageObject*)ImageType.tp_alloc(&ImageType, 0);
    if (!result) {
        ovf_destroy_image(img);
        return nullptr;
    }
    
    result->handle = img;
    result->width = ovf_image_width(img);
    result->height = ovf_image_height(img);
    result->channels = ovf_image_channels(img);
    
    return (PyObject*)result;
}

static PyObject* FlowEngine_get_output_number(FlowEngineObject* self, PyObject* args) {
    const char* node_id, *port_id;
    
    if (!PyArg_ParseTuple(args, "ss", &node_id, &port_id)) {
        return nullptr;
    }
    
    double value = 0;
    OVFErrorCode code = ovf_get_output_number(self->handle, node_id, port_id, &value);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    
    return PyFloat_FromDouble(value);
}

static PyObject* FlowEngine_get_node_count(FlowEngineObject* self, PyObject* Py_UNUSED(ignored)) {
    int count = ovf_get_node_count(self->handle);
    return PyLong_FromLong(count);
}

static PyObject* FlowEngine_get_node_ids(FlowEngineObject* self, PyObject* Py_UNUSED(ignored)) {
    int count = ovf_get_node_count(self->handle);
    if (count == 0) {
        return PyList_New(0);
    }
    
    PyObject* list = PyList_New(count);
    if (!list) {
        return nullptr;
    }
    
    // 分配临时缓冲区
    char** ids = new char*[count];
    for (int i = 0; i < count; ++i) {
        ids[i] = new char[256];
    }
    
    int actual_count = ovf_get_node_ids(self->handle, ids, count, 256);
    
    for (int i = 0; i < actual_count; ++i) {
        PyList_SET_ITEM(list, i, PyUnicode_FromString(ids[i]));
    }
    
    // 释放缓冲区
    for (int i = 0; i < count; ++i) {
        delete[] ids[i];
    }
    delete[] ids;
    
    return list;
}

static PyObject* FlowEngine_get_node_state(FlowEngineObject* self, PyObject* args) {
    const char* node_id;
    
    if (!PyArg_ParseTuple(args, "s", &node_id)) {
        return nullptr;
    }
    
    OVFNodeState state = ovf_get_node_state(self->handle, node_id);
    return PyLong_FromLong(state);
}

static PyObject* FlowEngine_get_node_execute_time(FlowEngineObject* self, PyObject* args) {
    const char* node_id;
    
    if (!PyArg_ParseTuple(args, "s", &node_id)) {
        return nullptr;
    }
    
    uint64_t time = ovf_get_node_execute_time(self->handle, node_id);
    return PyLong_FromUnsignedLong(time);
}

static PyObject* FlowEngine_connect(FlowEngineObject* self, PyObject* args) {
    const char* source_node, *source_port, *target_node, *target_port;
    
    if (!PyArg_ParseTuple(args, "ssss", &source_node, &source_port, &target_node, &target_port)) {
        return nullptr;
    }
    
    OVFErrorCode code = ovf_connect_nodes(self->handle, source_node, source_port, target_node, target_port);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    
    Py_RETURN_NONE;
}

static PyObject* FlowEngine_get_flow_json(FlowEngineObject* self, PyObject* Py_UNUSED(ignored)) {
    char buffer[4096];
    size_t len = ovf_get_flow_json(self->handle, buffer, sizeof(buffer));
    return PyUnicode_FromStringAndSize(buffer, len);
}

static PyMethodDef FlowEngine_methods[] = {
    {"load_flow", (PyCFunction)FlowEngine_load_flow, METH_VARARGS, "加载流程文件"},
    {"load_flow_from_json", (PyCFunction)FlowEngine_load_flow_from_json, METH_VARARGS, "从JSON加载流程"},
    {"save_flow", (PyCFunction)FlowEngine_save_flow, METH_VARARGS, "保存流程文件"},
    {"run", (PyCFunction)FlowEngine_run, METH_NOARGS, "运行流程"},
    {"run_node", (PyCFunction)FlowEngine_run_node, METH_VARARGS, "运行指定节点"},
    {"set_input_image", (PyCFunction)FlowEngine_set_input_image, METH_VARARGS, "设置输入图像"},
    {"set_input_number", (PyCFunction)FlowEngine_set_input_number, METH_VARARGS, "设置输入数值"},
    {"set_input_string", (PyCFunction)FlowEngine_set_input_string, METH_VARARGS, "设置输入字符串"},
    {"get_output_image", (PyCFunction)FlowEngine_get_output_image, METH_VARARGS, "获取输出图像"},
    {"get_output_number", (PyCFunction)FlowEngine_get_output_number, METH_VARARGS, "获取输出数值"},
    {"get_node_count", (PyCFunction)FlowEngine_get_node_count, METH_NOARGS, "获取节点数量"},
    {"get_node_ids", (PyCFunction)FlowEngine_get_node_ids, METH_NOARGS, "获取节点ID列表"},
    {"get_node_state", (PyCFunction)FlowEngine_get_node_state, METH_VARARGS, "获取节点状态"},
    {"get_node_execute_time", (PyCFunction)FlowEngine_get_node_execute_time, METH_VARARGS, "获取节点执行时间"},
    {"connect", (PyCFunction)FlowEngine_connect, METH_VARARGS, "连接节点"},
    {"get_flow_json", (PyCFunction)FlowEngine_get_flow_json, METH_NOARGS, "获取流程JSON"},
    {nullptr}
};

static PyTypeObject FlowEngineType = {
    PyVarObject_HEAD_INIT(nullptr, 0)
    "ovf.FlowEngine",                    /* tp_name */
    sizeof(FlowEngineObject),            /* tp_basicsize */
    0,                                   /* tp_itemsize */
    (destructor)FlowEngine_dealloc,      /* tp_dealloc */
    0,                                   /* tp_vectorcall_offset */
    nullptr,                             /* tp_getattr */
    nullptr,                             /* tp_setattr */
    nullptr,                             /* tp_as_async */
    nullptr,                             /* tp_repr */
    nullptr,                             /* tp_as_number */
    nullptr,                             /* tp_as_sequence */
    nullptr,                             /* tp_as_mapping */
    nullptr,                             /* tp_hash */
    nullptr,                             /* tp_call */
    nullptr,                             /* tp_str */
    nullptr,                             /* tp_getattro */
    nullptr,                             /* tp_setattro */
    nullptr,                             /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, /* tp_flags */
    "OpenVisionFlow 流程引擎",            /* tp_doc */
    nullptr,                             /* tp_traverse */
    nullptr,                             /* tp_clear */
    nullptr,                             /* tp_richcompare */
    0,                                   /* tp_weaklistoffset */
    nullptr,                             /* tp_iter */
    nullptr,                             /* tp_iternext */
    FlowEngine_methods,                  /* tp_methods */
    nullptr,                             /* tp_members */
    nullptr,                             /* tp_getset */
    nullptr,                             /* tp_base */
    nullptr,                             /* tp_dict */
    nullptr,                             /* tp_descr_get */
    nullptr,                             /* tp_descr_set */
    0,                                   /* tp_dictoffset */
    (initproc)FlowEngine_init,           /* tp_init */
    nullptr,                             /* tp_alloc */
    FlowEngine_new,                      /* tp_new */
};

// ============================================================
// Camera类
// ============================================================

typedef struct {
    PyObject_HEAD
    OVFCamera handle;
} CameraObject;

static void Camera_dealloc(CameraObject* self) {
    if (self->handle) {
        ovf_close_camera(self->handle);
        self->handle = nullptr;
    }
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject* Camera_new(PyTypeObject* type, PyObject* args, PyObject* kwds) {
    CameraObject* self;
    self = (CameraObject*)type->tp_alloc(type, 0);
    if (self) {
        self->handle = nullptr;
    }
    return (PyObject*)self;
}

static int Camera_init(CameraObject* self, PyObject* args, PyObject* kwds) {
    const char* driver_type = "mock";
    const char* device_id = "default";
    static char* kwlist[] = {"driver_type", "device_id", nullptr};
    
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|ss", kwlist, &driver_type, &device_id)) {
        return -1;
    }
    
    self->handle = ovf_open_camera(driver_type, device_id);
    if (!self->handle) {
        set_python_error(ovf_get_last_error_code());
        return -1;
    }
    
    return 0;
}

static PyObject* Camera_start_capture(CameraObject* self, PyObject* Py_UNUSED(ignored)) {
    OVFErrorCode code = ovf_start_capture(self->handle);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    Py_RETURN_NONE;
}

static PyObject* Camera_stop_capture(CameraObject* self, PyObject* Py_UNUSED(ignored)) {
    OVFErrorCode code = ovf_stop_capture(self->handle);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    Py_RETURN_NONE;
}

static PyObject* Camera_capture_frame(CameraObject* self, PyObject* args) {
    int timeout_ms = 1000;
    
    if (!PyArg_ParseTuple(args, "|i", &timeout_ms)) {
        return nullptr;
    }
    
    OVFImage img = ovf_capture_frame(self->handle, timeout_ms);
    if (!img) {
        set_python_error(ovf_get_last_error_code());
        return nullptr;
    }
    
    ImageObject* result = (ImageObject*)ImageType.tp_alloc(&ImageType, 0);
    if (!result) {
        ovf_destroy_image(img);
        return nullptr;
    }
    
    result->handle = img;
    result->width = ovf_image_width(img);
    result->height = ovf_image_height(img);
    result->channels = ovf_image_channels(img);
    
    return (PyObject*)result;
}

static PyObject* Camera_set_exposure(CameraObject* self, PyObject* args) {
    double exposure_us;
    
    if (!PyArg_ParseTuple(args, "d", &exposure_us)) {
        return nullptr;
    }
    
    OVFErrorCode code = ovf_set_exposure(self->handle, exposure_us);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    Py_RETURN_NONE;
}

static PyObject* Camera_get_exposure(CameraObject* self, PyObject* Py_UNUSED(ignored)) {
    double exposure_us = 0;
    OVFErrorCode code = ovf_get_exposure(self->handle, &exposure_us);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    return PyFloat_FromDouble(exposure_us);
}

static PyObject* Camera_set_gain(CameraObject* self, PyObject* args) {
    double gain;
    
    if (!PyArg_ParseTuple(args, "d", &gain)) {
        return nullptr;
    }
    
    OVFErrorCode code = ovf_set_gain(self->handle, gain);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    Py_RETURN_NONE;
}

static PyObject* Camera_get_gain(CameraObject* self, PyObject* Py_UNUSED(ignored)) {
    double gain = 0;
    OVFErrorCode code = ovf_get_gain(self->handle, &gain);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    return PyFloat_FromDouble(gain);
}

static PyObject* Camera_send_soft_trigger(CameraObject* self, PyObject* Py_UNUSED(ignored)) {
    OVFErrorCode code = ovf_send_soft_trigger(self->handle);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    Py_RETURN_NONE;
}

static PyMethodDef Camera_methods[] = {
    {"start_capture", (PyCFunction)Camera_start_capture, METH_NOARGS, "开始采集"},
    {"stop_capture", (PyCFunction)Camera_stop_capture, METH_NOARGS, "停止采集"},
    {"capture_frame", (PyCFunction)Camera_capture_frame, METH_VARARGS, "采集一帧"},
    {"set_exposure", (PyCFunction)Camera_set_exposure, METH_VARARGS, "设置曝光时间(us)"},
    {"get_exposure", (PyCFunction)Camera_get_exposure, METH_NOARGS, "获取曝光时间"},
    {"set_gain", (PyCFunction)Camera_set_gain, METH_VARARGS, "设置增益"},
    {"get_gain", (PyCFunction)Camera_get_gain, METH_NOARGS, "获取增益"},
    {"send_soft_trigger", (PyCFunction)Camera_send_soft_trigger, METH_NOARGS, "发送软触发"},
    {nullptr}
};

static PyTypeObject CameraType = {
    PyVarObject_HEAD_INIT(nullptr, 0)
    "ovf.Camera",                        /* tp_name */
    sizeof(CameraObject),                /* tp_basicsize */
    0,                                   /* tp_itemsize */
    (destructor)Camera_dealloc,          /* tp_dealloc */
    0,                                   /* tp_vectorcall_offset */
    nullptr,                             /* tp_getattr */
    nullptr,                             /* tp_setattr */
    nullptr,                             /* tp_as_async */
    nullptr,                             /* tp_repr */
    nullptr,                             /* tp_as_number */
    nullptr,                             /* tp_as_sequence */
    nullptr,                             /* tp_as_mapping */
    nullptr,                             /* tp_hash */
    nullptr,                             /* tp_call */
    nullptr,                             /* tp_str */
    nullptr,                             /* tp_getattro */
    nullptr,                             /* tp_setattro */
    nullptr,                             /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, /* tp_flags */
    "OpenVisionFlow 相机对象",            /* tp_doc */
    nullptr,                             /* tp_traverse */
    nullptr,                             /* tp_clear */
    nullptr,                             /* tp_richcompare */
    0,                                   /* tp_weaklistoffset */
    nullptr,                             /* tp_iter */
    nullptr,                             /* tp_iternext */
    Camera_methods,                      /* tp_methods */
    nullptr,                             /* tp_members */
    nullptr,                             /* tp_getset */
    nullptr,                             /* tp_base */
    nullptr,                             /* tp_dict */
    nullptr,                             /* tp_descr_get */
    nullptr,                             /* tp_descr_set */
    0,                                   /* tp_dictoffset */
    (initproc)Camera_init,               /* tp_init */
    nullptr,                             /* tp_alloc */
    Camera_new,                          /* tp_new */
};

// ============================================================
// FlowRunner类
// ============================================================

typedef struct {
    PyObject_HEAD
    OVFRunner handle;
} FlowRunnerObject;

static void FlowRunner_dealloc(FlowRunnerObject* self) {
    if (self->handle) {
        ovf_destroy_runner(self->handle);
        self->handle = nullptr;
    }
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject* FlowRunner_new(PyTypeObject* type, PyObject* args, PyObject* kwds) {
    FlowRunnerObject* self;
    self = (FlowRunnerObject*)type->tp_alloc(type, 0);
    if (self) {
        self->handle = nullptr;
    }
    return (PyObject*)self;
}

static int FlowRunner_init(FlowRunnerObject* self, PyObject* args, PyObject* kwds) {
    self->handle = ovf_create_runner();
    if (!self->handle) {
        set_python_error(ovf_get_last_error_code());
        return -1;
    }
    return 0;
}

static PyObject* FlowRunner_set_engine(FlowRunnerObject* self, PyObject* args) {
    FlowEngineObject* engine;
    
    if (!PyArg_ParseTuple(args, "O!", &FlowEngineType, &engine)) {
        return nullptr;
    }
    
    OVFErrorCode code = ovf_runner_set_engine(self->handle, engine->handle);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    Py_RETURN_NONE;
}

static PyObject* FlowRunner_start_continuous(FlowRunnerObject* self, PyObject* Py_UNUSED(ignored)) {
    OVFErrorCode code = ovf_runner_start_continuous(self->handle);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    Py_RETURN_NONE;
}

static PyObject* FlowRunner_start_triggered(FlowRunnerObject* self, PyObject* Py_UNUSED(ignored)) {
    OVFErrorCode code = ovf_runner_start_triggered(self->handle);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    Py_RETURN_NONE;
}

static PyObject* FlowRunner_stop(FlowRunnerObject* self, PyObject* Py_UNUSED(ignored)) {
    OVFErrorCode code = ovf_runner_stop(self->handle);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    Py_RETURN_NONE;
}

static PyObject* FlowRunner_trigger(FlowRunnerObject* self, PyObject* Py_UNUSED(ignored)) {
    OVFErrorCode code = ovf_runner_trigger(self->handle);
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    Py_RETURN_NONE;
}

static PyObject* FlowRunner_is_running(FlowRunnerObject* self, PyObject* Py_UNUSED(ignored)) {
    int running = ovf_runner_is_running(self->handle);
    return PyBool_FromLong(running);
}

static PyObject* FlowRunner_get_stats(FlowRunnerObject* self, PyObject* Py_UNUSED(ignored)) {
    uint64_t total, success, failed;
    ovf_runner_get_stats(self->handle, &total, &success, &failed);
    
    return Py_BuildValue("(KKK)", total, success, failed);
}

static PyMethodDef FlowRunner_methods[] = {
    {"set_engine", (PyCFunction)FlowRunner_set_engine, METH_VARARGS, "设置关联的流程引擎"},
    {"start_continuous", (PyCFunction)FlowRunner_start_continuous, METH_NOARGS, "启动连续运行"},
    {"start_triggered", (PyCFunction)FlowRunner_start_triggered, METH_NOARGS, "启动触发模式"},
    {"stop", (PyCFunction)FlowRunner_stop, METH_NOARGS, "停止运行"},
    {"trigger", (PyCFunction)FlowRunner_trigger, METH_NOARGS, "触发执行"},
    {"is_running", (PyCFunction)FlowRunner_is_running, METH_NOARGS, "检查是否运行中"},
    {"get_stats", (PyCFunction)FlowRunner_get_stats, METH_NOARGS, "获取运行统计"},
    {nullptr}
};

static PyTypeObject FlowRunnerType = {
    PyVarObject_HEAD_INIT(nullptr, 0)
    "ovf.FlowRunner",                    /* tp_name */
    sizeof(FlowRunnerObject),            /* tp_basicsize */
    0,                                   /* tp_itemsize */
    (destructor)FlowRunner_dealloc,      /* tp_dealloc */
    0,                                   /* tp_vectorcall_offset */
    nullptr,                             /* tp_getattr */
    nullptr,                             /* tp_setattr */
    nullptr,                             /* tp_as_async */
    nullptr,                             /* tp_repr */
    nullptr,                             /* tp_as_number */
    nullptr,                             /* tp_as_sequence */
    nullptr,                             /* tp_as_mapping */
    nullptr,                             /* tp_hash */
    nullptr,                             /* tp_call */
    nullptr,                             /* tp_str */
    nullptr,                             /* tp_getattro */
    nullptr,                             /* tp_setattro */
    nullptr,                             /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, /* tp_flags */
    "OpenVisionFlow 流程运行器",          /* tp_doc */
    nullptr,                             /* tp_traverse */
    nullptr,                             /* tp_clear */
    nullptr,                             /* tp_richcompare */
    0,                                   /* tp_weaklistoffset */
    nullptr,                             /* tp_iter */
    nullptr,                             /* tp_iternext */
    FlowRunner_methods,                  /* tp_methods */
    nullptr,                             /* tp_members */
    nullptr,                             /* tp_getset */
    nullptr,                             /* tp_base */
    nullptr,                             /* tp_dict */
    nullptr,                             /* tp_descr_get */
    nullptr,                             /* tp_descr_set */
    0,                                   /* tp_dictoffset */
    (initproc)FlowRunner_init,           /* tp_init */
    nullptr,                             /* tp_alloc */
    FlowRunner_new,                      /* tp_new */
};

// ============================================================
// 模级函数
// ============================================================

static PyObject* py_initialize(PyObject* self, PyObject* Py_UNUSED(ignored)) {
    OVFErrorCode code = ovf_initialize();
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    Py_RETURN_NONE;
}

static PyObject* py_shutdown(PyObject* self, PyObject* Py_UNUSED(ignored)) {
    OVFErrorCode code = ovf_shutdown();
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    Py_RETURN_NONE;
}

static PyObject* py_get_version(PyObject* self, PyObject* Py_UNUSED(ignored)) {
    int major, minor, patch;
    ovf_get_version(&major, &minor, &patch);
    return Py_BuildValue("(iii)", major, minor, patch);
}

static PyObject* py_get_registered_node_types(PyObject* self, PyObject* Py_UNUSED(ignored)) {
    int count = ovf_get_registered_node_type_count();
    if (count == 0) {
        return PyList_New(0);
    }
    
    PyObject* list = PyList_New(count);
    if (!list) {
        return nullptr;
    }
    
    char** types = new char*[count];
    for (int i = 0; i < count; ++i) {
        types[i] = new char[256];
    }
    
    int actual_count = ovf_get_registered_node_types(types, count, 256);
    
    for (int i = 0; i < actual_count; ++i) {
        PyList_SET_ITEM(list, i, PyUnicode_FromString(types[i]));
    }
    
    for (int i = 0; i < count; ++i) {
        delete[] types[i];
    }
    delete[] types;
    
    return list;
}

static PyObject* py_get_node_type_info(PyObject* self, PyObject* args) {
    const char* type_id;
    
    if (!PyArg_ParseTuple(args, "s", &type_id)) {
        return nullptr;
    }
    
    char name[256], category[256], description[256];
    OVFErrorCode code = ovf_get_node_type_info(type_id, name, category, description, sizeof(name));
    if (code != OVF_SUCCESS) {
        set_python_error(code);
        return nullptr;
    }
    
    return Py_BuildValue("{ssssss}", 
                         "name", name,
                         "category", category,
                         "description", description);
}

static PyObject* py_enumerate_cameras(PyObject* self, PyObject* args) {
    const char* driver_type = "";
    
    if (!PyArg_ParseTuple(args, "|s", &driver_type)) {
        return nullptr;
    }
    
    int count = 10;  // 假设最多10个相机
    PyObject* list = PyList_New(0);
    if (!list) {
        return nullptr;
    }
    
    char** ids = new char*[count];
    char** names = new char*[count];
    for (int i = 0; i < count; ++i) {
        ids[i] = new char[256];
        names[i] = new char[256];
    }
    
    int actual_count = ovf_enumerate_cameras(driver_type, ids, names, count, 256);
    
    for (int i = 0; i < actual_count; ++i) {
        PyObject* dict = Py_BuildValue("{ssss}", "id", ids[i], "name", names[i]);
        PyList_Append(list, dict);
        Py_DECREF(dict);
    }
    
    for (int i = 0; i < count; ++i) {
        delete[] ids[i];
        delete[] names[i];
    }
    delete[] ids;
    delete[] names;
    
    return list;
}

static PyObject* py_get_last_error(PyObject* self, PyObject* Py_UNUSED(ignored)) {
    char buffer[512];
    ovf_get_last_error_message(buffer, sizeof(buffer));
    OVFErrorCode code = ovf_get_last_error_code();
    
    return Py_BuildValue("(is)", code, buffer);
}

static PyObject* py_clear_error(PyObject* self, PyObject* Py_UNUSED(ignored)) {
    ovf_clear_error();
    Py_RETURN_NONE;
}

static PyMethodDef module_methods[] = {
    {"initialize", py_initialize, METH_NOARGS, "初始化SDK"},
    {"shutdown", py_shutdown, METH_NOARGS, "关闭SDK"},
    {"get_version", py_get_version, METH_NOARGS, "获取SDK版本"},
    {"get_registered_node_types", py_get_registered_node_types, METH_NOARGS, "获取注册的节点类型"},
    {"get_node_type_info", py_get_node_type_info, METH_VARARGS, "获取节点类型信息"},
    {"enumerate_cameras", py_enumerate_cameras, METH_VARARGS, "枚举相机设备"},
    {"get_last_error", py_get_last_error, METH_NOARGS, "获取最后错误"},
    {"clear_error", py_clear_error, METH_NOARGS, "清除错误状态"},
    {nullptr, nullptr, 0, nullptr}
};

// ============================================================
// 模块定义
// ============================================================

static struct PyModuleDef ovf_module = {
    PyModuleDef_HEAD_INIT,
    "ovf",                               /* m_name */
    "OpenVisionFlow Python SDK - 开源工业机器视觉平台\n\n"
    "提供流程引擎、图像处理、相机接口等功能。\n\n"
    "示例:\n"
    "    import ovf\n"
    "    ovf.initialize()\n"
    "    engine = ovf.FlowEngine()\n"
    "    engine.load_flow('flow.json')\n"
    "    engine.run()\n"
    "    ovf.shutdown()\n",               /* m_doc */
    -1,                                  /* m_size */
    module_methods,                      /* m_methods */
    nullptr,                             /* m_slots */
    nullptr,                             /* m_traverse */
    nullptr,                             /* m_clear */
    nullptr                              /* m_free */
};

PyMODINIT_FUNC PyInit_ovf(void) {
    PyObject* m;
    
    if (PyType_Ready(&ImageType) < 0) return nullptr;
    if (PyType_Ready(&FlowEngineType) < 0) return nullptr;
    if (PyType_Ready(&CameraType) < 0) return nullptr;
    if (PyType_Ready(&FlowRunnerType) < 0) return nullptr;
    
    m = PyModule_Create(&ovf_module);
    if (m == nullptr) return nullptr;
    
    // 创建异常类
    OVFError = PyErr_NewException("ovf.Error", nullptr, nullptr);
    Py_INCREF(OVFError);
    if (PyModule_AddObject(m, "Error", OVFError) < 0) {
        Py_DECREF(OVFError);
        Py_DECREF(m);
        return nullptr;
    }
    
    // 添加类型到模块
    Py_INCREF(&ImageType);
    if (PyModule_AddObject(m, "Image", (PyObject*)&ImageType) < 0) {
        Py_DECREF(&ImageType);
        Py_DECREF(m);
        return nullptr;
    }
    
    Py_INCREF(&FlowEngineType);
    if (PyModule_AddObject(m, "FlowEngine", (PyObject*)&FlowEngineType) < 0) {
        Py_DECREF(&FlowEngineType);
        Py_DECREF(m);
        return nullptr;
    }
    
    Py_INCREF(&CameraType);
    if (PyModule_AddObject(m, "Camera", (PyObject*)&CameraType) < 0) {
        Py_DECREF(&CameraType);
        Py_DECREF(m);
        return nullptr;
    }
    
    Py_INCREF(&FlowRunnerType);
    if (PyModule_AddObject(m, "FlowRunner", (PyObject*)&FlowRunnerType) < 0) {
        Py_DECREF(&FlowRunnerType);
        Py_DECREF(m);
        return nullptr;
    }
    
    // 添加常量
    PyModule_AddIntConstant(m, "SUCCESS", OVF_SUCCESS);
    PyModule_AddIntConstant(m, "ERROR_UNKNOWN", OVF_ERROR_UNKNOWN);
    PyModule_AddIntConstant(m, "ERROR_INVALID_PARAM", OVF_ERROR_INVALID_PARAM);
    PyModule_AddIntConstant(m, "ERROR_NULL_POINTER", OVF_ERROR_NULL_POINTER);
    PyModule_AddIntConstant(m, "ERROR_NODE_NOT_FOUND", OVF_ERROR_NODE_NOT_FOUND);
    PyModule_AddIntConstant(m, "ERROR_DEVICE_NOT_FOUND", OVF_ERROR_DEVICE_NOT_FOUND);
    PyModule_AddIntConstant(m, "ERROR_EXECUTION_FAILED", OVF_ERROR_EXECUTION_FAILED);
    
    // 节点状态常量
    PyModule_AddIntConstant(m, "NODE_STATE_IDLE", OVF_NODE_STATE_IDLE);
    PyModule_AddIntConstant(m, "NODE_STATE_RUNNING", OVF_NODE_STATE_RUNNING);
    PyModule_AddIntConstant(m, "NODE_STATE_SUCCESS", OVF_NODE_STATE_SUCCESS);
    PyModule_AddIntConstant(m, "NODE_STATE_FAILED", OVF_NODE_STATE_FAILED);
    PyModule_AddIntConstant(m, "NODE_STATE_DISABLED", OVF_NODE_STATE_DISABLED);
    
    return m;
}