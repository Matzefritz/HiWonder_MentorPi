// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ros_robot_controller_msgs:msg/GetBusServoCmd.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "ros_robot_controller_msgs/msg/detail/get_bus_servo_cmd__struct.h"
#include "ros_robot_controller_msgs/msg/detail/get_bus_servo_cmd__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool ros_robot_controller_msgs__msg__get_bus_servo_cmd__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[64];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("ros_robot_controller_msgs.msg._get_bus_servo_cmd.GetBusServoCmd", full_classname_dest, 63) == 0);
  }
  ros_robot_controller_msgs__msg__GetBusServoCmd * ros_message = _ros_message;
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // get_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "get_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->get_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // get_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "get_position");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->get_position = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // get_offset
    PyObject * field = PyObject_GetAttrString(_pymsg, "get_offset");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->get_offset = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // get_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "get_voltage");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->get_voltage = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // get_temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "get_temperature");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->get_temperature = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // get_position_limit
    PyObject * field = PyObject_GetAttrString(_pymsg, "get_position_limit");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->get_position_limit = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // get_voltage_limit
    PyObject * field = PyObject_GetAttrString(_pymsg, "get_voltage_limit");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->get_voltage_limit = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // get_max_temperature_limit
    PyObject * field = PyObject_GetAttrString(_pymsg, "get_max_temperature_limit");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->get_max_temperature_limit = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // get_torque_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "get_torque_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->get_torque_state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros_robot_controller_msgs__msg__get_bus_servo_cmd__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GetBusServoCmd */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros_robot_controller_msgs.msg._get_bus_servo_cmd");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GetBusServoCmd");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros_robot_controller_msgs__msg__GetBusServoCmd * ros_message = (ros_robot_controller_msgs__msg__GetBusServoCmd *)raw_ros_message;
  {  // id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // get_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->get_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "get_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // get_position
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->get_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "get_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // get_offset
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->get_offset);
    {
      int rc = PyObject_SetAttrString(_pymessage, "get_offset", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // get_voltage
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->get_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "get_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // get_temperature
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->get_temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "get_temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // get_position_limit
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->get_position_limit);
    {
      int rc = PyObject_SetAttrString(_pymessage, "get_position_limit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // get_voltage_limit
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->get_voltage_limit);
    {
      int rc = PyObject_SetAttrString(_pymessage, "get_voltage_limit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // get_max_temperature_limit
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->get_max_temperature_limit);
    {
      int rc = PyObject_SetAttrString(_pymessage, "get_max_temperature_limit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // get_torque_state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->get_torque_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "get_torque_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
