# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros_robot_controller_msgs:msg/GetBusServoCmd.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetBusServoCmd(type):
    """Metaclass of message 'GetBusServoCmd'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ros_robot_controller_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros_robot_controller_msgs.msg.GetBusServoCmd')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__get_bus_servo_cmd
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__get_bus_servo_cmd
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__get_bus_servo_cmd
            cls._TYPE_SUPPORT = module.type_support_msg__msg__get_bus_servo_cmd
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__get_bus_servo_cmd

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetBusServoCmd(metaclass=Metaclass_GetBusServoCmd):
    """Message class 'GetBusServoCmd'."""

    __slots__ = [
        '_id',
        '_get_id',
        '_get_position',
        '_get_offset',
        '_get_voltage',
        '_get_temperature',
        '_get_position_limit',
        '_get_voltage_limit',
        '_get_max_temperature_limit',
        '_get_torque_state',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'id': 'uint8',
        'get_id': 'uint8',
        'get_position': 'uint8',
        'get_offset': 'uint8',
        'get_voltage': 'uint8',
        'get_temperature': 'uint8',
        'get_position_limit': 'uint8',
        'get_voltage_limit': 'uint8',
        'get_max_temperature_limit': 'uint8',
        'get_torque_state': 'uint8',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id = kwargs.get('id', int())
        self.get_id = kwargs.get('get_id', int())
        self.get_position = kwargs.get('get_position', int())
        self.get_offset = kwargs.get('get_offset', int())
        self.get_voltage = kwargs.get('get_voltage', int())
        self.get_temperature = kwargs.get('get_temperature', int())
        self.get_position_limit = kwargs.get('get_position_limit', int())
        self.get_voltage_limit = kwargs.get('get_voltage_limit', int())
        self.get_max_temperature_limit = kwargs.get('get_max_temperature_limit', int())
        self.get_torque_state = kwargs.get('get_torque_state', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.id != other.id:
            return False
        if self.get_id != other.get_id:
            return False
        if self.get_position != other.get_position:
            return False
        if self.get_offset != other.get_offset:
            return False
        if self.get_voltage != other.get_voltage:
            return False
        if self.get_temperature != other.get_temperature:
            return False
        if self.get_position_limit != other.get_position_limit:
            return False
        if self.get_voltage_limit != other.get_voltage_limit:
            return False
        if self.get_max_temperature_limit != other.get_max_temperature_limit:
            return False
        if self.get_torque_state != other.get_torque_state:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'id' field must be an unsigned integer in [0, 255]"
        self._id = value

    @builtins.property
    def get_id(self):
        """Message field 'get_id'."""
        return self._get_id

    @get_id.setter
    def get_id(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'get_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'get_id' field must be an unsigned integer in [0, 255]"
        self._get_id = value

    @builtins.property
    def get_position(self):
        """Message field 'get_position'."""
        return self._get_position

    @get_position.setter
    def get_position(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'get_position' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'get_position' field must be an unsigned integer in [0, 255]"
        self._get_position = value

    @builtins.property
    def get_offset(self):
        """Message field 'get_offset'."""
        return self._get_offset

    @get_offset.setter
    def get_offset(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'get_offset' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'get_offset' field must be an unsigned integer in [0, 255]"
        self._get_offset = value

    @builtins.property
    def get_voltage(self):
        """Message field 'get_voltage'."""
        return self._get_voltage

    @get_voltage.setter
    def get_voltage(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'get_voltage' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'get_voltage' field must be an unsigned integer in [0, 255]"
        self._get_voltage = value

    @builtins.property
    def get_temperature(self):
        """Message field 'get_temperature'."""
        return self._get_temperature

    @get_temperature.setter
    def get_temperature(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'get_temperature' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'get_temperature' field must be an unsigned integer in [0, 255]"
        self._get_temperature = value

    @builtins.property
    def get_position_limit(self):
        """Message field 'get_position_limit'."""
        return self._get_position_limit

    @get_position_limit.setter
    def get_position_limit(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'get_position_limit' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'get_position_limit' field must be an unsigned integer in [0, 255]"
        self._get_position_limit = value

    @builtins.property
    def get_voltage_limit(self):
        """Message field 'get_voltage_limit'."""
        return self._get_voltage_limit

    @get_voltage_limit.setter
    def get_voltage_limit(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'get_voltage_limit' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'get_voltage_limit' field must be an unsigned integer in [0, 255]"
        self._get_voltage_limit = value

    @builtins.property
    def get_max_temperature_limit(self):
        """Message field 'get_max_temperature_limit'."""
        return self._get_max_temperature_limit

    @get_max_temperature_limit.setter
    def get_max_temperature_limit(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'get_max_temperature_limit' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'get_max_temperature_limit' field must be an unsigned integer in [0, 255]"
        self._get_max_temperature_limit = value

    @builtins.property
    def get_torque_state(self):
        """Message field 'get_torque_state'."""
        return self._get_torque_state

    @get_torque_state.setter
    def get_torque_state(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'get_torque_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'get_torque_state' field must be an unsigned integer in [0, 255]"
        self._get_torque_state = value
