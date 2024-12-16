# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros_robot_controller_msgs:msg/RGBState.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RGBState(type):
    """Metaclass of message 'RGBState'."""

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
                'ros_robot_controller_msgs.msg.RGBState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__rgb_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__rgb_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__rgb_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__rgb_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__rgb_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RGBState(metaclass=Metaclass_RGBState):
    """Message class 'RGBState'."""

    __slots__ = [
        '_index',
        '_red',
        '_green',
        '_blue',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'index': 'int32',
        'red': 'uint8',
        'green': 'uint8',
        'blue': 'uint8',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
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
        self.index = kwargs.get('index', int())
        self.red = kwargs.get('red', int())
        self.green = kwargs.get('green', int())
        self.blue = kwargs.get('blue', int())

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
        if self.index != other.index:
            return False
        if self.red != other.red:
            return False
        if self.green != other.green:
            return False
        if self.blue != other.blue:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def index(self):
        """Message field 'index'."""
        return self._index

    @index.setter
    def index(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'index' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'index' field must be an integer in [-2147483648, 2147483647]"
        self._index = value

    @builtins.property
    def red(self):
        """Message field 'red'."""
        return self._red

    @red.setter
    def red(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'red' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'red' field must be an unsigned integer in [0, 255]"
        self._red = value

    @builtins.property
    def green(self):
        """Message field 'green'."""
        return self._green

    @green.setter
    def green(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'green' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'green' field must be an unsigned integer in [0, 255]"
        self._green = value

    @builtins.property
    def blue(self):
        """Message field 'blue'."""
        return self._blue

    @blue.setter
    def blue(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'blue' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'blue' field must be an unsigned integer in [0, 255]"
        self._blue = value
