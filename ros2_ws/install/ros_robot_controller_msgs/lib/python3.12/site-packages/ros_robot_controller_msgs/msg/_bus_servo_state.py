# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros_robot_controller_msgs:msg/BusServoState.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

# Member 'present_id'
# Member 'target_id'
# Member 'position'
# Member 'offset'
# Member 'voltage'
# Member 'temperature'
# Member 'position_limit'
# Member 'voltage_limit'
# Member 'max_temperature_limit'
# Member 'enable_torque'
# Member 'save_offset'
# Member 'stop'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BusServoState(type):
    """Metaclass of message 'BusServoState'."""

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
                'ros_robot_controller_msgs.msg.BusServoState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__bus_servo_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__bus_servo_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__bus_servo_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__bus_servo_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__bus_servo_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BusServoState(metaclass=Metaclass_BusServoState):
    """Message class 'BusServoState'."""

    __slots__ = [
        '_present_id',
        '_target_id',
        '_position',
        '_offset',
        '_voltage',
        '_temperature',
        '_position_limit',
        '_voltage_limit',
        '_max_temperature_limit',
        '_enable_torque',
        '_save_offset',
        '_stop',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'present_id': 'sequence<uint16>',
        'target_id': 'sequence<uint16>',
        'position': 'sequence<uint16>',
        'offset': 'sequence<int16>',
        'voltage': 'sequence<uint16>',
        'temperature': 'sequence<uint16>',
        'position_limit': 'sequence<uint16>',
        'voltage_limit': 'sequence<uint16>',
        'max_temperature_limit': 'sequence<uint16>',
        'enable_torque': 'sequence<uint16>',
        'save_offset': 'sequence<uint16>',
        'stop': 'sequence<uint16>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
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
        self.present_id = array.array('H', kwargs.get('present_id', []))
        self.target_id = array.array('H', kwargs.get('target_id', []))
        self.position = array.array('H', kwargs.get('position', []))
        self.offset = array.array('h', kwargs.get('offset', []))
        self.voltage = array.array('H', kwargs.get('voltage', []))
        self.temperature = array.array('H', kwargs.get('temperature', []))
        self.position_limit = array.array('H', kwargs.get('position_limit', []))
        self.voltage_limit = array.array('H', kwargs.get('voltage_limit', []))
        self.max_temperature_limit = array.array('H', kwargs.get('max_temperature_limit', []))
        self.enable_torque = array.array('H', kwargs.get('enable_torque', []))
        self.save_offset = array.array('H', kwargs.get('save_offset', []))
        self.stop = array.array('H', kwargs.get('stop', []))

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
        if self.present_id != other.present_id:
            return False
        if self.target_id != other.target_id:
            return False
        if self.position != other.position:
            return False
        if self.offset != other.offset:
            return False
        if self.voltage != other.voltage:
            return False
        if self.temperature != other.temperature:
            return False
        if self.position_limit != other.position_limit:
            return False
        if self.voltage_limit != other.voltage_limit:
            return False
        if self.max_temperature_limit != other.max_temperature_limit:
            return False
        if self.enable_torque != other.enable_torque:
            return False
        if self.save_offset != other.save_offset:
            return False
        if self.stop != other.stop:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def present_id(self):
        """Message field 'present_id'."""
        return self._present_id

    @present_id.setter
    def present_id(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'H', \
                    "The 'present_id' array.array() must have the type code of 'H'"
                self._present_id = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'present_id' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._present_id = array.array('H', value)

    @builtins.property
    def target_id(self):
        """Message field 'target_id'."""
        return self._target_id

    @target_id.setter
    def target_id(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'H', \
                    "The 'target_id' array.array() must have the type code of 'H'"
                self._target_id = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'target_id' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._target_id = array.array('H', value)

    @builtins.property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'H', \
                    "The 'position' array.array() must have the type code of 'H'"
                self._position = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'position' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._position = array.array('H', value)

    @builtins.property
    def offset(self):
        """Message field 'offset'."""
        return self._offset

    @offset.setter
    def offset(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'h', \
                    "The 'offset' array.array() must have the type code of 'h'"
                self._offset = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -32768 and val < 32768 for val in value)), \
                "The 'offset' field must be a set or sequence and each value of type 'int' and each integer in [-32768, 32767]"
        self._offset = array.array('h', value)

    @builtins.property
    def voltage(self):
        """Message field 'voltage'."""
        return self._voltage

    @voltage.setter
    def voltage(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'H', \
                    "The 'voltage' array.array() must have the type code of 'H'"
                self._voltage = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'voltage' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._voltage = array.array('H', value)

    @builtins.property
    def temperature(self):
        """Message field 'temperature'."""
        return self._temperature

    @temperature.setter
    def temperature(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'H', \
                    "The 'temperature' array.array() must have the type code of 'H'"
                self._temperature = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'temperature' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._temperature = array.array('H', value)

    @builtins.property
    def position_limit(self):
        """Message field 'position_limit'."""
        return self._position_limit

    @position_limit.setter
    def position_limit(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'H', \
                    "The 'position_limit' array.array() must have the type code of 'H'"
                self._position_limit = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'position_limit' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._position_limit = array.array('H', value)

    @builtins.property
    def voltage_limit(self):
        """Message field 'voltage_limit'."""
        return self._voltage_limit

    @voltage_limit.setter
    def voltage_limit(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'H', \
                    "The 'voltage_limit' array.array() must have the type code of 'H'"
                self._voltage_limit = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'voltage_limit' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._voltage_limit = array.array('H', value)

    @builtins.property
    def max_temperature_limit(self):
        """Message field 'max_temperature_limit'."""
        return self._max_temperature_limit

    @max_temperature_limit.setter
    def max_temperature_limit(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'H', \
                    "The 'max_temperature_limit' array.array() must have the type code of 'H'"
                self._max_temperature_limit = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'max_temperature_limit' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._max_temperature_limit = array.array('H', value)

    @builtins.property
    def enable_torque(self):
        """Message field 'enable_torque'."""
        return self._enable_torque

    @enable_torque.setter
    def enable_torque(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'H', \
                    "The 'enable_torque' array.array() must have the type code of 'H'"
                self._enable_torque = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'enable_torque' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._enable_torque = array.array('H', value)

    @builtins.property
    def save_offset(self):
        """Message field 'save_offset'."""
        return self._save_offset

    @save_offset.setter
    def save_offset(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'H', \
                    "The 'save_offset' array.array() must have the type code of 'H'"
                self._save_offset = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'save_offset' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._save_offset = array.array('H', value)

    @builtins.property
    def stop(self):
        """Message field 'stop'."""
        return self._stop

    @stop.setter
    def stop(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'H', \
                    "The 'stop' array.array() must have the type code of 'H'"
                self._stop = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'stop' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._stop = array.array('H', value)
