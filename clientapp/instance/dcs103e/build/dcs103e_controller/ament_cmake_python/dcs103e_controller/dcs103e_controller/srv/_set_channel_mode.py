# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dcs103e_controller:srv/SetChannelMode.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetChannelMode_Request(type):
    """Metaclass of message 'SetChannelMode_Request'."""

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
            module = import_type_support('dcs103e_controller')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dcs103e_controller.srv.SetChannelMode_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_channel_mode__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_channel_mode__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_channel_mode__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_channel_mode__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_channel_mode__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetChannelMode_Request(metaclass=Metaclass_SetChannelMode_Request):
    """Message class 'SetChannelMode_Request'."""

    __slots__ = [
        '_channel',
        '_mode',
        '_current',
        '_pulse_width',
        '_pulse_delay',
    ]

    _fields_and_field_types = {
        'channel': 'int32',
        'mode': 'string',
        'current': 'double',
        'pulse_width': 'double',
        'pulse_delay': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.channel = kwargs.get('channel', int())
        self.mode = kwargs.get('mode', str())
        self.current = kwargs.get('current', float())
        self.pulse_width = kwargs.get('pulse_width', float())
        self.pulse_delay = kwargs.get('pulse_delay', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
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
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.channel != other.channel:
            return False
        if self.mode != other.mode:
            return False
        if self.current != other.current:
            return False
        if self.pulse_width != other.pulse_width:
            return False
        if self.pulse_delay != other.pulse_delay:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def channel(self):
        """Message field 'channel'."""
        return self._channel

    @channel.setter
    def channel(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'channel' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'channel' field must be an integer in [-2147483648, 2147483647]"
        self._channel = value

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'mode' field must be of type 'str'"
        self._mode = value

    @builtins.property
    def current(self):
        """Message field 'current'."""
        return self._current

    @current.setter
    def current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'current' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._current = value

    @builtins.property
    def pulse_width(self):
        """Message field 'pulse_width'."""
        return self._pulse_width

    @pulse_width.setter
    def pulse_width(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pulse_width' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pulse_width' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pulse_width = value

    @builtins.property
    def pulse_delay(self):
        """Message field 'pulse_delay'."""
        return self._pulse_delay

    @pulse_delay.setter
    def pulse_delay(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pulse_delay' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pulse_delay' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pulse_delay = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SetChannelMode_Response(type):
    """Metaclass of message 'SetChannelMode_Response'."""

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
            module = import_type_support('dcs103e_controller')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dcs103e_controller.srv.SetChannelMode_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_channel_mode__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_channel_mode__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_channel_mode__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_channel_mode__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_channel_mode__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetChannelMode_Response(metaclass=Metaclass_SetChannelMode_Response):
    """Message class 'SetChannelMode_Response'."""

    __slots__ = [
        '_success',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
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
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


class Metaclass_SetChannelMode(type):
    """Metaclass of service 'SetChannelMode'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('dcs103e_controller')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dcs103e_controller.srv.SetChannelMode')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_channel_mode

            from dcs103e_controller.srv import _set_channel_mode
            if _set_channel_mode.Metaclass_SetChannelMode_Request._TYPE_SUPPORT is None:
                _set_channel_mode.Metaclass_SetChannelMode_Request.__import_type_support__()
            if _set_channel_mode.Metaclass_SetChannelMode_Response._TYPE_SUPPORT is None:
                _set_channel_mode.Metaclass_SetChannelMode_Response.__import_type_support__()


class SetChannelMode(metaclass=Metaclass_SetChannelMode):
    from dcs103e_controller.srv._set_channel_mode import SetChannelMode_Request as Request
    from dcs103e_controller.srv._set_channel_mode import SetChannelMode_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
