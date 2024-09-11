# generated from rosidl_generator_py/resource/_idl.py.em
# with input from norlab_icp_mapper_ros:srv/SaveTrajectory.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SaveTrajectory_Request(type):
    """Metaclass of message 'SaveTrajectory_Request'."""

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
            module = import_type_support('norlab_icp_mapper_ros')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'norlab_icp_mapper_ros.srv.SaveTrajectory_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__save_trajectory__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__save_trajectory__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__save_trajectory__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__save_trajectory__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__save_trajectory__request

            from std_msgs.msg import String
            if String.__class__._TYPE_SUPPORT is None:
                String.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SaveTrajectory_Request(metaclass=Metaclass_SaveTrajectory_Request):
    """Message class 'SaveTrajectory_Request'."""

    __slots__ = [
        '_trajectory_file_name',
    ]

    _fields_and_field_types = {
        'trajectory_file_name': 'std_msgs/String',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'String'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import String
        self.trajectory_file_name = kwargs.get('trajectory_file_name', String())

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
        if self.trajectory_file_name != other.trajectory_file_name:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def trajectory_file_name(self):
        """Message field 'trajectory_file_name'."""
        return self._trajectory_file_name

    @trajectory_file_name.setter
    def trajectory_file_name(self, value):
        if __debug__:
            from std_msgs.msg import String
            assert \
                isinstance(value, String), \
                "The 'trajectory_file_name' field must be a sub message of type 'String'"
        self._trajectory_file_name = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_SaveTrajectory_Response(type):
    """Metaclass of message 'SaveTrajectory_Response'."""

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
            module = import_type_support('norlab_icp_mapper_ros')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'norlab_icp_mapper_ros.srv.SaveTrajectory_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__save_trajectory__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__save_trajectory__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__save_trajectory__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__save_trajectory__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__save_trajectory__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SaveTrajectory_Response(metaclass=Metaclass_SaveTrajectory_Response):
    """Message class 'SaveTrajectory_Response'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


class Metaclass_SaveTrajectory(type):
    """Metaclass of service 'SaveTrajectory'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('norlab_icp_mapper_ros')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'norlab_icp_mapper_ros.srv.SaveTrajectory')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__save_trajectory

            from norlab_icp_mapper_ros.srv import _save_trajectory
            if _save_trajectory.Metaclass_SaveTrajectory_Request._TYPE_SUPPORT is None:
                _save_trajectory.Metaclass_SaveTrajectory_Request.__import_type_support__()
            if _save_trajectory.Metaclass_SaveTrajectory_Response._TYPE_SUPPORT is None:
                _save_trajectory.Metaclass_SaveTrajectory_Response.__import_type_support__()


class SaveTrajectory(metaclass=Metaclass_SaveTrajectory):
    from norlab_icp_mapper_ros.srv._save_trajectory import SaveTrajectory_Request as Request
    from norlab_icp_mapper_ros.srv._save_trajectory import SaveTrajectory_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
