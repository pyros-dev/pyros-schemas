from __future__ import absolute_import
from __future__ import print_function

import inspect
import importlib
import roslib
from .exceptions import InvalidClassException, InvalidModuleException, InvalidPackageException, InvalidTypeStringException


def _splittype(typestring):
    """ Split the string the / delimiter and strip out empty strings
    Performs similar logic to roslib.names.package_resource_name but is a bit
    more forgiving about excess slashes
    """
    splits = [x for x in typestring.split("/") if x]
    if len(splits) == 2:
        return splits
    raise InvalidTypeStringException(typestring)


# TODO  : check genpy.Message for get_message_class and get_service_class
def _get_msg_class(typestring):
    """ If not loaded, loads the specified msg class then returns an instance
    of it
    Throws various exceptions if loading the msg class fails
    """
    return _get_class(typestring, "msg")
    # return genpy.get_message_class(typestring, reload_on_error=True)


def _get_srv_class(typestring):
    """ If not loaded, loads the specified srv class then returns an instance
    of it

    Throws various exceptions if loading the srv class fails """
    global _loaded_srvs, _srvs_lock
    return _get_class(typestring, "srv")


def _get_class(typestring, subname):
    """ If not loaded, loads the specified class then returns an instance
    of it.
    Loaded classes are cached in the provided cache dict
    Throws various exceptions if loading the msg class fails
    """
    # normalise the typestring
    modname, classname = _splittype(typestring)
    norm_typestring = modname + "/" + classname

    # Load the class
    cls = _load_class(modname, subname, classname)

    return cls


def _load_class(modname, subname, classname):
    """ Loads the manifest and imports the module that contains the specified
    type.

    Logic is similar to that of roslib.message.get_message_class, but we want
    more expressive exceptions.

    Returns the loaded module, or None on failure """
    global loaded_modules

    try:
        # roslib maintains a cache of loaded manifests, so no need to duplicate
        roslib.launcher.load_manifest(modname)
    except Exception as exc:
        raise InvalidPackageException(modname, exc)

    try:
        pymod = importlib.import_module('{0!s}.{1!s}'.format(modname, subname))
    except Exception as exc:
        raise InvalidModuleException(modname, subname, exc)

    try:
        return getattr(pymod, classname)
    except Exception as exc:
        raise InvalidClassException(modname, subname, classname, exc)


def _get_rosmsg_members_as_dict(inst):
    if hasattr(inst, '__slots__'):  # ROS style
        slots = []
        ancestors = inspect.getmro(type(inst))
        for a in ancestors:
            slots += set(a.__slots__) if hasattr(a, '__slots__') else set()
        # Remove special ROS slots
        if '_connection_header' in slots:
            slots.remove('_connection_header')
        data_dict = {
            slot: getattr(inst, slot)
            for slot in slots
        }
    elif hasattr(inst, '__dict__'):  # generic python object (for tests)
        data_dict = vars(inst)
    else:  # this is a basic python type (including dict and list)
        data_dict = inst
    return data_dict


def _get_rosmsg_fields_as_dict(cls):
    if hasattr(cls, '__slots__'):  # ROS style
        slots = []
        slots_types = []
        ancestors = inspect.getmro(cls)
        for a in ancestors:
            slots += a.__slots__ if hasattr(a, '__slots__') else []
            slots_types += a._slot_types if hasattr(a, '_slot_types') else []
        # Remove special ROS slots
        if '_connection_header' in slots:
            slots.remove('_connection_header')
        data_dict = dict(zip(slots, slots_types))
    elif hasattr(cls, '__dict__'):  # generic python object (for tests)
        data_dict = {k: type(v) for k, v in vars(cls).items()}
    else:  # this is a basic python type (including dict)
        data_dict = cls
    return data_dict