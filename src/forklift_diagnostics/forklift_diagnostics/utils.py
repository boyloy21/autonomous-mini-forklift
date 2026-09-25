"""Shared helpers for forklift_diagnostics nodes."""

from forklift_interfaces.msg import DeviceStatus

OK = DeviceStatus.OK
WARNING = DeviceStatus.WARNING
ERROR = DeviceStatus.ERROR

def device(name, level, message='', value=0.0):
    """Build a DeviceStatus line item."""
    d = DeviceStatus()
    d.name = name
    d.level = level
    d.message = message
    d.value = float(value)
    return d

def level_from_range(value, warn_at, error_at):
    """Higher-is-worse threshold (usage %, temperature, current)"""
    if value >= error_at:
        return ERROR
    if value >= warn_at:
        return WARNING
    return OK

def level_from_low_range(value, warn_below, error_below):
    """Lower-is-worse threshold (battery voltage, health %)."""
    if value <= error_below:
        return ERROR
    if value <= warn_below:
        return WARNING
    return OK

def overall_level(devices):
    """Worst level across a list of DeviceStatus"""
    if any(d.level == ERROR for d in devices):
        return ERROR
    if any(d.level == WARNING for d in devices):
        return WARNING
    return OK

def level_name(level):
    return {OK: 'OK', WARNING: 'WARNING', ERROR: 'ERROR'}.get(level, 'UNKNOW')