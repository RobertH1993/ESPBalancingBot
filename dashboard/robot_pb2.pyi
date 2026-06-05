from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class PidType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    BALANCE: _ClassVar[PidType]
    SPEED: _ClassVar[PidType]
    POSITION: _ClassVar[PidType]
    WHEEL_TRIM: _ClassVar[PidType]
BALANCE: PidType
SPEED: PidType
POSITION: PidType
WHEEL_TRIM: PidType

class SetPidParams(_message.Message):
    __slots__ = ("target", "kp", "ki", "kd")
    TARGET_FIELD_NUMBER: _ClassVar[int]
    KP_FIELD_NUMBER: _ClassVar[int]
    KI_FIELD_NUMBER: _ClassVar[int]
    KD_FIELD_NUMBER: _ClassVar[int]
    target: PidType
    kp: float
    ki: float
    kd: float
    def __init__(self, target: _Optional[_Union[PidType, str]] = ..., kp: _Optional[float] = ..., ki: _Optional[float] = ..., kd: _Optional[float] = ...) -> None: ...

class GetPidParams(_message.Message):
    __slots__ = ("target",)
    TARGET_FIELD_NUMBER: _ClassVar[int]
    target: PidType
    def __init__(self, target: _Optional[_Union[PidType, str]] = ...) -> None: ...

class GetPidParamsReply(_message.Message):
    __slots__ = ("target", "kp", "ki", "kd", "setpoint")
    TARGET_FIELD_NUMBER: _ClassVar[int]
    KP_FIELD_NUMBER: _ClassVar[int]
    KI_FIELD_NUMBER: _ClassVar[int]
    KD_FIELD_NUMBER: _ClassVar[int]
    SETPOINT_FIELD_NUMBER: _ClassVar[int]
    target: PidType
    kp: float
    ki: float
    kd: float
    setpoint: float
    def __init__(self, target: _Optional[_Union[PidType, str]] = ..., kp: _Optional[float] = ..., ki: _Optional[float] = ..., kd: _Optional[float] = ..., setpoint: _Optional[float] = ...) -> None: ...

class SetPidSetpoint(_message.Message):
    __slots__ = ("target", "setpoint")
    TARGET_FIELD_NUMBER: _ClassVar[int]
    SETPOINT_FIELD_NUMBER: _ClassVar[int]
    target: PidType
    setpoint: float
    def __init__(self, target: _Optional[_Union[PidType, str]] = ..., setpoint: _Optional[float] = ...) -> None: ...

class PidTelemetry(_message.Message):
    __slots__ = ("target", "p", "i", "d", "integral", "error")
    TARGET_FIELD_NUMBER: _ClassVar[int]
    P_FIELD_NUMBER: _ClassVar[int]
    I_FIELD_NUMBER: _ClassVar[int]
    D_FIELD_NUMBER: _ClassVar[int]
    INTEGRAL_FIELD_NUMBER: _ClassVar[int]
    ERROR_FIELD_NUMBER: _ClassVar[int]
    target: PidType
    p: float
    i: float
    d: float
    integral: float
    error: float
    def __init__(self, target: _Optional[_Union[PidType, str]] = ..., p: _Optional[float] = ..., i: _Optional[float] = ..., d: _Optional[float] = ..., integral: _Optional[float] = ..., error: _Optional[float] = ...) -> None: ...

class ConfigMessage(_message.Message):
    __slots__ = ("transaction_id", "set_pid_paramsp", "get_pid_paramsp", "get_pid_params_replyp", "set_pid_setpointp")
    TRANSACTION_ID_FIELD_NUMBER: _ClassVar[int]
    SET_PID_PARAMSP_FIELD_NUMBER: _ClassVar[int]
    GET_PID_PARAMSP_FIELD_NUMBER: _ClassVar[int]
    GET_PID_PARAMS_REPLYP_FIELD_NUMBER: _ClassVar[int]
    SET_PID_SETPOINTP_FIELD_NUMBER: _ClassVar[int]
    transaction_id: int
    set_pid_paramsp: SetPidParams
    get_pid_paramsp: GetPidParams
    get_pid_params_replyp: GetPidParamsReply
    set_pid_setpointp: SetPidSetpoint
    def __init__(self, transaction_id: _Optional[int] = ..., set_pid_paramsp: _Optional[_Union[SetPidParams, _Mapping]] = ..., get_pid_paramsp: _Optional[_Union[GetPidParams, _Mapping]] = ..., get_pid_params_replyp: _Optional[_Union[GetPidParamsReply, _Mapping]] = ..., set_pid_setpointp: _Optional[_Union[SetPidSetpoint, _Mapping]] = ...) -> None: ...

class TelemetryMessage(_message.Message):
    __slots__ = ("sequence", "pid_telemetryp")
    SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    PID_TELEMETRYP_FIELD_NUMBER: _ClassVar[int]
    sequence: int
    pid_telemetryp: PidTelemetry
    def __init__(self, sequence: _Optional[int] = ..., pid_telemetryp: _Optional[_Union[PidTelemetry, _Mapping]] = ...) -> None: ...
