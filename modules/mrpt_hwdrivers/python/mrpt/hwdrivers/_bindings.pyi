"""
Python bindings for mrpt::hwdrivers: sensor drivers
"""
from __future__ import annotations
import datetime
import mrpt.config
import mrpt.serialization
import typing
__all__: list[str] = ['CGPSInterface', 'CGenericSensor', 'CHokuyoURG', 'CJoystick', 'CRoboPeakLidar', 'CTaoboticsIMU', 'CVelodyneScanner']
class CGenericSensor:
    class TSensorState:
        """
        Members:
        
          ssInitializing
        
          ssWorking
        
          ssError
        
          ssUninitialized
        """
        __members__: typing.ClassVar[dict[str, CGenericSensor.TSensorState]]
        ssError: typing.ClassVar[CGenericSensor.TSensorState]
        ssInitializing: typing.ClassVar[CGenericSensor.TSensorState]
        ssUninitialized: typing.ClassVar[CGenericSensor.TSensorState]
        ssWorking: typing.ClassVar[CGenericSensor.TSensorState]
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    ssError: typing.ClassVar[CGenericSensor.TSensorState]
    ssInitializing: typing.ClassVar[CGenericSensor.TSensorState]
    ssUninitialized: typing.ClassVar[CGenericSensor.TSensorState]
    ssWorking: typing.ClassVar[CGenericSensor.TSensorState]
    @staticmethod
    def createSensor(className: str) -> CGenericSensor:
        """
        Creates a sensor driver by its class name (e.g. 'CGPSInterface'), or returns None if the class is unknown. Configure it with loadConfig().
        """
    def __repr__(self) -> str:
        ...
    def doProcess(self) -> None:
        """
        Reads from the device. Call it periodically, e.g. at getProcessRate() Hz.
        """
    def enableVerbose(self, enabled: bool = True) -> None:
        ...
    def getClassName(self) -> str:
        ...
    def getObservations(self) -> list[tuple[datetime.timedelta, mrpt.serialization.CSerializable]]:
        """
        Returns (and removes) the observations gathered so far, as a list of (timestamp, observation) tuples
        """
    def getProcessRate(self) -> float:
        """
        Suggested doProcess() rate (Hz)
        """
    def getSensorLabel(self) -> str:
        ...
    def getState(self) -> CGenericSensor.TSensorState:
        ...
    def initialize(self) -> None:
        """
        Opens the device and prepares it for capturing (call after configuring)
        """
    def loadConfig(self, configSource: mrpt.config.CConfigFileBase, section: str) -> None:
        """
        Loads the sensor parameters from a config file section
        """
    def setPathForExternalImages(self, directory: str) -> None:
        """
        For camera sensors: directory where to save images as external files
        """
    def setSensorLabel(self, sensorLabel: str) -> None:
        ...
class CTaoboticsIMU(CGenericSensor):
    def __init__(self) -> None:
        ...
    def setSerialBaudRate(self, rate: int) -> None:
        ...
    def setSerialPort(self, serialPort: str) -> None:
        ...
class CGPSInterface(CGenericSensor):
    def __init__(self) -> None:
        ...
    def getSerialPortName(self) -> str:
        ...
    def setSerialPortName(self, COM_port: str) -> None:
        ...
    def setSetupCommands(self, cmds: list[str]) -> None:
        ...
    def setSetupCommandsDelay(self, delay_secs: float) -> None:
        ...
    def setShutdownCommands(self, cmds: list[str]) -> None:
        ...
class CHokuyoURG(CGenericSensor):
    def __init__(self) -> None:
        ...
    def setIPandPort(self, ip: str, port: int) -> None:
        ...
    def setReducedFOV(self, fov: float) -> None:
        ...
    def setScanInterval(self, skipScanCount: int) -> None:
        ...
    def setSerialPort(self, port_name: str) -> None:
        ...
class CRoboPeakLidar(CGenericSensor):
    def __init__(self) -> None:
        ...
    def setSerialPort(self, port_name: str) -> None:
        ...
class CVelodyneScanner(CGenericSensor):
    class model_t:
        """
        Members:
        
          VLP16
        
          HDL32
        
          HDL64
        """
        HDL32: typing.ClassVar[CVelodyneScanner.model_t]
        HDL64: typing.ClassVar[CVelodyneScanner.model_t]
        VLP16: typing.ClassVar[CVelodyneScanner.model_t]
        __members__: typing.ClassVar[dict[str, CVelodyneScanner.model_t]]
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    HDL32: typing.ClassVar[CVelodyneScanner.model_t]
    HDL64: typing.ClassVar[CVelodyneScanner.model_t]
    VLP16: typing.ClassVar[CVelodyneScanner.model_t]
    def __init__(self) -> None:
        ...
    def setDeviceIP(self, ip: str) -> None:
        ...
    def setModelName(self, model: CVelodyneScanner.model_t) -> None:
        ...
    def setPCAPInputFile(self, pcap_file: str) -> None:
        """
        Replays packets from a PCAP file instead of reading from the network
        """
class CJoystick:
    class State:
        buttons: list[bool]
        def __init__(self) -> None:
            ...
        @property
        def axes(self) -> list[float]:
            """
            Normalized axis positions
            """
        @axes.setter
        def axes(self, arg0: list[float]) -> None:
            ...
        @property
        def axes_raw(self) -> list[int]:
            """
            Raw axis positions
            """
        @axes_raw.setter
        def axes_raw(self, arg0: list[int]) -> None:
            ...
    @staticmethod
    def getJoysticksCount() -> int:
        ...
    def __init__(self) -> None:
        ...
    def getJoystickPosition(self, nJoy: int = 0) -> CJoystick.State | None:
        """
        Reads the state of joystick nJoy, or None on error
        """
    def setLimits(self, minPerAxis: list[int], maxPerAxis: list[int]) -> None:
        """
        Sets the raw range of each axis, used to normalize positions
        """
