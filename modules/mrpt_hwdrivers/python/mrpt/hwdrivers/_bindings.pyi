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
    """
    A generic interface for a wide-variety of sensors designed to be used in the application RawLogGrabber.
    """
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
        """
        Enables or disables extra debug output.
        """
    def getClassName(self) -> str:
        """
        Returns the name of the driver class.
        """
    def getObservations(self) -> list[tuple[datetime.timedelta, mrpt.serialization.CSerializable]]:
        """
        Returns (and removes) the observations gathered so far, as a list of (timestamp, observation) tuples
        """
    def getProcessRate(self) -> float:
        """
        Suggested doProcess() rate (Hz)
        """
    def getSensorLabel(self) -> str:
        """
        Returns the sensor label, copied into each observation.
        """
    def getState(self) -> CGenericSensor.TSensorState:
        """
        The current state of the sensor.
        """
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
        """
        Sets the sensor label, copied into each observation.
        """
class CTaoboticsIMU(CGenericSensor):
    """
    A driver for Taobotics IMU.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def setSerialBaudRate(self, rate: int) -> None:
        """
        Sets the serial port baud rate (default: 921600). Call before initialize().
        """
    def setSerialPort(self, serialPort: str) -> None:
        """
        Sets the serial port device (default: /dev/ttyUSB0). Call before initialize().
        """
class CGPSInterface(CGenericSensor):
    """
    Reads GPS/GNSS receiver data from a serial port or any input stream and parses it into CObservationGPS observations.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def getSerialPortName(self) -> str:
        """
        Returns the currently configured serial port device name.
        """
    def setSerialPortName(self, COM_port: str) -> None:
        """
        Sets the serial port device name (e.g. "COM1", "ttyUSB0").
        """
    def setSetupCommands(self, cmds: list[str]) -> None:
        """
        Sets the commands sent to the receiver after opening the port.
        """
    def setSetupCommandsDelay(self, delay_secs: float) -> None:
        """
        Sets the delay between setup commands, in seconds.
        """
    def setShutdownCommands(self, cmds: list[str]) -> None:
        """
        Sets the commands sent to the receiver before closing the port.
        """
class CHokuyoURG(CGenericSensor):
    """
    Driver for Hokuyo URG/UTM/UXM/UST 2-D laser range-finders via the SCIP-2.0 protocol over USB serial or Ethernet.
    """
    def __init__(self) -> None:
        """
        Constructor.
        """
    def setIPandPort(self, ip: str, port: int) -> None:
        """
        Configures the IP address and TCP port for Ethernet connection.
        """
    def setReducedFOV(self, fov: float) -> None:
        """
        Restricts the angular field of view of the scanner.
        """
    def setScanInterval(self, skipScanCount: int) -> None:
        """
        Sets the scan decimation factor.
        """
    def setSerialPort(self, port_name: str) -> None:
        """
        Configures the serial port device name for USB/serial connection.
        """
class CRoboPeakLidar(CGenericSensor):
    """
    Interfaces a Robo Peak LIDAR laser scanner.
    """
    def __init__(self) -> None:
        """
        Constructor.
        """
    def setSerialPort(self, port_name: str) -> None:
        """
        Sets the serial port device of the scanner.
        """
class CVelodyneScanner(CGenericSensor):
    """
    Driver for Velodyne lidars (HDL-64, HDL-32, VLP-16, ...).
    """
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
        """
        Default constructor.
        """
    def setDeviceIP(self, ip: str) -> None:
        """
        Only accepts UDP packets from this IP address (empty: any address).
        """
    def setModelName(self, model: CVelodyneScanner.model_t) -> None:
        """
        Sets the scanner model (e.g. VLP16, HDL32, HDL64).
        """
    def setPCAPInputFile(self, pcap_file: str) -> None:
        """
        Replays packets from a PCAP file instead of reading from the network
        """
class CJoystick:
    """
    Reads axis positions and button states from joysticks and gamepads.
    """
    class State:
        """
        Joystick state: button states and axis positions.
        """
        buttons: list[bool]
        def __init__(self) -> None:
            """
            Default constructor.
            """
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
        """
        Returns the number of joystick/gamepad devices currently connected to the system.
        """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def getJoystickPosition(self, nJoy: int = 0) -> CJoystick.State | None:
        """
        Reads the state of joystick nJoy, or None on error
        """
    def setLimits(self, minPerAxis: list[int], maxPerAxis: list[int]) -> None:
        """
        Sets the raw range of each axis, used to normalize positions
        """
