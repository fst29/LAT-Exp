import _thread
import tkinter as tk
import tkinter.ttk as ttk
import os
from dataclasses import dataclass


validKeywords = ["CARRIAGE_GOTO", "DRIVE_GOTO", "DRIVE_SET_POS", "CARRIAGE_SET_POS", "OUTPUT_SET_POS", "DRIVE_SINE", "CARRIAGE_SINE", "BOTH_SINE", "STATIC_FRICTION", "STATIC_FRICTION_WITH_PERCENTAGE", "INITALISE_DRIVE", "INITIALISE_CARRAIGE", "DYNAMIC_FRICTION", "PID_DRIVE", "SPRING_CHARACTERISATION"]  # Keywords that can be handled by the backend
# Names of named pipes, used for two-way communication with the backend
commandPipePath = "/home/pi/fst29/commands"
measurementPipePath = "/home/pi/fst29/measurements"


@dataclass
class motorMeasurements:
    position: float = 0
    velocity: float = 0
    current: float = 0


@dataclass
class measurementsClass:
    carriage: motorMeasurements = motorMeasurements(0, 0, 0)
    drive: motorMeasurements = motorMeasurements(0, 0, 0)
    output: motorMeasurements = motorMeasurements(0, 0, 0)
    p: float = 0
    command: str = ""
    state: str = ""


measurements = measurementsClass()

# Disable named pipes on windows
# Allows the gui to run without attemting to communicate with the backend
if os.name == "nt":
    # Running on windows
    usePipes = False
elif os.name == "posix":
    # Running on Linux
    usePipes = True

# Create new, empty pipes
if usePipes:
    try:
        os.remove(measurementPipePath)
    except FileNotFoundError:
        pass
    os.mkfifo(measurementPipePath)

    try:
        os.remove(commandPipePath)
    except FileNotFoundError:
        pass
    os.mkfifo(commandPipePath)


def keyPress(character):
    if isinstance(window.focus_get(), tk.Entry):
        if character == "DEL":
            window.focus_get().delete(len(window.focus_get().get())-1, tk.END)
        else:
            window.focus_get().insert(tk.END, character)


def validCommand(command):
    """Check whether the command could be handled by the backend"""
    if command in ["STOP", "INITIALISE_DRIVE", "INITIALISE_CARRIAGE", "STATIC_FRICTION", "SPRING_CHARACTERISATION"]:
        return True

    separated = command.split(" ")

    keyword = separated[0]
    values = separated[1:]

    if keyword not in validKeywords:
        return False

    for value in values:
        try:
            float(value)
        except ValueError:
            print(value)
            return False

    if keyword in ["CARRIAGE_GOTO", "DRIVE_GOTO", "DRIVE_SET_POS", "CARRIAGE_SET_POS", "OUTPUT_SET_POS"]:
        if len(values) != 1:
            return False

    if keyword in ["DYNAMIC_FRICTION"]:
        if len(values) != 2:
            return False

    if keyword in ["PID_DRIVE", "DRIVE_SINE", "CARRIAGE_SINE"]:
        if len(values) != 4:
            return False

    if keyword in ["BOTH_SINE"]:
        if len(values) != 8:
            return False

    return True


def sendCommand(command):
    """Sends the command over the named pipe"""
    if validCommand(command):
        print(command)
        if usePipes:
            commandPipe = open(commandPipePath, 'w')
            commandPipe.write(command)
            commandPipe.close()
    else:
        print("Invalid command")


def isNumber(value):
    """ Returns True for numbers that can be converted to floats, also returns True for empty strings and a single dash"""
    if value == "" or value == "-":
        return True
    try:
        float(value)
    except ValueError:
        return False
    return True


def updateMeasurements():
    """Updates the data displayed on the screen"""

    print("updating screen")
    carriagePositionLabel.config(text=str(measurements.carriage.position))
    carriageVelocityLabel.config(text=str(measurements.carriage.velocity)+"/s")
    carriageCurrentLabel.config(text=str(measurements.carriage.current))

    drivePositionLabel.config(text=str(measurements.drive.position)+"°")
    driveVelocityLabel.config(text=str(measurements.drive.velocity)+"°/s")
    driveCurrentLabel.config(text=str(measurements.drive.current))

    outputPositionLabel.config(text=str(measurements.output.position)+"°")
    outputVelocityLabel.config(text=str(measurements.output.velocity)+"°/s")

    commandLabel.config(text=measurements.command)
    stateLabel.config(text=measurements.state)


def readPipes():
    """Reads the data coming from the backend"""
    while True and usePipes:

        if usePipes:
            measurementPipe = open(measurementPipePath)
            rawString = measurementPipe.read()
            measurementPipe.close()
            print("message received: ", rawString)

        # Split the incoming message at each space
        separated = rawString.split(" ")

        if len(separated) != 20:
            print(f"Corrupted message received: {rawString}")
        else:

            if separated[0] == "CARRIAGE_POSITION" and separated[2] == "CARRIAGE_VELOCITY" and separated[4] == "CARRIAGE_CURRENT":
                measurements.carriage.position = float(separated[1])
                measurements.carriage.velocity = float(separated[3])
                measurements.carriage.current = float(separated[5])
            else:
                print(f"Corrupted carriage message received: {rawString}")

            if separated[6] == "DRIVE_POSITION" and separated[8] == "DRIVE_VELOCITY" and separated[10] == "DRIVE_CURRENT":
                measurements.drive.position = float(separated[7])
                measurements.drive.velocity = float(separated[9])
                measurements.drive.current = float(separated[11])

            else:
                print(f"Corrupted drive message received: {rawString}")

            if separated[12] == "OUTPUT_POSITION" and separated[14] == "OUTPUT_VELOCITY":
                measurements.output.position = float(separated[13])
                measurements.output.velocity = float(separated[15])

            else:
                print(f"Corrupted output message received: {rawString}")

            if separated[16] == "COMMAND" and separated[18] == "STATE":
                measurements.command = separated[17]
                measurements.state = separated[19]

            else:
                print(f"Corrupted output message received: {rawString}")

        updateMeasurements()  # Refresh the values on screen


# Create the window
window = tk.Tk()
window.geometry("800x400")
window.title("LAT-EXP")

# --------------------------------NUMPAD --------------------------------------
numpadFrame = tk.Frame(window)
numpadFrame.pack(expand=1, fill="both")
keypadButtons = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "0", ".", "-", "DEL"]

for i, name in enumerate(keypadButtons):
    tk.Button(numpadFrame, text=name, command=lambda name=name: keyPress(name), width=4, height=2).grid(column=i, row=0)

# --------------------------------DATA DISPLAY -------------------------------


dataFrame = tk.Frame(window)
dataFrame.pack(expand=1, fill="both")


tk.Label(dataFrame, text="Carriage").grid(row=0, column=0, columnspan=5, sticky=tk.EW)

tk.Label(dataFrame, text="p-value:").grid(row=1, column=0)
carriagePositionLabel = tk.Label(dataFrame, text="0")
carriagePositionLabel.grid(row=1, column=1)
tk.Label(dataFrame, text="change in p:").grid(row=1, column=2)
carriageVelocityLabel = tk.Label(dataFrame, text="0")
carriageVelocityLabel.grid(row=1, column=3)
tk.Label(dataFrame, text="Current:").grid(row=1, column=4)
carriageCurrentLabel = tk.Label(dataFrame, text="0")
carriageCurrentLabel.grid(row=1, column=5)


tk.Label(dataFrame, text="Drive").grid(row=2, column=0, columnspan=5, sticky=tk.EW)

tk.Label(dataFrame, text="Position:").grid(row=3, column=0)
drivePositionLabel = tk.Label(dataFrame, text="0")
drivePositionLabel.grid(row=3, column=1)
tk.Label(dataFrame, text="Velocity:").grid(row=3, column=2)
driveVelocityLabel = tk.Label(dataFrame, text="0")
driveVelocityLabel.grid(row=3, column=3)
tk.Label(dataFrame, text="Current:").grid(row=3, column=4)
driveCurrentLabel = tk.Label(dataFrame, text="0")
driveCurrentLabel.grid(row=3, column=5)


tk.Label(dataFrame, text="Output").grid(row=4, column=0, columnspan=5, sticky=tk.EW)

tk.Label(dataFrame, text="Position:").grid(row=5, column=0)
outputPositionLabel = tk.Label(dataFrame, text="0")
outputPositionLabel.grid(row=5, column=1)
tk.Label(dataFrame, text="Velocity:").grid(row=5, column=2)
outputVelocityLabel = tk.Label(dataFrame, text="0")
outputVelocityLabel.grid(row=5, column=3)


tk.Label(dataFrame, text="Control").grid(row=0, column=6, columnspan=5, sticky=tk.EW)

tk.Label(dataFrame, text="Command:").grid(row=1, column=6)
commandLabel = tk.Label(dataFrame, text="")
commandLabel.grid(row=1, column=7)
tk.Label(dataFrame, text="State:").grid(row=1, column=8)
stateLabel = tk.Label(dataFrame, text="")
stateLabel.grid(row=1, column=9)


# Add tabs
tabControl = ttk.Notebook(window)
manualTab = ttk.Frame(tabControl)
initialiseTab = ttk.Frame(tabControl)
sinusoidalTab = ttk.Frame(tabControl)
PIDTab = ttk.Frame(tabControl)

tabControl.add(manualTab, text='Manual')
tabControl.add(initialiseTab, text='Initialise')
tabControl.add(sinusoidalTab, text='Sinusoidal')
tabControl.add(PIDTab, text='PID')

tabControl.pack(expand=1, fill="both")

isNumberTCL = window.register(isNumber)  # Create TCL function from python function

# Stop button
tk.Button(window, text="STOP", command=lambda: sendCommand("STOP")).pack(expand=1, fill="both")

# --------------------------------MANUAL TAB -----------------------------------------

# Populate manual tab
carriageFrame = tk.Frame(manualTab, borderwidth=3, relief=tk.RIDGE)
carriageFrame.pack(expand=1, fill="both")
tk.Label(carriageFrame, text="Carriage").grid(row=0, column=0, columnspan=5, sticky=tk.EW)


tk.Button(carriageFrame, text="Move left", command=lambda: sendCommand(f"CARRIAGE_GOTO {measurements.carriage.position-0.05}")).grid(row=2, column=0)
tk.Button(carriageFrame, text="Return to one", command=lambda: sendCommand(f"CARRIAGE_GOTO {1}")).grid(row=2, column=1)
tk.Button(carriageFrame, text="Move right", command=lambda: sendCommand(f"CARRIAGE_GOTO {measurements.carriage.position+0.05}")).grid(row=2, column=2)
carriageEntry = tk.Entry(carriageFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
carriageEntry.grid(row=2, column=3)
tk.Button(carriageFrame, text="Go to position", command=lambda: sendCommand(f"CARRIAGE_GOTO {carriageEntry.get()}")).grid(row=2, column=4)

driveFrame = ttk.Frame(manualTab, borderwidth=3, relief=tk.RIDGE)
driveFrame.pack(expand=1, fill="both")
tk.Label(driveFrame, text="Drive").grid(row=0, column=0, columnspan=10, sticky=tk.EW)

tk.Button(driveFrame, text="Move left", command=lambda: sendCommand(f"DRIVE_GOTO {measurements.drive.position-5}")).grid(row=2, column=0)
tk.Button(driveFrame, text="Return to zero", command=lambda: sendCommand(f"DRIVE_GOTO {0}")).grid(row=2, column=1)
tk.Button(driveFrame, text="Move right", command=lambda: sendCommand(f"DRIVE_GOTO {measurements.drive.position+5}")).grid(row=2, column=2)
driveEntry = tk.Entry(driveFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
driveEntry.grid(row=2, column=3)
tk.Button(driveFrame, text="Go to position", command=lambda: sendCommand(f"DRIVE_GOTO {driveEntry.get()}")).grid(row=2, column=4)


updateFrame = tk.Frame(manualTab, borderwidth=3, relief=tk.RIDGE)
tk.Label(updateFrame, text="Manually set position").grid(row=0, column=0, columnspan=5, sticky=tk.EW)
updateFrame.pack(expand=1, fill="both")
updateEntry = tk.Entry(updateFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
updateEntry.grid(row=1, column=0)
tk.Button(updateFrame, text="Set carriage position", command=lambda: sendCommand(f"CARRIAGE_SET_POS {updateEntry.get()}")).grid(row=1, column=1)
tk.Button(updateFrame, text="Set drive position", command=lambda: sendCommand(f"DRIVE_SET_POS {updateEntry.get()}")).grid(row=1, column=2)
tk.Button(updateFrame, text="Set output position", command=lambda: sendCommand(f"OUTPUT_SET_POS {updateEntry.get()}")).grid(row=1, column=3)

# -------------------------------------INITIALISE TAB -------------------------------------------
tk.Button(initialiseTab, text="Initialise drive", command=lambda: sendCommand("INITIALISE_DRIVE")).grid(row=0, column=0)
tk.Button(initialiseTab, text="Initialise carriage", command=lambda: sendCommand("INITIALISE_CARRIAGE")).grid(row=0, column=1)

tk.Button(initialiseTab, text="Static friction", command=lambda: sendCommand("STATIC_FRICTION")).grid(row=0, column=2)
tk.Button(initialiseTab, text="Static friction with %", command=lambda: sendCommand("STATIC_FRICTION_WITH_PERCENTAGE")).grid(row=0, column=3)

tk.Button(initialiseTab, text="Spring characterisation", command=lambda: sendCommand("SPRING_CHARACTERISATION")).grid(row=0, column=4)

dynamicFrictionFrame = tk.Frame(initialiseTab, borderwidth=3, relief=tk.RIDGE)
dynamicFrictionFrame.grid(column=0, row=1, columnspan=4)
tk.Label(dynamicFrictionFrame, text="Dynamic friction").grid(row=0, column=0, columnspan=4, sticky=tk.EW)
tk.Label(dynamicFrictionFrame, text="Speed (def: 200)").grid(row=1, column=0)
tk.Label(dynamicFrictionFrame, text="Acceleration (def: 800)").grid(row=1, column=1)

dynamicFrictionSpeedEntry = tk.Entry(dynamicFrictionFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
dynamicFrictionSpeedEntry.grid(row=2, column=0)
dynamicFrictionAccelerationEntry = tk.Entry(dynamicFrictionFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
dynamicFrictionAccelerationEntry.grid(row=2, column=1)


tk.Button(dynamicFrictionFrame, text="Start", command=lambda: sendCommand(f"DYNAMIC_FRICTION {dynamicFrictionSpeedEntry.get()} {dynamicFrictionAccelerationEntry.get()}")).grid(row=1, column=3, rowspan=2)

# -------------------------------------SINUSOIDAL TAB -------------------------------------------

sineFrame = tk.Frame(sinusoidalTab, borderwidth=3, relief=tk.RIDGE)
sineFrame.pack(expand=1, fill="both")
tk.Label(sineFrame, text="Sinusoidal drive movement").grid(row=0, column=0, columnspan=5, sticky=tk.EW)
tk.Label(sineFrame, text="Drive amplitude (°)").grid(row=1, column=0)
tk.Label(sineFrame, text="Drive frequency (Hz)").grid(row=1, column=1)
tk.Label(sineFrame, text="Drive phase (°)").grid(row=1, column=2)
tk.Label(sineFrame, text="Drive offset (°)").grid(row=1, column=3)
driveSineAmplitudeEntry = tk.Entry(sineFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
driveSineAmplitudeEntry.grid(row=2, column=0)
driveSineFrequencyEntry = tk.Entry(sineFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
driveSineFrequencyEntry.grid(row=2, column=1)
driveSinePhaseEntry = tk.Entry(sineFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
driveSinePhaseEntry.grid(row=2, column=2)
driveSineOffsetEntry = tk.Entry(sineFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
driveSineOffsetEntry.grid(row=2, column=3)

tk.Label(sineFrame, text="Sinusoidal movement").grid(row=0, column=0, columnspan=5, sticky=tk.EW)
tk.Label(sineFrame, text="Carriage amplitude (°)").grid(row=3, column=0)
tk.Label(sineFrame, text="Carriage frequency (Hz)").grid(row=3, column=1)
tk.Label(sineFrame, text="Carriage phase (°)").grid(row=3, column=2)
tk.Label(sineFrame, text="Carriage offset (°)").grid(row=3, column=3)
carriageSineAmplitudeEntry = tk.Entry(sineFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
carriageSineAmplitudeEntry.grid(row=4, column=0)
carriageSineFrequencyEntry = tk.Entry(sineFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
carriageSineFrequencyEntry.grid(row=4, column=1)
carriageSinePhaseEntry = tk.Entry(sineFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
carriageSinePhaseEntry.grid(row=4, column=2)
carriageSineOffsetEntry = tk.Entry(sineFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
carriageSineOffsetEntry.grid(row=4, column=3)

tk.Button(sineFrame, text="Start drive", command=lambda: sendCommand(f"DRIVE_SINE {driveSineAmplitudeEntry.get()} {driveSineFrequencyEntry.get()} {driveSinePhaseEntry.get()} {driveSineOffsetEntry.get()}")).grid(row=1, column=4, rowspan=4)
tk.Button(sineFrame, text="Start carriage", command=lambda: sendCommand(f"CARRIAGE_SINE {carriageSineAmplitudeEntry.get()} {carriageSineFrequencyEntry.get()} {carriageSinePhaseEntry.get()} {carriageSineOffsetEntry.get()}")).grid(row=1, column=5, rowspan=4)
tk.Button(sineFrame, text="Start both", command=lambda: sendCommand(f"BOTH_SINE {driveSineAmplitudeEntry.get()} {driveSineFrequencyEntry.get()} {driveSinePhaseEntry.get()} {driveSineOffsetEntry.get()} {carriageSineAmplitudeEntry.get()} {carriageSineFrequencyEntry.get()} {carriageSinePhaseEntry.get()} {carriageSineOffsetEntry.get()}")).grid(row=1, column=6, rowspan=4)

# -------------------------------------PID TAB -------------------------------------------

PIDFrame = tk.Frame(PIDTab, borderwidth=3, relief=tk.RIDGE)
PIDFrame.pack(expand=1, fill="both")
tk.Label(PIDFrame, text="Drive motor PID tuning").grid(row=0, column=0, columnspan=5, sticky=tk.EW)
tk.Label(PIDFrame, text="Kp").grid(row=1, column=0)
tk.Label(PIDFrame, text="Kd").grid(row=1, column=1)
tk.Label(PIDFrame, text="Ki").grid(row=1, column=2)
tk.Label(PIDFrame, text="Kf").grid(row=1, column=3)

kpEntry = tk.Entry(PIDFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
kpEntry.grid(row=2, column=0)
kdEntry = tk.Entry(PIDFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
kdEntry.grid(row=2, column=1)
kiEntry = tk.Entry(PIDFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
kiEntry.grid(row=2, column=2)
kfEntry = tk.Entry(PIDFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
kfEntry.grid(row=2, column=3)


tk.Button(PIDFrame, text="Update parameters", command=lambda: sendCommand(f"PID_DRIVE {kpEntry.get()} {kdEntry.get()} {kiEntry.get()} {kfEntry.get()}")).grid(row=1, column=4, rowspan=2)

# Start a new thread that reads the incoming data
window.after(100, _thread.start_new_thread, readPipes, ())

window.mainloop()
