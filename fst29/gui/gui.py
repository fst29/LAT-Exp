import _thread
import tkinter as tk
import tkinter.ttk as ttk
import os
from dataclasses import dataclass


validKeywords = ["CARRIAGE_GOTO", "DRIVE_GOTO", "DRIVE_SET_POS", "CARRIAGE_SET_POS", "OUTPUT_SET_POS", "DRIVE_SINE", "STATIC_FRICTION", "INITALISE_DRIVE", "INITIALISE_CARRAIGE"]  # Keywords that can be handled by the backend
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
    if command in ["STOP", "INITIALISE_DRIVE", "INITIALISE_CARRIAGE", "STATIC_FRICTION"]:
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
            return False

    if keyword in ["CARRIAGE_GOTO", "DRIVE_GOTO", "DRIVE_SET_POS", "CARRIAGE_SET_POS", "OUTPUT_SET_POS"]:
        if len(values) != 1:
            return False

    if keyword in ["DRIVE_SINE"]:
        if len(values) != 3:
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


def readPipes():
    """Reads the data coming from the backend"""
    while True:

        if usePipes:
            measurementPipe = open(measurementPipePath)
            rawString = measurementPipe.read()
            measurementPipe.close()
            print("message received: ", rawString)

        # Split the incoming message at each space
        separated = rawString.split(" ")

        if len(separated) != 16:
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

        updateMeasurements()  # Refresh the values on screen


# Create the window
window = tk.Tk()
window.geometry("800x400")
window.title("LAT-EXP")

# --------------------------------NUMPAD --------------------------------------
numpadFrame = tk.Frame(window)
numpadFrame.pack(expand=1, fill="both")
keypadButtons = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "0", ".", "DEL"]

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


# Add tabs
tabControl = ttk.Notebook(window)
manualTab = ttk.Frame(tabControl)
initialiseTab = ttk.Frame(tabControl)
sinusoidalTab = ttk.Frame(tabControl)

tabControl.add(manualTab, text='Manual')
tabControl.add(initialiseTab, text='Initialise')
tabControl.add(sinusoidalTab, text='Sinusoidal')

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

# -------------------------------------SINUSOIDAL TAB -------------------------------------------

driveSineFrame = tk.Frame(sinusoidalTab, borderwidth=3, relief=tk.RIDGE)
driveSineFrame.pack(expand=1, fill="both")
tk.Label(driveSineFrame, text="Sinusoidal drive movement").grid(row=0, column=0, columnspan=5, sticky=tk.EW)
tk.Label(driveSineFrame, text="Amplitude").grid(row=1, column=0)
tk.Label(driveSineFrame, text="Frequency").grid(row=1, column=1)
tk.Label(driveSineFrame, text="Offset").grid(row=1, column=2)
driveSineAmplitudeEntry = tk.Entry(driveSineFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
driveSineAmplitudeEntry.grid(row=2, column=0)
driveSineFrequencyEntry = tk.Entry(driveSineFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
driveSineFrequencyEntry.grid(row=2, column=1)
driveSineOffsetEntry = tk.Entry(driveSineFrame, text="", validate="all", validatecommand=(isNumberTCL, "%P"))
driveSineOffsetEntry.grid(row=2, column=2)

tk.Button(driveSineFrame, text="Start", command=lambda: sendCommand(f"DRIVE_SINE {driveSineAmplitudeEntry.get()} {driveSineFrequencyEntry.get()} {driveSineOffsetEntry.get()}")).grid(row=1, column=3, rowspan=2)

# Start a new thread that reads the incoming data
window.after(100, _thread.start_new_thread, readPipes, ())

window.mainloop()
