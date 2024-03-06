import csv
import matplotlib.pyplot as plt


dataFile = "C:/Users/ftief/OneDrive - University of Cambridge/Cambridge/Part IIB/IIB Project/LAT-Exp/fst29/processedData/Dynamic friction/forFitting.csv"
outputFolder = "C:/Users/ftief/OneDrive - University of Cambridge/Cambridge/Part IIB/IIB Project/LAT-Exp/fst29/processedData/Dynamic friction/plots/"


TIME = 0
COMMAND = 1
STATE = 2
INPUTPOS = 3
CARRIAGEPOS = 4
OUTPUTPOS = 5
ACCELERATION = 6
TORQUE = 7
FRICTION = 8
INRANGE = 9
OFFSET = 10
QUADRATIC = 11
SINUSOIDAL = 12

inRangePoints = []

speeds = ["200", "400", "600", "800", "1000", "1200"]
pvalues = [0.63, 1, 1.42]
offsets = [20, -60, -120]


with open(dataFile) as f:
    reader = csv.reader(f)

    header = next(reader)
    for row in reader:
        if row[INRANGE] == "1":
            inRangePoints.append(row)


for speed in speeds:
    for p in pvalues:
        for offset in offsets:
            x = []
            measured = []
            quadratic = []
            sinusoidal = []
            print(speed, p, offset)
            for row in inRangePoints:
                if speed in row[COMMAND][:-4] and "1"+speed not in row[COMMAND][:-4] and abs(float(row[CARRIAGEPOS]) - p) <= 0.15 and float(row[OFFSET]) == offset:
                    #x.append(float(row[OUTPUTPOS]))
                    x.append(float(row[INPUTPOS]))
                    measured.append(float(row[FRICTION]))
                    quadratic.append(float(row[QUADRATIC]))
                    sinusoidal.append(float(row[SINUSOIDAL]))

            print(len(x))

            ax = plt.axes()

            ax.scatter(x, measured, label="Measured")
            ax.scatter(x, quadratic, label="Quadratic fit")
            ax.scatter(x, sinusoidal, label="Sinusoidal fit")
            ax.set_xlabel("Primary shaft position (deg)")
            ax.set_ylabel("Friction (Nm)")
            ax.legend()
            title = f"speed: {speed}, p: {p}, offset: {offset}"
            filename = f"speed{speed}, p{p}, offset{offset}"
            ax.set_title(title)
            plt.savefig(outputFolder+"FvsI"+filename+".png")
            plt.clf()
