import csv
import matplotlib.pyplot as plt


dataFile = "C:/Users/ftief/OneDrive - University of Cambridge/Cambridge/Part IIB/IIB Project/LAT-Exp/fst29/processedData/Dynamic friction/forFitting.csv"
outputFolder = "C:/Users/ftief/OneDrive - University of Cambridge/Cambridge/Part IIB/IIB Project/Final report/Dynamic friction plots/"


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
            againstInput = []
            againstOutput = []
            measured = []
            quadratic = []
            sinusoidal = []
            print(speed, p, offset)
            for row in inRangePoints:
                if speed in row[COMMAND][:-4] and "1"+speed not in row[COMMAND][:-4] and abs(float(row[CARRIAGEPOS]) - p) <= 0.15 and float(row[OFFSET]) == offset:
                    againstOutput.append(float(row[OUTPUTPOS]))
                    againstInput.append(float(row[INPUTPOS]))
                    measured.append(float(row[FRICTION]))
                    quadratic.append(float(row[QUADRATIC]))
                    sinusoidal.append(float(row[SINUSOIDAL]))

            print(len(x))

             #plt.figure()
            #ax = plt.axes()
            fig, (ax1, ax2) = plt.subplots(1, 2, sharey=True, figsize=(8, 4))
            ax1.scatter(againstInput, measured, label="Measured")
            ax1.scatter(againstInput, quadratic, label="Quadratic fit")
            ax1.scatter(againstInput, sinusoidal, label="Sinusoidal fit")
            ax1.set_xlabel("Primary shaft position (deg)")
            ax1.set_ylabel("Friction (Nm)")
            ax1.yaxis.tick_right()
            ax1.yaxis.set_label_position("right")
            ax1.legend()

            ax2.scatter(againstOutput, measured, label="Measured")
            ax2.scatter(againstOutput, quadratic, label="Quadratic fit")
            ax2.scatter(againstOutput, sinusoidal, label="Sinusoidal fit")
            ax2.set_xlabel("Secondary shaft position (deg)")
            #ax2.set_ylabel("Friction (Nm)")
            ax2.legend()

            
            title = f"speed: {speed}, p: {p}, offset: {offset}"
            filename = f"speed{speed}, p{p}, offset{offset}"
            fig.suptitle(title)
            plt.subplots_adjust(bottom=0.11,left=0.01, right=0.99, top=0.93)
            #plt.set_size_cm()
            plt.savefig(outputFolder+"dynFric"+filename+".png")
            #plt.show()
