import csv
import numpy as np

inputFile = "C:/Users/ftief/OneDrive - University of Cambridge/Cambridge/Part IIB/IIB Project/LAT-Exp/fst29/processedData/Static friction/Percentage/positiveRuns.csv"
outputFile = "C:/Users/ftief/OneDrive - University of Cambridge/Cambridge/Part IIB/IIB Project/LAT-Exp/fst29/processedData//Static friction/Percentage/positiveSeparated.csv"
# inputFile = "C:/Users/ftief/OneDrive - University of Cambridge/Cambridge/Part IIB/IIB Project/LAT-Exp/fst29/processedData/test.csv"


resolution = 1024  # datapoints per revolutiion
n_measurements = 720  # total number of measurements

M = np.zeros((n_measurements, 2*resolution))  # Matrix describing
b = np.zeros((n_measurements, 1))  # sum of stictions


def mapAlpha(value):
    return int(value + resolution/2)


def mapBeta(value):
    return int(value + 3*resolution/2)


currentRow = 0

with open(inputFile) as f:
    reader = csv.reader(f)
    header = next(reader, None)
    for line in reader:
        b[currentRow] = float(line[3])
        M[currentRow, mapAlpha(int(line[0]))] = 1
        M[currentRow, mapBeta(int(line[1]))] = line[4]
        currentRow += 1

x = np.linalg.lstsq(M, b, rcond=None)[0]

print(x)


with open(outputFile, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(("Input position", "Input stiction", "Output position", "Output stiction"))

    for i in range(resolution):

        T1 = "" if abs(x[i, 0]) <= 1e-2 else x[i, 0]
        T2 = "" if abs(x[i+resolution, 0]) <= 1e-2 else x[i+resolution, 0]

        row = [i-resolution/2, T1, i-resolution/2,  T2]
        writer.writerow(row)
