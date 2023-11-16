import csv
import os

inputPath = "C:/Users/ftief/OneDrive - University of Cambridge/Cambridge/Part IIB/IIB Project/LAT-Exp/fst29/data/Static friction/Third run/"
outputPath = "C:/Users/ftief/OneDrive - University of Cambridge/Cambridge/Part IIB/IIB Project/LAT-Exp/fst29/processedData/thirdRunCellData"


allInputFiles = os.listdir(inputPath)
inputFiles = []

for file in allInputFiles:
    if file[0] != "D":
        inputFiles.append(file)

data = []

for file in inputFiles:

    with open(inputPath+file) as f:
        reader = csv.reader(f)

        for row in reader:
            data.append(row)


with open(outputPath+"1.csv", "w", newline="") as f:
    writer = csv.writer(f)

    writer.writerows(data)
