import csv

dataFolder = "C:/Users/ftief/OneDrive - University of Cambridge/Cambridge/Part IIB/IIB Project/LAT-Exp/fst29/processedData/Static friction/Percentage/Third run/"

inputFiles = "Third run.csv"  # ["Second run1.csv", "Second run2.csv"]
outputFile = "Third run peaks.csv"


rawData = []


def findTorque(driveTimeStamp):

    for i, data in enumerate(cellData):
        # find first drop
        if int(data[0]) > int(driveTimeStamp) and cellData[i+1][1] < data[1]:
            return data[0], data[1]


# for inputFile in inputFiles:
with open(dataFolder+inputFiles) as f:
    reader = csv.reader(f)

    for line in reader:
        rawData.append(line)


cellData = []
driveData = []

processed = [("Drive time", "Drive_position", "Output_position", "Current", "Cell time", "Torque")]

for line in rawData[1:]:
    driveData.append(line[:5])
    cellData.append(line[5:])


for i, data in enumerate(driveData):
    try:
        if data[2] == "ramp_up" and driveData[i+1][2] == "cooldown":
            print(data[3])

            row = [data[0], data[3], data[4], data[1]]
            row.extend(findTorque(data[0]))

            processed.append(row)

    except IndexError:
        print("Reached the end")


with open(dataFolder+outputFile, "w", newline="") as f:
    writer = csv.writer(f)

    writer.writerows(processed)
