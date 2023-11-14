import csv

dataFolder = "C:/Users/ftief/OneDrive - University of Cambridge/Cambridge/Part IIB/IIB Project/LAT-Exp/fst29/processedData/"

inputFile = "First run.csv"
outputFile = "First run peaks.csv"


rawData = []


def findTorque(driveTimeStamp):

    for i, data in enumerate(cellData):
        if int(data[0]) > int(driveTimeStamp):
           # print(data[0], driveTimeStamp)
            return cellData[i-1][1]




with open(dataFolder+inputFile) as f:
    reader = csv.reader(f)

    for line in reader:
        rawData.append(line)


cellData = []
driveData = []

processed = [("Drive_position", "Output_position", "Current", "Torque")]

for line in rawData[1:]:
    driveData.append(line[:5])
    cellData.append(line[5:])


for i, data in enumerate(driveData):
    try:
        if data[2] == "ramp_up" and driveData[i+1][2]=="cooldown":
            
            
            processed.append( ( data[3], data[4], data[1], findTorque(data[0]) ) )


    except IndexError:
        pass 


#print(processed)

with open(dataFolder+outputFile, "w", newline="") as f:
    writer=csv.writer(f)

    writer.writerows(processed)

