import csv
import matplotlib.pyplot as plt
folder = "C:/Users/ftief/OneDrive - University of Cambridge/Cambridge/Part IIB/IIB Project/LAT-Exp/fst29/processedData/Spring characterisation/"
file = "p0.63"

data = []
targets = set()
with open(folder+file+".csv", "r") as f:

    reader = csv.reader(f)
    header = next(reader)

    for row in reader:
        data.append((float(row[6]), float(row[9]), float(row[15]), float(row[17])))  #  target pos, i/p pos, o/p pos, torque 
        targets.add(float(row[6]))

print(targets)

targets = list(targets)
avgTorques = []
maxTorques = []
outputPos_s = []

outputData = []

for target in targets:
    torques = []

    inputPos = []
    outputPos = []
    for row in data:
        if row[0] == target:
            torques.append(row[3])
            outputPos.append(row[2])
            inputPos.append(row[1])
    avgTorques.append(sum(torques)/len(torques))
    avgOutputPos = sum(outputPos)/len(outputPos)
    avgInputPos = sum(inputPos)/len(inputPos)
    outputPos_s.append(avgOutputPos)
    if avgOutputPos >=0:
        maxTorques.append(max(torques))
    else:
        maxTorques.append(min(torques))
    
    outputData.append((avgInputPos, avgOutputPos, maxTorques[-1]))

plt.scatter(outputPos_s, maxTorques)
plt.show()


with open(folder+file+" peaks.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(("Input", "Output", "Torque"))
    writer.writerows(outputData)