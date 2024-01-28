import csv
import numpy as np
import scipy


dataFile = "C:/Users/ftief/OneDrive - University of Cambridge/Cambridge/Part IIB/IIB Project/LAT-Exp/fst29/processedData/Static friction/Percentage/combined.csv"

datapoints = 4197

x = np.zeros((3,datapoints))
y = np.zeros(datapoints)




with open(dataFile) as f:
    reader = csv.reader(f)

    header = next(reader)

    for i, row in enumerate(reader):
        x[0][i] = row[0]
        x[1][i] = row[1]
        x[2][i] = row[2]
        y[i] = row[3]


def firstModel(x, a1, b1, c1, a2, b2, c2):
    inputPosition = x[0]
    outputPosition = x[1]
    p = x[2]

    return a1 * inputPosition ** 2 + b1 * inputPosition + c1 + (a2 * outputPosition ** 2 + b2 * outputPosition +c2) / p


def secondModel(x, a1, b1, c1, d1, a2, b2, c2, d2):
    inputPosition = x[0]
    outputPosition = x[1]
    p = x[2]

    return a1 * inputPosition ** 3 + b1 * inputPosition ** 2 + c1 * inputPosition + d1 + (a2 * outputPosition ** 3 + b2 * outputPosition ** 2 + c2 * inputPosition + d2) / p


def thirdModel(x, a1, b1, c1, a2, b2, c2, a3, b3, c3):
    inputPosition = x[0]
    outputPosition = x[1]
    p = x[2]

    return  a1 * inputPosition ** 2 + b1 * inputPosition + c1 + (a2 * outputPosition ** 2 + b2 * outputPosition +c2) / p + (a3 * outputPosition ** 2 + b3 * outputPosition +c3) / (1+p)

def fourthModel(x, a1, b1, c1, a2, b2, c2, a3, b3, c3):
    inputPosition = x[0]
    outputPosition = x[1]
    p = x[2]

    return  a1 * inputPosition ** 2 + b1 * inputPosition + c1 + (a2 * outputPosition ** 2 + b2 * outputPosition +c2) / p + (a3 * inputPosition ** 2 + b3 * inputPosition +c3) / (1+p)

def fifthModel(x, a1, b1, c1, a2, b2, c2, c3):
    inputPosition = x[0]
    outputPosition = x[1]
    p = x[2]

    return  a1 * inputPosition ** 2 + b1 * inputPosition + c1 + (a2 * outputPosition ** 2 + b2 * outputPosition +c2) / p + (c3) / (1+p)



def degToRad(degree):
    return degree / 180 * np.pi


def sixthModel(x, c1, c2, c3, a1, a2, p1, p2):
    inputPosition = x[0]
    outputPosition = x[1]
    p = x[2]

    return  a1 * np.sin( degToRad(inputPosition) + p1 )+ c1 + ( a2*np.sin( degToRad(outputPosition) + p2 )+c2) / p + (c3) / (1+p)
  


def seventhModel(x, c1, c2, c3, ai1, ai2, ai3, pi1, pi2, pi3, ao1, ao2, ao3, po1, po2, po3):
    inputPosition = x[0]
    outputPosition = x[1]
    p = x[2]

    return  (ai1 * np.sin( degToRad(inputPosition) + pi1 ) + ai2* np.sin(2*degToRad(inputPosition) +pi2 ) + ai3* np.sin(3*degToRad(inputPosition) +pi3 ) + c1 +
             ( ao1 * np.sin( degToRad(outputPosition) + po1 ) + ao2* np.sin(2*degToRad(inputPosition) +po2 )+ao3* np.sin(3*degToRad(inputPosition) +po3 )+ c2) / p +
             (c3) / (1+p))
  




popt, pcov, infodict, mesg, ier = scipy.optimize.curve_fit(firstModel, x, y, full_output=True)

print(popt, mesg)

MSE = 0

for i in range(len(x)):
    MSE += (firstModel(x[:, i], *popt) - y[i])**2

print("1: ",MSE)



popt, pcov, infodict, mesg, ier = scipy.optimize.curve_fit(secondModel, x, y, full_output=True)

print(popt, mesg)


MSE = 0
for i in range(len(x)):
    MSE += (secondModel(x[:, i], *popt) - y[i])**2

print("2: ",MSE)


popt, pcov, infodict, mesg, ier = scipy.optimize.curve_fit(thirdModel, x, y, full_output=True)

print(popt, mesg)


MSE = 0
for i in range(len(x)):
    MSE += (thirdModel(x[:, i], *popt) - y[i])**2

print("3: ",MSE)

popt, pcov, infodict, mesg, ier = scipy.optimize.curve_fit(fourthModel, x, y, full_output=True)

print(popt, mesg)


MSE = 0
for i in range(len(x)):
    MSE += (fourthModel(x[:, i], *popt) - y[i])**2

print("4: ",MSE)


popt, pcov, infodict, mesg, ier = scipy.optimize.curve_fit(fifthModel, x, y, full_output=True)

print(popt, mesg)


MSE = 0
for i in range(len(x)):
    MSE += (fifthModel(x[:, i], *popt) - y[i])**2

print("5: ",MSE)






popt, pcov, infodict, mesg, ier = scipy.optimize.curve_fit(sixthModel, x, y, full_output=True)

print(popt, mesg)


MSE = 0
for i in range(len(x)):
    MSE += (sixthModel(x[:, i], *popt) - y[i])**2

print("6: ",MSE)



popt, pcov, infodict, mesg, ier = scipy.optimize.curve_fit(seventhModel, x, y, full_output=True)

print(popt, mesg)


MSE = 0
for i in range(len(x)):
    MSE += (seventhModel(x[:, i], *popt) - y[i])**2

print("7: ",MSE)


