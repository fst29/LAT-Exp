function delta = fMatchTorque(c, data1, data2)
delta = sum(abs(data1*c - data2));