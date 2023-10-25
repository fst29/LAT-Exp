function syncdata = fSyncData_v4(data1, data2)
% data1 is RP
% data2 is LC
%% Resample
% RP data
names = fieldnames(data1);
Ts = 1/100;
[aTime, bT, ~] = unique([0; data1.time]);
time = 0:Ts:data1.time(end);
for i = 1:length(names)
    data = [data1.(names{i})(1); data1.(names{i})];
%     data = [0; data1.(names{i})];
    data1.(names{i}) = interp1(aTime, data(bT), time)';
end

% LC data
names = fieldnames(data2);
Ts = 1/100;
time = 0:Ts:data2.time(end);
aTime = data2.time;
for i = 1:length(names)
    data2.(names{i}) = interp1(aTime, data2.(names{i}), time)';
end

% Get data number
n1 = length(data1.time);
n2 = length(data2.time);
dn = n1-n2;
for i = 1:abs(dn)
    if dn < 0
        dataN1 = [zeros(i, 1); data1.torque; zeros(abs(dn)-i,1)];
        dataN2 = data2.torque;
        RMS(i) = sum(abs(abs(dataN1)/sum(abs(dataN1)) - abs(dataN2)/sum(abs(dataN2))));
    else
        dataN2 = [zeros(i, 1); data2.torque; zeros(abs(dn)-i,1)];
        dataN1 = data1.torque;
        RMS(i) = sum(abs(abs(dataN1)/sum(abs(dataN1)) - abs(dataN2)/sum(abs(dataN2))));
    end
end
[~, index] = min(RMS);
% index1 = find(abs(data1.torque) > 0.02, 1);
% index2 = find(abs(data2.torque) > 0.02, 1);
% index = index2-index1;
index = index + 3;
% index = 1;

if dn < 0
    name1 = fieldnames(data1);
    for i = 1:length(name1)
        syncdata.RP.(name1{i}) = [zeros(index, 1); data1.(name1{i}); zeros(abs(dn)-index,1)];
    end
    name2 = fieldnames(data2);
    for i = 1:length(name2)
        syncdata.LC.(name2{i}) = data2.(name2{i});
    end
else
    name2 = fieldnames(data2);
    for i = 1:length(name2)
        syncdata.LC.(name2{i}) = [zeros(index, 1); data2.(name2{i}); zeros(abs(dn)-index,1)];
    end
    name1 = fieldnames(data1);
    for i = 1:length(name1)
        syncdata.RP.(name1{i}) = data1.(name1{i});
    end
end

name1 = fieldnames(data1);
for i = 1:length(name1)
    syncdata.RP.(name1{i}) = syncdata.RP.(name1{i})(index+1:end-(abs(dn)-index));
end
name2 = fieldnames(data2);
for i = 1:length(name2)
    syncdata.LC.(name2{i}) = syncdata.LC.(name2{i})(index+1:end-(abs(dn)-index));
end 
direction = lowpass(sign(syncdata.LC.torque), 10, 100);
% syncdata.RP.torqueSign = -direction.*syncdata.RP.torque;
stop = true;
Ts = 1/100;
time = 0:Ts:syncdata.RP.time(end);
syncdata.RP.time = time';
syncdata.LC.time = time';
