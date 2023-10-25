function syncdata = fSyncData_v3(data1, data2)
% data1 is RP
% data2 is LC
%% Resample
% RP data
names = fieldnames(data1);
Ts = 1/100;
[aTime, bT, ~] = unique([0; data1.time]);
time = 0:Ts:data1.time(end);
for i = 1:length(names)
    if strcmp(names{i},'time')
        data = [0; data1.(names{i})];
    else
        data = [data1.(names{i})(1); data1.(names{i})];
    end
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
dDate1 = find((diff(data1.date)>0));
dDate2 = find((diff(data2.date)>0));
bDate = find(data1.date(dDate1(1)) == data2.date(dDate2));
% iDate = dDate2(bDate)-dDate1(1) - 1;
iDate = dDate2(bDate)-dDate1(1) - 3;
name2 = fieldnames(data2);
for i = 1:length(name2)
    syncdata.LC.(name2{i}) = data2.(name2{i})(iDate+1:length(data1.time)+iDate);
end 
syncdata.RP = data1;

time = 0:Ts:syncdata.RP.time(end);
syncdata.RP.time = time';
syncdata.LC.time = time';
