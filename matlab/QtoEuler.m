function [angles] = QtoEuler(refQuat)
% d=readCSV32('fuzz/fuzz-1.csv');
% refQuat=d.ref;

angles=zeros(length(refQuat),3);
    for i=1:length(refQuat)
        [angles(i,1),angles(i,2),angles(i,3)]=MtoEuler(QtoM(refQuat(i,:)));
    end

end
