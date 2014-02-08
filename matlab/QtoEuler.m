function [angles] = QtoEuler(refQuat)
% d=readCSV32('fuzz/fuzz-1.csv');
% refQuat=d.ref;

angles=zeros(size(refQuat,1),3);
    for i=1:size(refQuat,1)
        [angles(i,1),angles(i,2),angles(i,3)]=MtoEuler(QtoM(refQuat(i,:)));
    end

end
