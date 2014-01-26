function [qe] = calcQuatError(qref,qmeas)


for a=1:length(qref) 
    
%     qref=d.ref;
%     qmeas=d.quat;
    Qt = [qref(a, 4), qref(a, 3), -qref(a, 2), qref(a, 1);
          -qref(a, 3), qref(a, 4), qref(a, 1), qref(a, 2);
          qref(a, 2), -qref(a, 1), qref(a, 4), qref(a, 3);
          -qref(a, 1), -qref(a, 2), -qref(a, 3), qref(a, 4);];
    qs = [qmeas(a,1:3) qmeas(a,4)]';
    qe(a,:) = Qt*qs;
    if qe(a,4)<0
        qe(a,:) = -qe(a,:);
    end
end

end