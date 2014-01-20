function d = readCSV(filename)
    data = load(filename);
    
    quat = data(:,2:5);
    ref = data(:,6:9);

    quatstar = data(:,10:13);
    omegastar = data(:,14:16);
    alphastar = data(:,17:19);
    error = data(:,20:23);
    tc3 = data(:,24:26);
    torque = data(:,27:30);
    
    speedcmd = data(:,31:34);
    dc = data(:,35:38);
    time = data(:,39); 
    roll = data(:,40);
    pitch = data(:,41);
    yaw =  data(:,42);
    
    r_dot = data(:,43);
    p_dot = data(:,44);
    y_dot = data(:,45);
    
        
    rpm = data(:,46:49);
    amps = data(:,50:53);
    
    d = struct('time', time,...
        'roll', roll,'pitch', pitch, 'yaw', yaw,...
        'r_dot', r_dot, 'p_dot', p_dot, 'y_dot', y_dot,...
        'quat', quat, 'ref', ref, 'speedcmd', speedcmd,...
        'dc', dc, 'rpm', rpm, 'amps', amps, ...
        'quatstar', quatstar, 'omegastar', omegastar, ...
        'alphastar', alphastar, 'error', error, ...
        'tc3', tc3, 'torque', torque ...
    );