function d = readCSV32(filename)
    data = load(filename);
    
    quat = data(:,2:5);
    ref = data(:,6:9);
    speedcmd = data(:,10:13);
    dc = data(:,14:17);
    time = data(:,18); 
    roll = data(:,19);
    pitch = data(:,20);
    yaw =  data(:,21);
    
    r_dot = data(:,22);
    p_dot = data(:,23);
    y_dot = data(:,24);
    
    rpm = data(:,25:28);
    amps = data(:,29:32);
    
    d = struct('time', time,...
        'roll', roll,'pitch', pitch, 'yaw', yaw,...
        'r_dot', r_dot, 'p_dot', p_dot, 'y_dot', y_dot,...
        'quat', quat, 'ref', ref, 'speedcmd', speedcmd,...
        'dc', dc, 'rpm', rpm, 'amps', amps);
