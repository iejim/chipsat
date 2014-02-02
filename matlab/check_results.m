clear all;close all;clc;

load('motorTest/15test1.csv');
test = X15test1;


t = test(:,1);%.*1e-3; % time vector
 
%% Plot all four motors time vs. speed & set cmd
figure()

subplot(4,1,1)  % MOTOR 0
speed_0 = test(:,2).*(5908.4/4)*(2*pi/60);
current_0 = test(:,3).*(9/3.996);
cmd = test(:,4);
for i=1:length(cmd)
    if (cmd(i)>10 || cmd(i)<-10)
        setcmd_0(i,1) = -cmd(i)*(5000/80)*(2*pi/60);
    else 
        setcmd_0(i,1) = 0;
    end
end
plot(t,setcmd_0,'r',t,speed_0,'b'); 
grid on;
xlabel('time (ms)');
ylabel('speed (rad/s)');
legend('setcmd','measured');


subplot(4,1,2)  % MOTOR 1
speed_1 = test(:,5).*(5908.4/4)*(2*pi/60);
current_1 = test(:,6).*(9/3.996);
cmd = test(:,7);
for i=1:length(cmd)
    if (cmd(i)>10 || cmd(i)<-10)
        setcmd_1(i,1) = -cmd(i)*(5000/80)*(2*pi/60);
    else 
        setcmd_1(i,1) = 0;
    end
end
plot(t,setcmd_1,'r',t,speed_1,'b'); 
grid on;
xlabel('time (ms)');
ylabel('speed (rad/s)');
legend('setcmd','measured');


subplot(4,1,3)  % MOTOR 2
speed_2 = test(:,8).*(5908.4/4)*(2*pi/60);
current_2 = test(:,9).*(9/3.996);
cmd = test(:,10);
for i=1:length(cmd)
    if (cmd(i)>10 || cmd(i)<-10)
        setcmd_2(i,1) = -cmd(i)*(5000/80)*(2*pi/60);
    else 
        setcmd_2(i,1) = 0;
    end
end
plot(t,setcmd_2,'r',t,speed_2,'b'); 
grid on;
xlabel('time (ms)');
ylabel('speed (rad/s)');
legend('setcmd','measured');


subplot(4,1,4)  % MOTOR 3
speed_3 = test(:,11).*(5908.4/4)*(2*pi/60);
current_3 = test(:,12).*(9/3.996);
cmd = test(:,13);
for i=1:length(cmd)
    if (cmd(i)>10 || cmd(i)<-10)
        setcmd_3(i,1) = -cmd(i)*(5000/80)*(2*pi/60);
    else 
        setcmd_3(i,1) = 0;
    end
end
plot(t,setcmd_3,'r',t,speed_3,'b'); 
grid on;
xlabel('time (ms)');
ylabel('speed (rad/s)');
legend('setcmd','measured');