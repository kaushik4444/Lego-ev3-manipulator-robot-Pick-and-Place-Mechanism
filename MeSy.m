% Group_14_MeSy
clear;
% Origin at Base exactly below motor C
% X-axis Outward (+ve)
% Y-axis Right (+ve)
% Z-axis Top (+ve)
stationA = [0,-118,70];
stationB = [118,0,0];
stationC = [0,118,0];

homing();% Reset to position where touch sensors are active
                     % and reset the encoder reading values to zero 
                     
% Sequence start
pick(stationC); % Move to StationC and Close the gripper to pick the object
place(stationA); % Move to StationA and Open the gripper to place the object
pick(stationA);
place(stationB);
pick(stationB);
place(stationC);
% Sequence end

function [] = homing()
    % initiate motor and touch sensor objects
    mylego = legoev3('usb');
    m1 = motor(mylego, 'C'); % motor C
    m2 = motor(mylego, 'B'); % motor B
    t1 = touchSensor(mylego, 1);
    t2 = touchSensor(mylego, 3);
    
    m2.Speed = -30;
    start(m2);
    while(readTouch(t2) == 0)
    end
    stop(m2); % stop when touch sensor t2 is active
    
    m1.Speed = 30;
    start(m1);
    while(readTouch(t1) == 0)
    end
    stop(m1); % stop when touch sensor t1 is active
    
    resetRotation(m1); % reset motor C encoder reading
    resetRotation(m2); % reset motor B encoder reading
end

function [] = move(P)
    % initiate motor and touch sensor objects
    mylego = legoev3('usb');
    m1 = motor(mylego, 'C'); % motor C
    m2 = motor(mylego, 'B'); % motor B
       
    % read position co-ordinates
    X = P(1);
    Y = P(2);
    Z = P(3);
    
    % initiate links lengths
    l1 = 50;
    l2 = 95;
    l3 = 185;
    l4 = 110;
    base = 70;
    
    % Inverse Kinematics to get thetha1 and thetha2
    thetha1 = 90 + atand(Y/X);
    thetha2 = 35 - asind((Z + l4 - base - l1 - l2*sind(45))/l3);
    cur_enc_1 = readRotation(m1); 
    
    % move to desired thetha1
    if(abs(-3*thetha1 - cur_enc_1) > 35) % compare existing and desired thetha1 values
                                                                % for a considerable amount of change in thetha1
        if (-3*thetha1 - cur_enc_1) > 0     % clockwise rotation  
            m1.Speed = 30;
            start(m1);
            while(cur_enc_1 < -3*thetha1) % gear ratio = 3
                cur_enc_1 = readRotation(m1);
            end
            stop(m1); % stop at desired thetha1
        else                                                 % counter clockwise rotation
            m1.Speed = -30;
            start(m1);
            while(cur_enc_1 > -3*thetha1)
                cur_enc_1 = readRotation(m1);
            end
            stop(m1); % stop at desired thetha1   
        end
   end
    
    % move to desired thetha2
    m2.Speed = 2;
    start(m2);
    cur_enc_2 = readRotation(m2);
    while(cur_enc_2 < 5*thetha2) % gear ratio = 5
        cur_enc_2 = readRotation(m2);
    end
    stop(m2); % stop at desired thetha2
end

function [] = pick(P)
    % move to desired station
    move(P);
    
   % initiate motor and touch sensor objects
    mylego = legoev3('usb');
    m3 = motor(mylego, 'A'); % motor C
    m2 = motor(mylego, 'B'); % motor B
    t2 = touchSensor(mylego, 3);
    
    % Pick up the object
    m3.Speed = -25;
    resetRotation(m3);
    start(m3);
    cur_enc_3 = readRotation(m3);
    while(abs(cur_enc_3) < 80) 
       cur_enc_3 = readRotation(m3);
    end
    stop(m3);
    pause(0.5); % wait 0.5 sec
    
   % bring elbow back to top (thetha2 = 0)
    m2.Speed = -35;
    start(m2);
    while(readTouch(t2) == 0)
    end
    stop(m2);
end

function [] = place(P)
    % move to desired station
    move(P);   
    
    % initiate motor and touch sensor objects
    mylego = legoev3('usb');
    m3 = motor(mylego, 'A'); % motor C
    m2 = motor(mylego, 'B'); % motor B
    t2 = touchSensor(mylego, 3);
    
    % Place the object
    m3.Speed = 25;
    resetRotation(m3);
    start(m3);
    cur_enc_3 = readRotation(m3);
    while(abs(cur_enc_3) < 75) 
       cur_enc_3 = readRotation(m3);
    end
    stop(m3);
    pause(0.5); % wait 0.5 sec   
    
   % bring elbow back to top (thetha2 = 0)
    m2.Speed = -35;
    start(m2);
    while(readTouch(t2) == 0)
    end
    stop(m2);
end