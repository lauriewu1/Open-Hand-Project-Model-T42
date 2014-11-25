%%
sensor_data=[1 2 4 8 16 32 64 128];
u2i=actxserver('USB2IIC.USB2IICcom_EXE');
%methodsview(u2i);
u2i.InitUSB
u2i_bridge_name=char(u2i.GetBridgeList())
%u2i.GetPowerSpeed(u2i_bridge_name)
u2i.SetPower(u2i_bridge_name,1)    % 1 - means 5.0v, 2 - means 3.3V
u2i.SetSpeed(u2i_bridge_name, 36)  % 36 - 400k  32 - 100k
u2i.GetDeviceList(u2i_bridge_name)

% init I2C IO  (ID:32)
feature('COM_SafeArraySingleDim', 1)
data=uint8([0 0]')  % all pins are outputs
u2i.SendIICdata(u2i_bridge_name, 32, data)
data=uint8([10 0]') % set all pins down
u2i.SendIICdata(u2i_bridge_name, 32, data)


%%
t=0;
p=0;
pc=0;

sensors=3;

% read calibration coefficients
for sensor=1:sensors
        % set second pressure sensor
        sensor_select=sensor;
        data=uint8([10 sensor_data(sensor_select)]'); % choose a sensor
        u2i.SendIICdata(u2i_bridge_name, 32, data);      % init Pressure Sensor  (ID:96)

        data=uint8([0 0]'); % set all pins down
        u2i.SendIICdata(u2i_bridge_name, 96, data);
        [a calib] = u2i.ReadIICdata(u2i_bridge_name, 96, 16);

        % Placing coefficients into 16-bit Variables
        % a0
        sia0 = bitshift(uint16(calib(5)),8);
        sia0 = sia0 + uint16(calib(6));
        sia0 = typecast(uint16(sia0), 'int16');
        cda0(sensor)=sia0;
        % b1
        sib1 = bitshift(uint16(calib(7)),8);
        sib1 = sib1 + uint16(calib(8));
        sib1 = typecast(uint16(sib1), 'int16');
        cdb1(sensor)=sib1;
        % b2
        sib2 = bitshift(uint16(calib(9)),8);
        sib2 = sib2 + uint16(calib(10));
        sib2 = typecast(uint16(sib2), 'int16');
        cdb2(sensor)=sib2;
        % c12
        sic12 = bitshift(uint16(calib(11)),8);
        sic12 = sic12 + uint16(calib(11));
        sic12 = typecast(uint16(sic12), 'int16');
        cdc12(sensor)=sic12;
        % c11
        sic11 = bitshift(uint16(calib(13)),8);
        sic11 = sic11 + uint16(calib(14));
        sic11 = typecast(uint16(sic11), 'int16');
        cdc11(sensor)=sic11;
        % c22
        sic22 = bitshift(uint16(calib(15)),8);
        sic22 = sic22 + uint16(calib(16));
        sic22 = typecast(uint16(sic22), 'int16');
        cdc22(sensor)=sic22;

end

tt = [];
pc_history=[0, 0, 0, 0,0,0,0,0,0];
pc_history_i=1;
pc_h=10;

p_sign=[1,1,1]; % if values are above 250 then 1 otherwise 0 (used for overflow calculations)
tic   % start timer

for i=1:1000000000
 for ii=1:1
    for sensor=1:sensors
        sensor_select=sensor;
        data=uint8([10 sensor_data(sensor_select)]'); % choose a sensor
        u2i.SendIICdata(u2i_bridge_name, 32, data);        
        data=uint8([18 1]');                            % activate reading
        u2i.SendIICdata(u2i_bridge_name, 96, data);     % send data out  (ID:96)
    end
    % read the data
    for sensor=1:sensors

        sensor_select=sensor;
        data=uint8([10 sensor_data(sensor_select)]'); % choose a sensor
        u2i.SendIICdata(u2i_bridge_name, 32, data);     % activate the pin  (ID:32)

        data=uint8([0 0]'); % set all pins down
        u2i.SendIICdata(u2i_bridge_name, 96, data);
        [a b] = u2i.ReadIICdata(u2i_bridge_name, 96, 4);  % ask for 4 bits of data

        tt(i) = toc; %take time
        
        p(i,sensor) = (bitshift(uint16(b(1)),8)); % bit shift to the left 8 bits
            p(i,sensor) = p(i,sensor) + uint16(b(2)); % add the second part
        if (~p_sign(sensor))
            p(i,sensor) = typecast(uint16(p(i,sensor)), 'int16'); % convert it to INT
        end
        p(i,sensor)=  bitshift(p(i,sensor),-6);     % bit shift it back to remove the zeroes
        p_sign(sensor)=p(i,sensor)>200;
        
        t(i,sensor) = (bitshift(uint16(b(3)),8));
        t(i,sensor) = t(i,sensor) + uint16(b(4));
        t(i,sensor) = bitshift(t(i,sensor),-6);

        Padc=p(i,sensor);
        Tadc=t(i,sensor);

        % a0= 0x41DF = 2107.875
        a0 = double(double(cda0(sensor))/double(2^3));

        % b1 = 0xB028 = -2.49512
        b1=double(double(cdb1(sensor))/double(2^13));

        % b2 - 0xBEAD = -1.02069
        b2 = double(double(cdb2(sensor))/double(2^14));

        % c12 - 0x38CC = 0.00086665
        c12 = double(double(cdc12(sensor))/double(2^24));
      
        c12x2 = c12 * Tadc;
        a1 = b1 + c12x2;
        a1x1 = a1 * Padc;
        y1 = a0 + a1x1;
        a2x2 = b2 * Tadc;

        % should be 733.29270
        Pcomp = y1 + a2x2;


                    pc(i,sensor)=Pcomp;
            end    
         end
               plot(tt,pc(:,3));

    pause(0.001);
    
    % tt is time
    % pc - calibrated pressure
    % t - temperature
    % p - uncalibrated pressure

end

%% Plots
figure(100)
plot(tt,pc,'b')
xlabel('Time [s]')
ylabel('Pressure (Calibrated)')
title('Title')
print -f100 -dpng name
saveas(gcf,'name','fig')

figure(200)
plot(tt,p,'g')
xlabel('Time [s]')
ylabel('Pressure (Uncalibrated)')
title('Title')
print -f200 -dpng name
saveas(gcf,'name','fig')

figure(300)
plot(tt,t,'r')
xlabel('Time [s]')
ylabel('Temperature')
title('Title')
print -f300 -dpng name
saveas(gcf,'name','fig')

save('name')

%% Release USB2IIC

feature('COM_SafeArraySingleDim', 0)
u2i.UnInitUSB
u2i.release()

%methods(u2i)
lsb=40
msb=176

s = bitshift(uint16(lsb),8);
s = s + uint16(msb);
s = typecast(uint16(s), 'int16')



