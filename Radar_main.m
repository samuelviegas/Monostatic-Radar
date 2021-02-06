%% Monostatic Radar %%
% Written by Samuel Viegas %
% User Input
[wave_type,targets_init]=UserInputPrompt();
%% Design Specifications/Variables/Constants
c=physconst('LightSpeed');          % Light speed
fs=200e6;                           % Sampling frequency
fc=80e9;                            % Carrier frequency
lambda=c/fc;                        % Wavelength
bw=fs/2;                            % Radar bandwidth
prf=fs/2000;                        % Pulse repetition frequency
pri=1/prf;                          % Pulse repetition interval
pd=0.9;                             % Probability of detection
pfa=1e-6;                           % Probability of false alarm
max_range=c/(2*prf);                % Maximum range
duty_cycle=0.02;                    % Waveform dutycycle
range_res=(c*(duty_cycle/prf))/2;   % Range resolution
n_pulses=300;                       % Loop pulses
txgain=20;                          % Transmitter antenna gain
rxgain=40;                          % Receiver antenna gain
noisefig=1;                         % Receiver noise figure

%% Waveform Generator
if wave_type==2
    % LinearFM waveform                                
    waveform=phased.LinearFMWaveform('SampleRate',fs,'PRF',prf,'OutputFormat',...
                                     'Pulses','NumPulses',1,'SweepBandwidth',fs/2,...
                                     'DurationSpecification','Duty cycle','DutyCycle',duty_cycle); 
elseif wave_type==1
    % Rectangular waveform
    waveform=phased.RectangularWaveform('SampleRate',fs,'PRF',prf,'OutputFormat',...
                                        'Pulses','NumPulses',1,'DurationSpecification',...
                                        'Duty cycle','DutyCycle',duty_cycle);
else
    load saved_target_init;
end    
                             
%% Transmitter
snr_min=albersheim(pd,pfa); % The required SNR can be derived using the Albersheim's equation
% The relationship between Pd, Pfa and SNR can be represented by a ROC curve
%rocsnr([2 4 6 8 10 12 13 14],'SignalType','NonfluctuatingNoncoherent')
% The peak power at the transmitter is calculated using the radar equation
peakpower=radareqpow(lambda,max_range,snr_min,pri,'RCS',1,'Gain',txgain);
transmitter=phased.Transmitter('PeakPower',peakpower,...
                               'Gain',txgain,...
                               'InUseOutputPort',true);

%% Receiver
receiver=phased.ReceiverPreamp('SampleRate',fs,...
                               'Gain',rxgain,...
                               'NoiseFigure',noisefig);

%% Tx/Rx Array
% Cosine antenna
antenna=phased.CosineAntennaElement;
% Assuming that the antenna is stationary
radarpos=[0;0;0];
radarspd=[0;0;0];
radarmotion=phased.Platform('InitialPosition',radarpos,'Velocity',radarspd);
% In this radar system the radiator and the collector are equal
radiator=phased.Radiator('Sensor',antenna,...
                         'PropagationSpeed',c,...
                         'OperatingFrequency',fc);
collector=phased.Collector('Sensor',antenna,...
                           'PropagationSpeed',c,...
                           'OperatingFrequency',fc);
                       
%% Environment, Targets & Interference
% Environment: FreeSpace channel, two way propagation model 
channel=phased.FreeSpace('SampleRate',fs,...    
                         'PropagationSpeed',c,...
                         'OperatingFrequency',fc,...
                         'TwoWayPropagation',true);
% Targets positions and velocities
Numtgts=4;
targets=[1,1,1,1]; % 4 initial active targets
tgtpos=zeros(3,Numtgts);
tgtspd=zeros(3,Numtgts);
tgtpos(1,:)=[targets_init(1,1),targets_init(1,2),targets_init(1,3),targets_init(1,4)]; % targets initial distances
tgtspd(1,:)=[targets_init(2,1),targets_init(2,2),targets_init(2,3),targets_init(2,4)]*(-1); % targets initial velocities
tgtrcs=[1 1 1 1]; % Targets radar cross section
% Seed for noise generation at the receiver
receiver.SeedSource='Property';
receiver.Seed=2009;
specanalyzer=dsp.SpectrumAnalyzer('SampleRate',fs,...
                                  'PlotAsTwoSidedSpectrum',true,...
                                  'Title','Spectrum for received and dechirped signal',...
                                  'YLimits',[-200 -50],...
                                  'ShowLegend',true);                              

%% Target & Radar Motion/Detection loop
for j=0:1:29
    if targets(1)~=0
        tgtpos(1,1)=targets_init(1,1)+(targets_init(2,1)*j); % Updates target #1 distance to the radar every second
         % Decision structure to check if the target #1 is still in the observation window
        if tgtpos(1,1)<=0 || tgtpos(1,1)>=max_range
            tgtpos(:,1)=[];
            tgtspd(:,1)=[];
            tgtrcs(1)=[];
            targets(1)=0;
        end  
    end
    if targets(2)~=0
        tgtpos(1,2)=targets_init(1,2)+(targets_init(2,2)*j); % Updates target #2 distance to the radar every second
         % Decision structure to check if the target #2 is still in the observation window
        if tgtpos(1,2)<=0 || tgtpos(1,2)>=max_range
            tgtpos(:,2)=[];
            tgtspd(:,2)=[];
            tgtrcs(2)=[];
            targets(2)=0;
        end
    end
    if targets(3)~=0
        tgtpos(1,3)=targets_init(1,3)+(targets_init(2,3)*j); % Updates target #3 distance to the radar every second
         % Decision structure to check if the target #3 is still in the observation window
        if tgtpos(1,3)<=0 || tgtpos(1,3)>=max_range
            tgtpos(:,3)=[];
            tgtspd(:,3)=[];
            tgtrcs(3)=[];
            targets(3)=0;
        end
    end
    if targets(4)~=0
        tgtpos(1,4)=targets_init(1,4)+(targets_init(2,4)*j); % Updates target #4 distance to the radar every second
         % Decision structure to check if the target #4 is still in the observation window
        if tgtpos(1,4)<=0 || tgtpos(1,4)>=max_range
            tgtpos(:,4)=[];
            tgtspd(:,4)=[];
            tgtrcs(4)=[];
            targets(4)=0;
        end
    end 
    tgtmotion=phased.Platform('InitialPosition',tgtpos,'Velocity',tgtspd);
    target=phased.RadarTarget('PropagationSpeed',c,'OperatingFrequency',fc,'MeanRCS',tgtrcs);
    collector = phased.Collector('Sensor',antenna,'PropagationSpeed',c,'OperatingFrequency',fc);
    % Pre-allocate array for improved processing speed
    rxpulses=zeros(length(waveform()),n_pulses);
    for n=1:n_pulses
        [sensorpos,sensorspd]=radarmotion(pri);
        [tgtpos,tgtspd]=tgtmotion(pri);
        [~,tgtang]=rangeangle(tgtpos,sensorpos);
        sig=waveform();
        [txsig,txstatus]=transmitter(sig);
        % Updates the functions only when a target gets out of range
        if sum(targets)<Numtgts
            Numtgts=sum(targets);
            release(radiator);
            release(channel);
            release(target);
        end
        txsig=radiator(txsig,tgtang);         
        txsig=channel(txsig,sensorpos,tgtpos,sensorspd,tgtspd);   
        tgtsig=target(txsig);  
        % Dechirp the received radar return
        rxcol=collector(tgtsig,tgtang);
        rxsig=receiver(rxcol);  
        rxpulses(:,n)=rxsig;
        dechirpsig=dechirp(rxsig,sig);
        specanalyzer([rxsig dechirpsig]);
        clearvars txsig tgtsig rxcol rxsig;
    end
    %% Signal Processing
    rangedopresp=phased.RangeDopplerResponse('SampleRate',fs,'PropagationSpeed',c,...
                                             'DopplerFFTLengthSource','Property',...
                                             'DopplerFFTLength',1024,'DopplerOutput','Speed',...
                                             'OperatingFrequency',fc);
    % Matched Filter
    matchingcoeff=getMatchedFilter(waveform);
    % Range doppler response
    [rngdopresp,rnggrid,dopgrid]=rangedopresp(rxpulses,matchingcoeff);
    figure(1);
    % Plot range doppler response
    refreshdata
    imagesc(dopgrid,rnggrid,mag2db(abs(rngdopresp)));
    xlabel('Speed (m/s)');
    ylabel('Range (m)');
    title('Range-Doppler Map');
    axis xy
    colormap(parula)
    cbar=colorbar;
    cbar.Label.String='Power (dB)';
    caxis([-150 10]);
    % Noise at the receiver
    mfgain=matchingcoeff'*matchingcoeff; % Matched filter gain
    dopplergain=n_pulses; % Doppler gain
    noisebw=fs/2; % In a loaded system, the noise bandwidth is half of the sample rate
    noisepower=noisepow(noisebw,receiver.NoiseFigure,receiver.ReferenceTemperature);
    noisepowerprc=mfgain*dopplergain*noisepower;
    noise=noisepowerprc*ones(size(rngdopresp));
    % Range estimation
    rangeestimator=phased.RangeEstimator('NumEstimatesSource','Auto',...
                                         'VarianceOutputPort',true,...
                                         'NoisePowerSource','Input port',...
                                         'RMSResolution',range_res);
    % Doppler estimation
    dopestimator=phased.DopplerEstimator('VarianceOutputPort',true,...
                                         'NoisePowerSource','Input port',...
                                         'NumPulses',n_pulses);
    % Calculating range and speed estimations
    dmatrix=NaN(2,Numtgts);
    tgtrng=rangeangle(tgtpos,radarpos);
    targetspd=radialspeed(tgtpos,tgtspd,radarpos,radarspd);
    for m=1:numel(tgtrng)
        [~,i]=min(abs(rnggrid-tgtrng(m)));
        dmatrix(1,m)=i;
        [~,i]=min(abs(dopgrid-targetspd(m)));
        dmatrix(2,m)=i;
    end
    ind=sub2ind(size(noise),dmatrix(1,:),dmatrix(2,:));
    % Range and doppler estimator
    [rngest,rngvar]=rangeestimator(rngdopresp,rnggrid,dmatrix,noise(ind));
    [spdest,spdvar]=dopestimator(rngdopresp,dopgrid,dmatrix,noise(ind));
    % Prints the targets information to command window
    if targets(1)~=0
        fprintf('Target 1:\tEstimated_distance= %.2f m\tEstimated_speed= %.2f m/s\n',rngest(1,1),spdest(1,1));
    else
        fprintf('Target 1 is out of range!\n');
    end
    if targets(2)~=0
        fprintf('Target 2:\tEstimated_distance= %.2f m\tEstimated_speed= %.2f m/s\n',rngest(2,1),spdest(2,1));
    else
        fprintf('Target 2 is out of range!\n');
    end
    if targets(3)~=0
        fprintf('Target 3:\tEstimated_distance= %.2f m\tEstimated_speed= %.2f m/s\n',rngest(3,1),spdest(3,1));
    else
        fprintf('Target 3 is out of range!\n');
    end    
    if targets(4)~=0
        fprintf('Target 4:\tEstimated_distance= %.2f m\tEstimated_speed= %.2f m/s\n',rngest(4,1),spdest(4,1));
    else
        fprintf('Target 4 is out of range!\n');
    end    
    fprintf('\n');
end