%% param define
clc, clear, close all;
method = 1 ; % 0 = downlink, 1 = uplink;
norm_method = 1; %0 onlyphase, 1 amp-phase
numFrame = 10;
numSlot = 20;
numSymbol = 14; % skip uplink slot / 20
qamLevel = 64;
qamrefLevel = 4;
lambda = 0.99; % learning rate
iter = 10;
SNR = 25;
pwrLevel = -15;
eps = 10^((pwrLevel-SNR)/20)*1e-2;
[numAnt,N_FFT,N_SC,Fs,ref_data,h_response,data_response]=load_param(method);
calibWeight = complex(ones(N_SC,numAnt));
invCov = complex(ones(N_SC,numAnt));
binData =[];
fData = [];
rmsCalibEVM =[];
maxCalibEVM =[];
calibBER =[];
rmsDataEVM =[];
maxDataEVM =[];
dataBER =[];
finalData =[];
dbfs_tx =[];
awgn_noise = zeros(N_SC,numAnt);
%% custom loop
for frameID = 1:numFrame
    awgn_noise = randi([-10 10],N_SC,numAnt,'double')*eps;
    H_response = awgn_noise + h_response;
    for slotID = 1:numSlot
        for symID = 1:numSymbol
            fprintf('==========symbol %d - slot %d - frame %d============\n',symID,slotID,frameID);
            if (frameID == 1)
                if ((slotID == 4 || slotID == 14) && symID == 8)
                    fprintf('Generate PILOT data\n');
                    pilot = 1;
                else
                    pilot = 0;
                    fprintf('Generate OFDM data\n');
                end
                
                lambda = 0.9; % learning rate
                iter = 20;
            elseif(frameID == 2)
                if ((slotID == 4 || slotID == 14) && symID == 8)
                    fprintf('Generate PILOT data\n');
                    pilot = 1;
                else
                    pilot = 0;
                    fprintf('Generate OFDM data\n');
                end
                
                lambda = 0.99; % learning rate
                iter = 20;
            else
                if ((slotID == 4 || slotID == 14) && symID == 8)
                    fprintf('Generate PILOT data\n');
                    pilot = 1;
                else
                    pilot = 0;
                    fprintf('Generate OFDM data\n');
                end
                
                lambda = 0.99; % learning rate
                iter = 5;
            end
            [binData,fData,loop_gain,dbfs_tx] = symbol_gen(N_SC,qamLevel,pilot,qamrefLevel,ref_data);
            [channelTime_data] = channel_creater(numAnt,pilot,calibWeight, ...
                            H_response,fData,N_SC,N_FFT,symID);
            if(pilot == 1)
                fprintf('Modulation %dQAM, dBFs TX: %.4f\n',qamrefLevel,dbfs_tx);
                if (qamrefLevel == 0)
                    [delay,h_matrix] = channel_estimation(channelTime_data,ref_data,...
                    N_FFT,numAnt,N_SC,1);
                    for ant=1:numAnt
                        fprintf('Delay ant%d: %f\n',ant,delay(i));
                    end
                end
                fprintf('Calib weight calculation\n');
                [calibfData,calibWeight,invCov,rmsEVM,maxEVM,BER] = calib_function(channelTime_data,...
                fData,binData,calibWeight,invCov,N_FFT,N_SC,numAnt,lambda,iter,qamrefLevel,dbfs_tx);
                rmsCalibEVM = [rmsCalibEVM,rmsEVM];
                maxCalibEVM = [maxCalibEVM,maxEVM];
                calibBER = [calibBER,BER];
                fprintf('Mean EVM: %f\n',mean(rmsEVM));
                fprintf('Max EVM: %f\n',max(maxEVM));
                fprintf('Mean BER(dB): %.4e\n',mean(BER));
            else
                fprintf('Modulation %dQAM, dBFs TX: %.4f\n',qamLevel,dbfs_tx);
                fprintf('Decode data\n');
                [finalData,rmsEVM,maxEVM,BER] = decode_function(channelTime_data,fData,binData,...
                calibWeight,N_FFT,N_SC,numAnt,qamLevel,dbfs_tx);
                rmsDataEVM = [rmsDataEVM,rmsEVM];
                maxDataEVM = [maxDataEVM,maxEVM];
                dataBER = [dataBER,BER];
                fprintf('Mean EVM: %f\n',mean(rmsEVM));
                fprintf('Max EVM: %f\n',max(maxEVM));
                fprintf('Mean BER(dB): %.4e\n',mean(BER));
            end
        end
    end
end
%% result
firstRMS_EVM = rmsDataEVM(:,1); firstMAX_EVM = maxDataEVM(:,1); firstBER = dataBER(:,1);
finalRMS_EVM = rmsDataEVM(:,end); finalMAX_EVM = maxDataEVM(:,end); finalBER = dataBER(:,end);

% ==========================
%  Helper function: add labels
% ==========================
addLabel = @(h) arrayfun(@(x) ...
    text(x.XEndPoints, x.YEndPoints, ...
    string(round(x.YData,4)), ...
    'HorizontalAlignment','center', ...
    'VerticalAlignment','bottom', ...
    'FontSize',10,'FontWeight','bold'), h);

% ==========================
%      RMS EVM
% ==========================
figure;
dataRMS = [firstRMS_EVM finalRMS_EVM];
h = bar(dataRMS, 'LineWidth',1.2);
title('RMS EVM: First vs Final','FontSize',14);
xlabel('Index','FontSize',12);
ylabel('RMS EVM(%)','FontSize',12);
legend('First','Final','FontSize',12);
grid on;
addLabel(h);
% ==========================
%      MAX EVM
% ==========================
figure;
dataMAX = [firstMAX_EVM finalMAX_EVM];
h = bar(dataMAX, 'LineWidth',1.2);
title('MAX EVM: First vs Final','FontSize',14);
xlabel('Index','FontSize',12);
ylabel('MAX EVM(%)','FontSize',12);
legend('First','Final','FontSize',12);
grid on;
addLabel(h);

% ==========================
%      BER (log-scale + dB)
% ==========================
firstBER_dB = firstBER;
finalBER_dB = finalBER;
figure;
dataBER_dB = [firstBER_dB finalBER_dB];
h = bar(dataBER_dB,'LineWidth',1.2);
title('BER (dB): First vs Final','FontSize',14);
xlabel('Index','FontSize',12);
ylabel('BER(dB)','FontSize',12);
legend('First','Final','FontSize',12);
grid on;
addLabel(h);

% show 8TX constellation
constDiagram = comm.ConstellationDiagram('ReferenceConstellation',fData,'ShowLegend',true, ...
    'XLimits',[-1.5 1.5],'YLimits',[-1.5 1.5],'ShowReferenceConstellation',true,...
    'ChannelNames',{'QAM , RX1',...
    'QAM , RX2', 'QAM , RX3', 'QAM , RX4', 'QAM , RX5', 'QAM , RX6',...
    'QAM , RX7', 'QAM , RX8',...
    });
constDiagram(finalData(:,1:8,3));

%% phase-amplitude comparator
if norm_method == 1
    for i =1:numAnt
        NewcalibWeight(:,i)= calibWeight(:,i)/max(abs(calibWeight(:,i)));
    end
else
    NewcalibWeight = exp(1i*angle(calibWeight));
end
for i=1:numAnt
    finalData1(:,i) = finalData(:,i,2)./calibWeight(:,i);
    finalData1(:,i) = finalData1(:,i).* NewcalibWeight(:,i);
%     finalData1(:,i) = finalData1(:,i) - mean(finalData1(:,i));
    dbfs_data = 20*log10(rms(finalData1(:,i)));
    loop_gain = dbfs_tx - dbfs_data;
    finalData1(:,i) = finalData1(:,i)*sqrt(10^(loop_gain/10));
end
% show 8TX constellation with normalized weight
constDiagram2 = comm.ConstellationDiagram('ReferenceConstellation',fData,'ShowLegend',true, ...
    'XLimits',[-1.5 1.5],'YLimits',[-1.5 1.5],'ShowReferenceConstellation',true,...
    'ChannelNames',{'cross check, TX1',...
    'cross check, TX2', 'cross check, TX3', 'cross check, TX4', 'cross check, TX5', 'cross check, TX6',...
    'cross check, TX7', 'cross check, TX8',...
    'Name','Cross check window'});
constDiagram2(finalData1(:,1:8));
phase_diff = zeros(size(finalData1));
amp_diff   = zeros(size(finalData1));
for i = 1:numAnt
    H_comp = finalData1(:,i)./fData;
    phase_diff(:, i) = rad2deg(angle(H_comp));
    amp_diff(:, i) = abs(H_comp);
end
amp_diff_db = 20*log10(amp_diff);
% ----- Heatmap amplitude -----
figure;
imagesc(amp_diff_db);
set(gca,'YDir','normal');
colorbar;
title('Amplitude Deviation vs fData (dB)');
xlabel('RX Channel Index');
ylabel('Subcarrier Index');
% ----- Heatmap phase -----
figure;
imagesc(phase_diff);
set(gca,'YDir','normal');
colorbar;
title('Phase Deviation vs fData (degrees)');
xlabel('RX Channel Index');
ylabel('Subcarrier Index');

%% average phase and power
for i = 1:numAnt
    avg_phaseRE(i) = mean(phase_diff(:, i));
    avg_powerRE(i) = mean(amp_diff_db(:, i));
end
figure,
plot(avg_phaseRE,'r-o');
title('Avgerage phase offset after calib');
xlabel('TX Channel Index');
ylabel('deg');
figure,
plot(avg_powerRE,'k-o');
title('Avgerage amplitude offset after calib');
xlabel('TX Channel Index');
ylabel('dB');

%% helper
function [channelTime_data] = channel_creater(numAnt,pilot,calibWeight, ...
                            h_response,fData,N_SC,N_FFT,symCount)
    timeData = [];
    for i = 1:numAnt
        tData = ifft_Nocomp(symCount,N_SC,N_FFT,fData);
        tdata_out = channel_model(h_response(:,i),tData,N_FFT,N_SC);
        timeData = [timeData,tdata_out];
    end
    channelTime_data = timeData;
end

function [nfData,calibWeight,invCov,rmsEVM,maxEVM,BER] = calib_function(channelTime_data,refData,encodeBits,...
          calibWeight,invCov,N_FFT,N_SC,numAnt,lambda,iter,qamrefLevel,dbfs_tx)
      
     evm = comm.EVM('MaximumEVMOutputPort',true, ...
    'ReferenceSignalSource','Estimated from reference constellation', ...
    'ReferenceConstellation',refData);

    for i = 1:numAnt
       fData0 = fft_Nocomp(channelTime_data(:,i),N_FFT,N_SC);
       fData1 = fData0 - mean(fData0); %remove DC leakage
       dbfs_rx = 20*log10(rms(fData1));
%        fprintf('RX%d power before scale: %.4f dBFs\n',i,dbfs_rx);
       fData = fData1 * sqrt((10^(dbfs_tx-dbfs_rx)/10));
       for it = 1:iter
            [~,calibWeight(:,i),invCov(:,i)] = rls_engine(fData, ...
            refData,calibWeight(:,i),invCov(:,i),lambda);
       end
       fData2 = fData.*calibWeight(:,i);
       nfData(:,i,1) = fData0;
       nfData(:,i,2) = fData1;
       nfData(:,i,3) = fData;
       nfData(:,i,4) = fData2;
       dbfs_rx_cal = 20*log10(std(fData2));
%        fprintf('RX%d power after calib: %.4f dBFs\n',i,dbfs_rx_cal);
      [rmsEVM(i,1),maxEVM(i,1)] = evm(nfData(:,i));
      if (qamrefLevel == 0)
         BER(:,i) = -100;
      else
         decodeBits = qamdemod(nfData(:,i),qamrefLevel,'OutputType','bit','UnitAveragePower',true);
         bit_ber = biterr(encodeBits,decodeBits);
         BER(i,1) = 10*log10(bit_ber/length(encodeBits)+eps);
      end
    end
end

function [nfData,rmsEVM,maxEVM,BER] = decode_function(channelTime_data,refData,encodeBits,...
    calibWeight,N_FFT,N_SC,numAnt,qamLevel,dbfs_tx)
    evm = comm.EVM('MaximumEVMOutputPort',true, ...
    'ReferenceSignalSource','Estimated from reference constellation', ...
    'ReferenceConstellation',refData);

    for i = 1:numAnt
       fData0 = fft_Nocomp(channelTime_data(:,i),N_FFT,N_SC);
%        fData1 = fData0 - mean(fData0); %remove DC leakage
       fData1 = fData0.*calibWeight(:,i);
       dbfs_rx = 20*log10(rms(fData1));
%        fprintf('RX%d power before scale: %.4f dBFs\n',i,dbfs_rx);
       fData = fData1 * sqrt(10^((dbfs_tx-dbfs_rx)/10));
       dbfs_rx_cal = 20*log10(std(fData));
%        fprintf('RX%d power after scale: %.4f dBFs\n',i,dbfs_rx_cal);
       nfData(:,i,1) = fData0;
       nfData(:,i,2) = fData1;
       nfData(:,i,3) = fData;
       [rmsEVM(i,1),maxEVM(i,1)] = evm(fData);
       decodeBits = qamdemod(fData,qamLevel,'OutputType','bit','UnitAveragePower',true);
       bit_ber = biterr(encodeBits,decodeBits);
       BER(i,1) = 10*log10(bit_ber/length(encodeBits)+eps);
    end
end
