% deconvolution_wsinc_fir

clear all; close all; clc;

degC_DC = 0;
degC_pkpk = 5;
quantStepDegC = 0.1; % 1 LSB (1.1 Vref) = 0.1 degC
LM61_tau = 20; % seconds, convection heat transfer

GAIN = 60;     % choose for mid-frequency amplitude restoration

% ---- Simulate LM61 output from variable frequency breathing --------
bpm5 =  [ones(1,60) -ones(1,60)];
bpm15 = [ones(1,20) -ones(1,20)];
bpm30 = [ones(1,10) -ones(1,10)];
bpm60 = [ones(1,5) -ones(1,5)];
bpm99 = [ones(1,3) -ones(1,3)];

% ---------- Create user breathing pattern ----------
breath = degC_DC+0.5*degC_pkpk*[repmat(bpm5,1,2) repmat(bpm15,1,3)... 
          repmat(bpm30,1,6) repmat(bpm60,1,8) repmat(bpm99,1,8)];
nb = 1:length(breath);

% ---------- LPF breathing to get dieDegC readings ----------
alpha = exp(-0.1/LM61_tau);
beta = 1-alpha; % DC gain = 1
dieDegC = zeros(size(breath)); % declare space
% dieDegCm1 = degC_DC;
dieDegCm1 = 0.0;
for i = nb;
    dieDegC(i) = alpha*dieDegCm1 + beta*breath(i);
    dieDegCm1 = dieDegC(i);
end

% ---------- Quantize per ADC resolution, dither ---------
qDieDegC = quantStepDegC*round(dieDegC/quantStepDegC);

% ---------- Estimate SNR by breathing rate ----------
snr5  = 20*log10(std(dieDegC(1:200))/(0.29*quantStepDegC))
snr15 = 20*log10(std(dieDegC(250:350))/(0.29*quantStepDegC))
snr30 = 20*log10(std(dieDegC(375:475))/(0.29*quantStepDegC))
snr60 = 20*log10(std(dieDegC(500:575))/(0.29*quantStepDegC))
snr99 = 20*log10(std(dieDegC(575:600))/(0.29*quantStepDegC))

% ---------- Display degC and dieDegC ----------
fig1 = figure(1);
set(fig1,'position',[220 100 1200,600]);
subplot(2,1,1); plot(nb,breath,'b');
grid on; xlim([0 600]); ylabel('Breath degC');
subplot(2,1,2); plot(nb,qDieDegC,'r'); grid on; xlim([0 600]);
grid on; xlim([0 600]); ylabel('ADC dieDegC'); xlabel('samples');

%return

% ---------- Windowed Sync LPF ---------- 
Fc = 120/600;
M = 10;
N = 128;
n = 1:N;
eps = 1e-6; % L'Hospital dodge
sinc = GAIN*sin(2*pi*Fc*(n-M/2+eps))./(n-M/2+eps);
sinc(M:end) = 0;

% ---------- Convolve with differencer ----------
kernel = conv([1 -1],sinc);
kernel = kernel(1:N);

wndw = zeros(size(n));
wndw(1:M) = hamming(M);
wkrnl = kernel.*wndw;

fig2 = figure(2);
set(fig2,'position',[220 100 500,600]);
stem(n,sinc,'b'); grid on;
xlim([0 M+5]);
title('LPF Impulse Response');

fig3 = figure(3);
set(fig3,'position',[750 100 500,600]);
stem(n,wkrnl,'b'); grid on;
xlim([0 M+5]);
title('Deconvolution Kernel Impulse Response');

f_sinc = abs(freqz(sinc.*wndw,1,N));
f_krnl = abs(freqz(wkrnl,1,N));
bpm = (n/N)*300; % N points to folding freq

figure(4)
plot(bpm,f_sinc,'r'); 
grid on; hold on;
plot(bpm,f_krnl,'b');
legend('LPF','deconv');
xlabel('BPM');
title('Magnitude Frequency Responses');

dcnvlv_degC = filter(wkrnl,1,qDieDegC);

fig5 = figure(5);
set(fig5,'position',[220 100 1200,600]);
%---------------------------------------
subplot(2,1,1);
plot(nb,qDieDegC,'b');
grid on; hold on;
plot(nb,dcnvlv_degC,'r');
plot(nb,breath,'g');
xlim([0 300]);
% xlim([M 300]); % suppress deconv kernel step response
ylabel('degC');
%---------------------------------------
subplot(2,1,2);
plot(nb,qDieDegC,'b');
grid on; hold on;
plot(nb,dcnvlv_degC,'r');
plot(nb,breath,'g');
xlim([300 600]);
ylabel('degC');
xlabel('seconds');

figure(1); set(fig1,'position',[250 70 1200,600]);
figure(5);


% ---------- Create C style declarations ----------
fprintf('  // FIR Thermal Compensator: LM61 tau = %0.1f seconds\n',LM61_tau);
% fprintf('  // Thermal tau FIR deconvolution kernel\n');
fprintf('  const float h_tcomp[] = {%06f,\n   ',wkrnl(1));
for i = 2:(M-1)
    fprintf('%0.6f, ',wkrnl(i));
    if (mod(i,6) == 0)
        fprintf('\n   ');
    end
end
fprintf('%0.6f};\n',wkrnl(M)');
fprintf('  const int DCV_KRNL_LEN = %d;\n',M);
% fprintf('  const float GAIN = %0.1f;\n;',GAIN);

