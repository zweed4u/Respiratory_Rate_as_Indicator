% Lab_6_filter_ba_starter

clear all; close all; clc

Ts = 0.1;   % seconds sample period
fs = 1/Ts;  % Hz sampling frequency

% NOTE: The Matlab cheby1 cut frequency spec is in terms
% of normalized frequency from 0 to 1.0 over the PASSBAND,
% that is, to the folding frequency rather than the 
% sampling frequency.

w = linspace(0,pi,4096); % freqz() requires 0..pi range
fbpm = (w/pi/2)*fs*60;

% %---------------------------------------------------
% % Chebyshev filter creation example
% 
% cutBPM = 40;
% cutWp = 2*cutBPM/60)/fs;
% bandWp = 2*[10,40]/60/fs; % band edges for pass & stop filters
% 
% nPole = 8; % set from 2 to 10, inclusive
% R = 1.0;
% [b,a] = cheby1(nPole,R,cutWp);            % low pass
% [b,a] = cheby1(nPole,R,cutWp,'high');     % hi pass
% [b,a] = cheby1(nPole,R,bandWp);           % band pass
% [b,a] = cheby1(nPole,R,bandWp,'stop');    % band stop

%--------------------------------------------------
% Evaluation instance: LPF
nPole = 6;
R = 0.5;
cutBPM = 11;
cutWp = 2*(cutBPM/60)/fs;
[b,a] = cheby1(nPole,R,cutWp);
xrng = [0 100];
comment_txt = sprintf...
('LPF, %d poles, R = %0.1f, %d BPM\n',nPole,R,cutBPM);

%--------------------------------------------------
% Evaluation instance: HPF
% nPole = 14;
% R = 0.1;
% cutBPM = 40;
% cutWp = 2*(cutBPM/60)/fs;
% [b,a] = cheby1(nPole,R,cutWp,'high');
% xrng = [0 100];
% comment_txt = sprintf...
% ('HPF, %d poles, R = %0.1f, %d BPM\n',nPole,R,cutBPM);


%--------------------------------------------------
% Evaluation instance: BPF


%---------------------------------------------------
% Display pole-zero plot for nPole pole filter
fig1 = figure(1);
zplane(b,a);
title('Filter Pole-Zero Plot');

%---------------------------------------------------
% Display impulse responses
trimRatio = 0.01;
[hir,smp1] = impz(b,a);
maxN = find((abs(hir)/max(hir)) > trimRatio,1,'last');

fig2 = figure(2);
if (maxN < 80)
    stem(smp1,hir,'b','linewidth',2);
else
    plot(smp1,hir,'b','linewidth',2);
end
grid on;
xlim([0 maxN]);
xlabel('sample');
title('Chebyshev Impulse Response');

%---------------------------------------------------
% Display frequency response

fprintf('number of poles = %s\n',num2str(nPole));
format long;
den_coeff_a = a'
num_coeff_b = b'
format short;

num_roots = roots(b);
den_roots = roots(a);
pole_wn = abs(den_roots) % display pole magnitudes

%---------------------------------------------------
% Display cascade frequency response
fig4 = figure(4);
ht = freqz(b,a,w);
plot(fbpm,abs(ht),'b','linewidth',2)
grid on;
xlim(xrng); ylim([0 1.1]);
xlabel('Breaths per Minute');
title(['IIR Frequency Response: ',num2str(nPole),' poles']);

%---------------------------------------------------
% Generate C code parameters for filter
% Copy and paste into Arduino function

fprintf(['// ' comment_txt]);
fprintf('const float a[] = {');
for n = 1:(length(a)-1);
    fprintf('%0.10f, ',a(n));
end
fprintf('%0.10f};\n',a(end));

G = max(b);
fprintf('const float b[] = {');
for n = 1:(length(b)-1);
    fprintf('%0.10f, ',b(n)/G);
end
fprintf('%0.10f};\n',b(end)/G);
fprintf('const float GAIN = %0.15f;\n',G);

