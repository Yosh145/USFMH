%% Data read stage
F = 48000;
[X, fs] = audioread('wave4.wav');
Xa = awgn(X, 0.5);
AB = resample(Xa, F, fs);
AC = abs(fft(AB));
L = length(AB);
f = (0:L-1)*(F/L);
plot(f, AC);
sound(AB, F);
pause;

%%
n = 1:1:34;
% Ff(n) = [1 11 15 20 26 35 46 60 80 105 139 185 245 325 431 571 756 1002 1327 1756 2325 3078 4075 5396 7145 9458 12519 16572 18440 20000];
% Ff2(n) = [10 30 39 52 68 90 118 155 205 270 357 471 621 819 1080 1425 1880 2480 2900 3270 4310 5690 6100 7500 8500 9900 13070 15000 17260 20000];
% Ff3(n) = [30 37 46 56 69 85 105 130 160 197 243 300 370 456 562 693 854 1052 1296 1596 1967 2424 2986 3680 4534 5586 6883 8479 9000 10445 15000 15000 20000 20000];
Ff3(n) = [1 30 69 85 105 130 160 197 243 300 370 456 562 693 854 1052 1296 1596 1967 2424 2986 3680 4534 5586 6883 8479 9000 10445 11273 15000 15000 20000 20000 25000];
for k = 1:33
   Ffinal(k) = ((Ff3(k + 1) - Ff3(k))/2) + Ff3(k);
end
N = 5000;
fid = fopen('matrix.txt', 'w');
Y1 = 0;
hold on; 
for i = 1:32
    hold on;
    % BP1 = fir1(N, [Ffinal(i) Ffinal(i+1)]/(F/2), 'bandpass');
    BP1 = fir1(N, [Ffinal(i) Ffinal(i+1)]/(F/2), 'bandpass', kaiser(N+1, 15));
    [H, W] = freqz(BP1, 1, 1024, F);
    title(['fmax = ' num2str(Ffinal(i+1))]);
    subplot(4,1,1);
    title('Filter Arrangement');
    semilogx(W, 20*log10(abs(H)));
    xlabel('Frequency in Hz');
    ylabel('Magnitude in dB');
    yline(-20, '--k');
    for k = 1:32
        xline(Ff3(k), '--r');
    end
    xlim([30 24000]);
    ylim([-300 20]);
    % disp(BP1');
    % disp(abs(roots(BP1)));
    fprintf(fid, 'Filter %d: fmin = %.2f Hz, fmax = %.2f Hz, fcenter = %d Hz \n', i, Ffinal(i), Ffinal(i+1), Ff3(i+1));
    fprintf(fid, '%.16f \n', BP1);
    fprintf(fid, '\n \n \n');
    Y = filter(BP1, 1, AB);
    Y1 = Y1 + Y;
    % Y = filtfilt(SB1, Ya);
    Yfft = abs(fft(Y));
    sound(Y, F);
    hold off;

    hold on; 
    subplot(4,1,2);
    semilogx(f, Yfft, 'LineWidth',2);
    xlabel('Frequency in Hz');
    ylim([0 5000]);
    xlim([30 24000]);
    for k = 1:32
        xline(Ff3(k), '--r');
    end
    title('Filtered Spectrum');
    ylabel('Amplitude');
    hold off; 

    hold on; 
    subplot(4,1,3);
    semilogx(f, AC, 'LineWidth',2);
    xlabel('Frequency in Hz');
    ylim([0 5000]);
    for k = 1:32
        xline(Ff3(k), '--r');
    end
    title('Original Spectrum');
    ylabel('Amplitude');
    xlim([30 24000]);
    pause; 
    hold off; 
end
pause; 
hold on;
    Y1fft = abs(fft(Y1));
    subplot(4,1,4);
    semilogx(f, Y1fft, 'LineWidth',2);
    xlabel('Frequency in Hz');
    ylim([0 5000]);
    xlim([30 25000]);
    for k = 1:32
        xline(Ff3(k), '--r');
    end
    title('Filtered All Spectrum');
    ylabel('Amplitude');
    hold off;
sound(Y1, F);