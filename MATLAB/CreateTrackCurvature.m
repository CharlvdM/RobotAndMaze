close all;

% Given parameters
Wc = 1;
r = Wc / 2;
mazeTrackLength = 4 + 8 * (pi / 4);
n = 400;
s_track = linspace(0, mazeTrackLength, n);

% Initialize C array
C_track = zeros(size(s_track));

% Calculate C as a function of s
for i = 1:n
    if s_track(i) < Wc % Cell 1
        C_track(i) = 0;
    elseif s_track(i) < Wc + (pi / 4) * Wc % Cell 2
        C_track(i) = 1 / r;
    elseif s_track(i) < Wc + 3 * (pi / 4) * Wc % Cell 3 & 4
        C_track(i) = -1 / r;
    elseif s_track(i) < Wc + 5 * (pi / 4) * Wc % Cell 5 & 6
        C_track(i) = 1 / r;
    elseif s_track(i) < 2 * Wc + 5 * (pi / 4) * Wc % Cell 7
        C_track(i) = 0;
    elseif s_track(i) < 2 * Wc + 6 * (pi / 4) * Wc % Cell 8
        C_track(i) = -1 / r;
    elseif s_track(i) < 4 * Wc + 6 * (pi / 4) * Wc % Cell 9 & 10
        C_track(i) = 0;
    elseif s_track(i) < 4 * Wc + 7 * (pi / 4) * Wc % Cell 11
        C_track(i) = -1 / r;
    elseif s_track(i) < 4 * Wc + 8 * (pi / 4) * Wc % Cell 12
        C_track(i) = 1 / r;
    else
        C_track(i) = 0;
    end
end

n_interp = 1000;
s_interp = linspace(0, mazeTrackLength, n_interp);

C_interp = interp1(s_track,C_track, s_interp, 'pchip');

% figure;
% plot(s_track, C_track, '-o', 'LineWidth', 1.5);
% hold on
% plot(s_interp, C_interp, 'LineWidth', 1.5);
% title('Change in Track Angle (C) vs Centre Line Displacement (s)');
% xlabel('s (Centre Line Displacement)');
% ylabel('C (Change in Track Angle)');
% legend('Computed Points', 'Interpolation')
% grid on;
