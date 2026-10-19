function results(wall_e, movements, transformations, c, R)
% REPORT_RESULTS  Numbers and plots that back up the write-up:
%   joint angles vs. limits, position error, and tool-z vs. sphere-normal error.
N = size(movements,1);
pos_err = zeros(N,1); ang_err = zeros(N,1); sphere_err = zeros(N,1);
for i = 1:N
    T = DH_FK(wall_e, movements(i,:));
    p = T(1:3,4)';
    pos_err(i)    = norm(p - transformations(1:3,4,i)');
    sphere_err(i) = abs(norm(p - c(:)') - R);
    inward = (c(:)' - p) / norm(c(:)' - p);                 % inward unit normal
    ang_err(i) = acosd(max(-1, min(1, T(1:3,3)' * inward')));  % tool z vs inward normal
end
step = rad2deg(abs(diff(movements,1,1)));
fprintf('Max position error      : %.2e m\n', max(pos_err));
fprintf('Max distance from sphere: %.2e m\n', max(sphere_err));
fprintf('Max tool-z vs normal    : %.3f deg\n', max(ang_err));
fprintf('Max joint step          : %.1f deg per frame\n', max(step(:)));
fprintf('Joint range used        : [%.0f, %.0f] deg (limits +/-360)\n', ...
    rad2deg(min(movements(:))), rad2deg(max(movements(:))));

figure; plot(rad2deg(movements), 'LineWidth', 1.2); grid on; hold on;
yline(360, 'k--'); yline(-360, 'k--');
xlabel('Step'); ylabel('Joint angle (deg)'); legend('q1','q2','q3','q4','q5','q6');
title('Joint angles (smooth, within \pm360^\circ limits)');
figure; subplot(2,1,1); plot(1e3*pos_err); grid on; ylabel('Position error (mm)');
subplot(2,1,2); plot(ang_err); grid on; ylabel('Tool z vs normal (deg)'); xlabel('Step');
end
