

% Long testing
imuNoise = 1.08e-4;
imuBias = 1.00e-5;
camNoise = 1.3;

msckf2(imuNoise, imuBias, camNoise);

% 
% for camNoise = 1:0.1:6
%     msckf2(imuNoise, imuBias, camNoise);
% end

% Saving all figures
h = get(0,'children');
for i=1:length(h)
saveas(h(i), num2str(h(i).Number), 'fig');
end
% msckf2(imuNoise, imuBias, camNoise);