function showFeatures( currImg, currTrackedPts )
%plotTrajectories shows image frames, along with indicated feature points

% Plotting and showing Img
imshow(currImg); hold on;
h = plot(currTrackedPts(:,1), currTrackedPts(:,2),'+', 'LineWidth',1, 'MarkerSize',4,'Color','g');
% plot(currTrackedPts(:,1), currTrackedPts(:,2),'--rs', 'LineWidth',1, 'MarkerSize',4,...
%     'MarkerEdgeColor','c', 'MarkerFaceColor','b');
drawnow;
% %         -------
% filename = [num2str(randi(10000,1,1)) '.png'];
% saveas(h, filename);

end

