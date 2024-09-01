%% DR DIETMAR's LOGIC

% function findCurvedTrials(data)
% 
% nData = length(data.data.part);
% 
% mdAll = [];
% trajectories = {};
% 
% 
% 
% for iData = 1: nData
%     mdAll = [mdAll data.data.part(iData).dv.maxDev(data.data.part(iData).iv.position==1)];
%     trajectories = {trajectories data.data.part.dv.trajectory{data.data.part(iData).iv.position==1}};
% 
% end
% 
% sMd = std(mdALL);
% mMd = mean(mdAll);
% idxCurved = find(abs(mdAll - mMd) > sMd);
% idxStraight = find(abs(mdAll - mMd) < sMd);
% 
% nCurved = length(idxStraight);
% 
% for iCurved=1:nCurved
%     anglesCurved(iCurved, :) = inverseKinematic(trajectories{idxCurved(iCurved});
% end
% 
% meanAnglesCurved = mean(anglesCurved,1);
% plot(meanAnglesCurved);
% 
% 
% nCurved = length(idxStraight);
% 
% for iCurved=1:nCurved
%     anglesCurved(iCurved, :) = inverseKinematic(trajectories{idxCurved(iCurved});
% end
% 
% meanAnglesCurved = mean(anglesCurved,1);
% plot(meanAnglesCurved);
% end
% 
% 
