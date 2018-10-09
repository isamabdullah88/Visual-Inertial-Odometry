function[ State ] = resetState( State, SubState, indToRem, indToRemCov )
%resetState This function resets every sub - state of State, after it is
%corrected

State.imuState = SubState.imuState;
State.camState(indToRem) =[]; % Note round brackets

State.imuCovMat = SubState.imuCovMat;

State.camCovMat(indToRemCov, : ) = [];
State.camCovMat( : , indToRemCov) = [];

State.cam_imu_CovMat( : , indToRemCov) = [];

State.N = (State.camState);

end

