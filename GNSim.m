

%% Setting up Camera

camera.px      = 320;                   % Principal point [px pixels]
camera.py      = 240;                   % Principal point [py pixels]
camera.mx      = 600;                   % Focal length [mx pixels]
camera.my      = 600;                   % Focal length [my pixels]

% The Intrinsics
K = [camera.mx,     0,     camera.px;
     0,         camera.my, camera.py;
     0,             0,         1     ];