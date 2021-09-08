%% Example: solve the QUASAR SDP relaxation using STRIDE
% The QUASAR SDP relaxation is a tight relaxation for solving the 
% outlier-robust rotation search problem (a.k.a. Wahab problem)
% For a description of the Wahba problem and the QUASAR relaxation, see
% Yang, Heng, and Luca Carlone. 
% "A quaternion-based certifiably optimal solution to the Wahba problem with outliers." 
% In Proceedings of the IEEE/CVF International Conference on Computer Vision, 2019.

%% Start clean
clc; clear; close all; restoredefaultpath;

%% Add STRIDE to matlab path, and provide path to dependencies
addpath(genpath(pwd));
manoptpath      = '../manopt'; % required for local search
sdpnalpath      = '../SDPNAL+v1.0'; % required for ADMM+

%% Load problem data
load('./data/quasar_50_1.mat')

% choose whether to run graduated non-convexity (GNC) for primal initialization
% For details about GNC, see
% Yang, Heng, Pasquale Antonante, Vasileios Tzoumas, and Luca Carlone. 
% "Graduated non-convexity for robust spatial perception: From non-minimal solvers to global outlier rejection." 
% IEEE Robotics and Automation Letters, 2020
rungnc          = true;

%% Solve using STRIDE
addpath(genpath(manoptpath)); % add manopt to path

% set parameters for STRIDE
options.pgdStepSize     = 10; % step size, default 10
options.maxiterPGD      = 5; % maximum outer iterations for STRIDE, default 5-10
options.SDPNALpath      = sdpnalpath; % provide path to SDPNAL
options.tolADMM         = 1e-4; % tolerance for warmstart, decrease this parameter for a better warmstart (but takes more time)
options.tolPGD          = 1e-8; % tolerance on KKT residual of the SDP

% provide implementation to the local search method
options.rrOpt           = 1:3; % round the leading 3 eigenvectors to generate hypotheses
options.rrFunName       = 'local_search_quasar'; % name of the .m file that implements the local search

% Primal initialization
if rungnc
    [R_gnc,info_gnc]    = GNC_Wahba(v1,v2,barc2,2.0);
    q_gnc               = rotm2quat(R_gnc); q_gnc = [q_gnc(2:4),q_gnc(1)]';
    v_gnc               = kron([1;info_gnc.theta_gnc],q_gnc);
    X0                  = {v_gnc*v_gnc'};
else
    X0                  = [];
end

% call STRIDE
[outPGD,Xopt,yopt,Sopt] = PGDSDP(SDP.blk, SDP.At, SDP.b, SDP.C, X0, options);
infostride              = get_performance_quasar(Xopt,yopt,Sopt,SDP,R_gt);



