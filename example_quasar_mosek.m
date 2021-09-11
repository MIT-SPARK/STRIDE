%% Example: solve the QUASAR SDP relaxation using MOSEK
% The QUASAR SDP relaxation is a tight relaxation for solving the 
% outlier-robust rotation search problem (a.k.a. Wahab problem)
% For a description of the Wahba problem and the QUASAR relaxation, see
% Yang, Heng, and Luca Carlone. 
% "A quaternion-based certifiably optimal solution to the Wahba problem with outliers." 
% In Proceedings of the IEEE/CVF International Conference on Computer Vision, 2019.

%% Start clean
clc; clear; close all; restoredefaultpath;

%% Provide path to MOSEK
mosekpath = '../mosek';
addpath(genpath(pwd));

%% Load problem data
% We provide two example data, both with 50% outliers
% load('./data/quasar_50_1.mat') % Wahba with 50 measurements
load('./data/quasar_100_1.mat') % Wahba with 100 measurements

%% Solve using MOSEK
[At,b,c,K] = SDPT3data_SEDUMIdata(SDP.blk,SDP.At,SDP.C,SDP.b); 
prob       = convert_sedumi2mosek(At, b, c, K);
addpath(genpath(mosekpath))
[~,res]    = mosekopt('minimize info',prob);
[Xopt,yopt,Sopt,obj] = recover_mosek_sol_blk(res,SDP.blk);
rmpath(genpath(mosekpath))
infomosek            = get_performance_quasar(Xopt,yopt,Sopt,SDP,R_gt);

