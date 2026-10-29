% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 428.416546054515950 ; 417.500915513914040 ];

%-- Principal point:
cc = [ 393.848798118113280 ; 274.881587014975250 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.392151390797248 ; 0.170605363082444 ; 0.000990044539824 ; 0.000188116452156 ; -0.038107679361224 ];

%-- Focal length uncertainty:
fc_error = [ 1.211097270340999 ; 1.149394703220486 ];

%-- Principal point uncertainty:
cc_error = [ 0.658527138507350 ; 0.649800698573866 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.003519153878137 ; 0.004915979838710 ; 0.000295289948009 ; 0.000238951555807 ; 0.002176154569590 ];

%-- Image size:
nx = 760;
ny = 570;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 14;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 1 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -3.028942e+000 ; -5.894813e-001 ; 2.332109e-001 ];
Tc_1  = [ -3.162034e+001 ; 1.104842e+001 ; 5.036656e+001 ];
omc_error_1 = [ 6.423602e-003 ; 2.496273e-003 ; 8.757772e-003 ];
Tc_error_1  = [ 8.858530e-002 ; 9.235378e-002 ; 1.781404e-001 ];

%-- Image #2:
omc_2 = [ 3.524313e-001 ; -2.599721e-001 ; -1.434710e+000 ];
Tc_2  = [ -1.608941e+001 ; -1.244349e+001 ; 2.475023e+001 ];
omc_error_2 = [ 4.200869e-003 ; 4.422122e-003 ; 1.199717e-003 ];
Tc_error_2  = [ 6.202475e-002 ; 5.404508e-002 ; 1.206662e-001 ];

%-- Image #3:
omc_3 = [ -3.010383e+000 ; -6.023685e-001 ; 5.442423e-001 ];
Tc_3  = [ -8.362675e+000 ; 2.721928e+000 ; 4.990888e+001 ];
omc_error_3 = [ 2.638122e-003 ; 1.161462e-003 ; 3.723109e-003 ];
Tc_error_3  = [ 8.012283e-002 ; 8.074217e-002 ; 1.386721e-001 ];

%-- Image #4:
omc_4 = [ -3.079783e+000 ; -4.984524e-001 ; 1.258667e-002 ];
Tc_4  = [ -7.721812e-002 ; 9.762941e-001 ; 7.045396e+001 ];
omc_error_4 = [ 1.254615e-003 ; 3.909389e-004 ; 2.149250e-003 ];
Tc_error_4  = [ 1.162243e-001 ; 1.165374e-001 ; 1.946282e-001 ];

%-- Image #5:
omc_5 = [ -5.847610e-001 ; 3.983667e-001 ; -9.421890e-001 ];
Tc_5  = [ -8.518624e+000 ; -1.782799e+001 ; 3.351157e+001 ];
omc_error_5 = [ 3.030111e-003 ; 4.454893e-003 ; 1.485003e-003 ];
Tc_error_5  = [ 6.305805e-002 ; 5.571218e-002 ; 7.900050e-002 ];

%-- Image #6:
omc_6 = [ -2.911378e+000 ; -3.400316e-001 ; -8.546612e-001 ];
Tc_6  = [ -4.199040e+000 ; 3.395469e+000 ; 1.900816e+001 ];
omc_error_6 = [ 5.792877e-003 ; 2.312664e-003 ; 6.599016e-003 ];
Tc_error_6  = [ 3.503283e-002 ; 3.243669e-002 ; 5.672499e-002 ];

%-- Image #7:
omc_7 = [ -2.984542e+000 ; -4.125594e-001 ; -4.547415e-001 ];
Tc_7  = [ -4.896507e+000 ; 2.250621e+000 ; 3.578459e+001 ];
omc_error_7 = [ 4.716663e-003 ; 1.827592e-003 ; 6.652019e-003 ];
Tc_error_7  = [ 6.011179e-002 ; 5.960514e-002 ; 1.072027e-001 ];

%-- Image #8:
omc_8 = [ -2.957470e+000 ; -4.126573e-001 ; -4.020689e-001 ];
Tc_8  = [ -9.435974e+000 ; 2.265750e+000 ; 5.457160e+001 ];
omc_error_8 = [ 1.295661e-003 ; 6.310071e-004 ; 2.314621e-003 ];
Tc_error_8  = [ 9.183571e-002 ; 9.119950e-002 ; 1.483089e-001 ];

%-- Image #9:
omc_9 = [ -2.971784e+000 ; -4.409364e-001 ; -3.099028e-001 ];
Tc_9  = [ -3.273618e+000 ; 1.054399e+000 ; 6.160209e+001 ];
omc_error_9 = [ 1.393983e-003 ; 5.962354e-004 ; 2.307833e-003 ];
Tc_error_9  = [ 1.012813e-001 ; 1.019768e-001 ; 1.680446e-001 ];

%-- Image #10:
omc_10 = [ -2.934118e+000 ; -7.804778e-001 ; 6.902549e-001 ];
Tc_10  = [ 3.838274e+000 ; 4.140206e+000 ; 4.417435e+001 ];
omc_error_10 = [ 3.231771e-003 ; 1.654007e-003 ; 4.608119e-003 ];
Tc_error_10  = [ 7.169827e-002 ; 7.245593e-002 ; 1.296767e-001 ];

%-- Image #11:
omc_11 = [ 2.466897e+000 ; 5.090220e-001 ; 1.147170e-001 ];
Tc_11  = [ -2.547043e+000 ; -5.482602e+000 ; 4.708596e+001 ];
omc_error_11 = [ 3.484659e-003 ; 2.008358e-003 ; 5.384940e-003 ];
Tc_error_11  = [ 7.651936e-002 ; 7.655245e-002 ; 1.204063e-001 ];

%-- Image #12:
omc_12 = [ -2.481997e+000 ; -2.127743e-001 ; 1.531840e-001 ];
Tc_12  = [ -8.690813e+000 ; -8.903536e+000 ; 1.762283e+001 ];
omc_error_12 = [ 3.750365e-003 ; 2.044514e-003 ; 5.338807e-003 ];
Tc_error_12  = [ 3.546553e-002 ; 4.226905e-002 ; 8.185641e-002 ];

%-- Image #13:
omc_13 = [ -2.470586e+000 ; 2.072014e-001 ; -2.712385e-001 ];
Tc_13  = [ -7.226710e+000 ; -1.120484e+001 ; 2.106918e+001 ];
omc_error_13 = [ 3.807288e-003 ; 1.799138e-003 ; 4.754153e-003 ];
Tc_error_13  = [ 4.431985e-002 ; 4.897953e-002 ; 9.324320e-002 ];

%-- Image #14:
omc_14 = [ -2.522215e+000 ; -2.377618e-001 ; 4.364407e-001 ];
Tc_14  = [ 2.224641e+000 ; 1.573816e+000 ; 4.497119e+001 ];
omc_error_14 = [ 1.946032e-003 ; 8.161159e-004 ; 2.133687e-003 ];
Tc_error_14  = [ 7.180066e-002 ; 7.370396e-002 ; 1.202359e-001 ];

