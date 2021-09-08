function rotError = getAngularError(R_gt,R_est)

rotError = rad2deg(abs(acos( (trace(R_gt' * R_est)-1) / 2 )));