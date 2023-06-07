function [feature,margin,summ,curv,curr_err] = calcchanges...
(ref,inp,inp_old,hz,xk,c,summ_old,curv_old,curr_err_old)
inpDiff = abs(inp(1:hz,:)-inp_old(1:hz,:)); 
margin=max(max(inpDiff)) ;

%% save feature record
curr_err = max(abs(ref(1:3,1)-xk')) ;
summ=0;
for j=1:3
    for kk=1:hz-1
     summ  = summ+abs(ref(j,kk+1)-ref(j,kk));
    end
         curv(j)  = calc_curv2(ref(j,1:hz),hz);
%          statesDif(i,j) = abs(curve(j)-curve2(j));
%          wavelet(j) = max(wavedec(ref(j,1:hz),3,'db2'));
end
feature = abs(curr_err);
% feature2 = abs(curr_err-curr_err_old);
% feature = [feature1 , feature2];

% feature1 = max(curv);
% feature2 = curr_err;
% feature = abs(summ-summ_old);
% feature = max(wavelet);
end