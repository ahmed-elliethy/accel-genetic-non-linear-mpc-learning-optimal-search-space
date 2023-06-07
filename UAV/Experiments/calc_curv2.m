function curveture= calc_curv2(trj,t)
x=t;
y=trj;
        dx  = gradient(x);
        ddx = gradient(dx);
        dy  = gradient(y);
        ddy = gradient(dy);
        num   =  abs(ddy) ;
        denom = 1 + dy .* dy;
        denom = sqrt(denom) .^ 3;
        curveture = num ./ denom;
       curveture= sum(curveture); 
end
