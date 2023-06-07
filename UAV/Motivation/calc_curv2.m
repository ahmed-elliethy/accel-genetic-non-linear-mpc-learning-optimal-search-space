function curveture= calc_curv(trj,Tl)
x=(1:Tl);
curvature = 0 ;
y=trj;
    for j =1 : Tl-2
        dx  = gradient(x);
        ddx = gradient(dx);
        dy  = gradient(y);
        ddy = gradient(dy);
        num   =  abs(ddy) ;
        denom = 1 + dy .* dy;
        denom = sqrt(denom) .^ 3;
        curvature = num ./ denom;
    end
       curveture= sum(curvature); 
end
