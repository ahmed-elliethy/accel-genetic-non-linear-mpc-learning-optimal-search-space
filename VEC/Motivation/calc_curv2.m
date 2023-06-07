function curveture= calc_curv(trj,t)
x=t;
y=trj;
dx  = gradient(x);
ddx = gradient(dx);
dy  = gradient(y);
ddy = gradient(dy);
num   = dx .* ddy - ddx .* dy;
denom = dx .* dx + dy .* dy;
denom = sqrt(denom) .^ 3;
        curv = num ./ denom;
        curveture= sum(abs(curv)); 
end
