function hess = Lab_2_hess(x,y)
hess(1,1) =12*x^2 +4*y - 42;
hess(1,2) = 4*x + 4*y;
hess(2,2) = 12*y^2 + 4*x - 26;
hess(2,1) = 4*x + 4*y;
end