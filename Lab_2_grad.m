function grad = LAB_2_grad(x,y);
grad(1,1) = 4*x^3 +4*x*y - 42*x + 2*y^2 - 14;
grad(2,1) = 4*y^3 - 26*y + 4*x*y + 2*x^2 - 22;
end