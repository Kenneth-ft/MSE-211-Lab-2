function [X, traj, f, k, Err] = Lab_2_sdm(x0,tol)
k = 0; ea = 1;
X = x0;
traj = [];
f(1) = Lab_2_Fun(X(1),X(2));
Err = NaN;
    while ea > tol
        % evaluate gradient 
        grad = Lab_2_grad(X(1),X(2));
    
        % evaluate hessian
        hess = Lab_2_hess(X(1), X(2));
    
        %evaluate step size
        gradt = transpose(grad);
        hnum = (gradt*grad);
        hden = gradt*hess*grad;
        h = norm(hnum/hden);
    
        %Evaluate Steepest Descent Formula
%         nextx = x-grad*h;
%         nexty = y-grad*h;
%         xnew = [nextx nexty];
        xnew = X - grad*h;
    
        %store solution in a vector
        traj = [traj xnew];

        % Evaluate error
        ea = norm(xnew-X);
        X = xnew;
        Err = [Err ea];
        f(k+1) = Lab_2_Fun(X(1),X(2));
        k = k+1;
    end 

end

