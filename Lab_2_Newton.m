function [X, traj, f, k, Err] = Lab_2_Newton(x0,tol)
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
        
        %Evaluate Steepest Descent Formula
        hessi = inv(hess);
        xnew = X - (hessi*grad);
    
        %store solution in a vector
        traj = [traj xnew];
        % Evaluate error
        ea = norm(xnew-X);
        Err = [Err ea];
        X = xnew;
        f(k+1) = Lab_2_Fun(X(1),X(2));
        k = k+1;
    end
end
