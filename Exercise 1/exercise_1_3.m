%% Advanced motion control - exercise 1.3
clear all; close all; clc

% lower triangular mxm matrix A with a(ii) = -1, a(ij) = 1 for all i>j, a(ij) = 0 for all i<j

% set size m and create the matrix A
m = 4;              % Size m
A = zeros(m,m);     % Pre-allocate
for i = 1:m
    for j = 1:m
        % a(ii) = -1
        if (i == j)
            A(i,j) = -1; 
        elseif (i>j)
            A(i,j) = 1;
        end
    end
end

% Show A
A

% a) What is det(A)?
detA = det(A)

% b) What are the eigenvalues of A?
eigA = eig(A)

% c) What is the RGA of A?
RGA_A = A.*inv(A).'

% d) let m = 4, find an E with the smallest maximum singular value such
% that A+E is singular.

% Chosen E: zeros with 1 at (1,1) because then the matrix is not invertible
% anymore (0 at diagonal)

[U_A,S_A,V_A] = svd(A)
[minSigmaA,indx] = min(diag(S_A))

if (false)
    E = zeros(m,m);
    E(1,1) = 1;
else
    E = (-U_A(:,indx))*minSigmaA*V_A(:,indx)'         % from book, page 524: does not obtain the lowest max(sigma(E))
end

[U_E,S_E,V_E] = svd(E);

% Determine maximum sigma of E
maxSigmaE = max(max(S_E))

% Determine if A+E is singular (det(A+E) = 0)
detAE = det(A+E)

