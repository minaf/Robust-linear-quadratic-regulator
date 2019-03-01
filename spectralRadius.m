function s = spectralRadius(A)

assert(~any(isnan(A(:))));
tmp = eig(A);
s = max(sqrt(diag(tmp*tmp')));

end