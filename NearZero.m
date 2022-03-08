function judge = NearZero(near)

% Takes a scalar.
% Checks if the scalar is small enough to be neglected.

judge = norm(near) < 1e-6;
end