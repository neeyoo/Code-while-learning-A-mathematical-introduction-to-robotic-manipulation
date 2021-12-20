function omega_skew = skew(omega)
% This function is used for turning a vector to a skew-symmetric matrix
% where A' = -A

omega_skew = [0         -omega(3) omega(2);
              omega(3)  0         -omega(1);
              -omega(2) omega(1)  0        ];
          


end