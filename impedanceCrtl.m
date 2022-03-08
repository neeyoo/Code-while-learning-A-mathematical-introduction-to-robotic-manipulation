function tauVec = impedanceCrtl(X, Xd, Xdd, M, B, K, Jb)


tauVec = -Jb' * (M*Xdd + B*Xd + K*X);

end