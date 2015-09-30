function predictedY = dnsvcoutput(tstX, trX, concensusV,  paramA, paramB, paramC, ker)
n = size(tstX,1);

K1 = svkernel2(ker, trX, tstX);
K2 = svkernel2(ker, concensusV, tstX);
partA = paramA' * K1;
partC = paramC' * K2;
partB = paramB * ones(1,n);
predictedY = sign( partA + partB + partC );

end