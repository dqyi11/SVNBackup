% GET THETA AND PHI
global zDOCU;
global cDOCU;
global THETA PHI;
global qWORD nDOCU;
global cTOPIC cWORDS;
global LABEL;

THETA=nDOCU./repmat(sum(nDOCU,1),cTOPIC,1);
PHI=qWORD./repmat(sum(qWORD,2),1,cWORDS);

% [~, iTHETA]=max(THETA,[],1);
% Accuray=nnz(iTHETA-LABEL)/size(iTHETA,2)
