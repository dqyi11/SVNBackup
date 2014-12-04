% FIRST SAMPLING AND INITIALLIZATION FOR VARIABLES.
global TRAIN ALPHA BETA;
global zDOCU qWORD nDOCU;
global cDOCU cWORDS cTOPIC;

% INITIALLIZATION

distrZZ=ALPHA/sum(ALPHA); % NORMALIZATION
for row=1:cDOCU
	for col=1:cWORDS
		if TRAIN(row,col)~=0
			for i=1:TRAIN(row,col)
				tmpTOPIC=getTopic(distrZZ);

				qWORD(tmpTOPIC,col)=qWORD(tmpTOPIC,col)+1;
				nDOCU(tmpTOPIC,row)=nDOCU(tmpTOPIC,row)+1;

				tmpMAT=zDOCU{row,1};
				tmpMAT=cat(2,tmpMAT,col);
				zDOCU{row,1}=tmpMAT;

				tmpMAT=zDOCU{row,2};
				tmpMAT=cat(2,tmpMAT,tmpTOPIC);
				zDOCU{row,2}=tmpMAT;
			end
		end
	end
end