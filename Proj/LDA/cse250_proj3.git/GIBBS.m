% PROGRESS OF GIBBS SAMPLING
global zDOCU;
global cDOCU;
global EPOCH;

for i=1:EPOCH
	for d=1:cDOCU
		tmpWORD=zDOCU{d,1};
		tmpTOPIC=zDOCU{d,2};

		if size(tmpTOPIC,2)>0
			for j=1:size(tmpTOPIC,2)
				topic=tmpTOPIC(j);
				word=tmpWORD(j);
				tmpTOPIC(j)=update(word,d,topic);
			end
		end

		zDOCU{d,1}=tmpWORD;
		zDOCU{d,2}=tmpTOPIC;
	end
	fprintf('EPOCH %d DONE\n',i);
end
