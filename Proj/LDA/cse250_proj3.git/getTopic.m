% GET A NEW TOPIC FROM CURRENT DISTRIBUTION
function [rTOPIC]=getTopic(distrCURNZ)
	global cTOPIC;

	% NORMALIZATION
	distrCURNZ=distrCURNZ.*(distrCURNZ>0);
	distrCURNZ=distrCURNZ./sum(distrCURNZ);

	rTOPIC=randsample(1:cTOPIC,1,true,distrCURNZ);
