% GET DISTRIBUTION USING GIBBS SAMPLING FORMULAR
function [distr]=getDistr(word,docu,topic)
	global qWORD nDOCU;
	global ALPHA BETA;
	global cTOPIC;

	% ANSWER IS A VECTOR
	distr=zeros(cTOPIC,1);

	% GET Q VALUE EXCEPT CURRENT POINT
	q=qWORD(:,word);
	q(topic)=q(topic)-1;

	% GET N VALUE EXCEPT CURRENT POINT
	n=nDOCU(:,docu);
	n(topic)=n(topic)-1;

	% GET SUM
	sumQ=sum(qWORD,2)-qWORD(:,word)+q;
	sumN=sum(n);

	distr=(q+BETA(word))./(sumQ+sum(BETA)).*(n+ALPHA)./(sumN+sum(ALPHA));