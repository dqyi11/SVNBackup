% WHEN GET A NEW TOPIC VALUE UPDATE ALL RELATED MATRIX
function newt=update(word,docu,topic)
	global qWORD nDOCU;

	% DELETE OLD DATA
	distrZZ=getDistr(word,docu,topic);
	tmpTOPIC=getTopic(distrZZ);

	qWORD(topic,word)=qWORD(topic,word)-1;
	nDOCU(topic,docu)=nDOCU(topic,docu)-1;

	qWORD(tmpTOPIC,word)=qWORD(tmpTOPIC,word)+1;
	nDOCU(tmpTOPIC,docu)=nDOCU(tmpTOPIC,docu)+1;

	newt=tmpTOPIC;
