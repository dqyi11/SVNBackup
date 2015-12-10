var =2 

%lamda = 1
errRec3 = [0.898124258777809,0.789847448101539,0.689728369447266,0.584386735339263,0.482369204160322,0.391221575965740,0.329951908059397,0.283079686712174,0.238668038067177,0.184199410310800,0.161914461139738,0.129620807557883,0.124049125917133,0.105439182596037,0.0991959484534907,0.102607977351977,0.0696096100862303,0.0619560004691203,0.0541433508831183,0.0565284658888836;];
errRec4 = [0.979970290627928,0.883154457825104,0.779525076836014,0.675772522639903,0.571982214706620,0.474660627116033,0.391378416702693,0.329691144035449,0.275880931078501,0.217835859770771,0.174589633052007,0.150736119011566,0.128743308336223,0.117454709363300,0.104897370520606,0.0967215924337256,0.0844115739785358,0.0684256769599504,0.0553191914689580,0.0487876795026510;];

%lamda = 2
errRec5 = [0.915031667488083,0.827724315822153,0.716206376619778,0.615733030584489,0.524553878387084,0.444567984519428,0.363493216264214,0.314563270276392,0.270037567909238,0.218883623153012,0.183989100317971,0.152737386176429,0.122160981994110,0.108871494418878,0.102216985027020,0.0863325648912668,0.0854119521421547,0.0890118190802674,0.0851505683603401,0.0867639685858807;];
errRec6 = [0.980805382592281,0.907353747138750,0.816693891706276,0.713161767074616,0.614369275339388,0.523981664184647,0.442649073326810,0.371811375839559,0.316875200752906,0.264779065833679,0.214657211249170,0.174339332611266,0.147388332366042,0.123894280784878,0.109844389892929,0.0975614682904956,0.0866224661007405,0.0800666347426595,0.0793357890783137,0.0790552652884716;];

%lamda = 5
errRec7 = [0.943043652564157,0.881882341631421,0.811644773788818,0.734891861348146,0.658424525129519,0.585555741005762,0.524956096092757,0.469838559582696,0.403732320655499,0.359226832420352,0.322587986813755,0.286673750920092,0.249934988481142,0.210877357934932,0.181797840083129,0.144034272646890,0.110200802586270,0.0850296855578126,0.0659979686926675,0.0502166248186947;];
errRec8 = [0.981720067444913,0.934491154163369,0.874615196162500,0.805315503577825,0.732721703498255,0.659494193627013,0.589244435370739,0.529134070864286,0.470858612339996,0.416807701754680,0.369670648923764,0.327455375037064,0.287588009796821,0.251197302826565,0.215638996466410,0.182105153492378,0.145826993385575,0.112009771263465,0.0842853930077235,0.0625080197768974;];

%lamda = 10

errRec9 = [0.961552741722017,0.911756504738652,0.861816832967686,0.806481323722973,0.752304374429351,0.702509322223881,0.649271894611918,0.600215793957073,0.550353754608300,0.494854262936145,0.448580941940870,0.405898707433771,0.366876050339012,0.323495194764238,0.284566568848980,0.249847370159879,0.214188675878484,0.178808667727685,0.152142993912456,0.130583520035484;];
errRec10 = [0.984627381471677,0.951067874293629,0.907664953580528,0.859705445233490,0.807785652080356,0.756477488718255,0.704668824590339,0.654623260453623,0.604929774156278,0.553990886332414,0.504706349255674,0.458021247950116,0.414721998713083,0.371924659918851,0.330682830264872,0.293753197995522,0.258341280800360,0.222977120561455,0.190624541293821,0.163876973303852;];

line = 1:1:20
plot(line, errRec3, '-r', line, errRec5, '-b' , line, errRec7, '-g', line, errRec10, '-y');
legend('lamda = 1', 'lamda = 2', 'lamda = 5', 'lamda = 10');
xlabel('epoch');
ylabel('mean error ratio');