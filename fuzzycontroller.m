Ts = 0.1;
Plant = c2d(zpk([],[-1 -3 -5],1),Ts);
open_system('sllookuptable');   %%open look up table
% open_system('sllookuptable/Fuzzy PID'); % open fuzzy PID
% open_system('sllookuptable/Conventional PID'); % open conventional PID
C0 = pid(1,1,1,'Ts',Ts,'IF','B','DF','B');    % configure parameters of PID controller
C = pidtune(Plant,C0);                         % parameter tunning
[Kp,Ki,Kd] = piddata(C);                      % showcase the PID controller

%% configure the parameter of the fuzzy PID inference engine
FIS = sugfis;
FIS = addInput(FIS,[-10 10],'Name','E');
FIS = addMF(FIS,'E','trimf',[-20 -10 0],'Name','Negative');
FIS = addMF(FIS,'E','trimf',[-10 0 10],'Name','Zero');
FIS = addMF(FIS,'E','trimf',[0 10 20],'Name','Positive');
FIS = addInput(FIS,[-10 10],'Name','CE');
FIS = addMF(FIS,'CE','trimf',[-20 -10 0],'Name','Negative');
FIS = addMF(FIS,'CE','trimf',[-10 0 10],'Name','Zero');
FIS = addMF(FIS,'CE','trimf',[0 10 20],'Name','Positive');
FIS = addOutput(FIS,[-20 20],'Name','u');
FIS = addMF(FIS,'u','constant',-20,'Name','LargeNegative');
FIS = addMF(FIS,'u','constant',-10,'Name','SmallNegative');
FIS = addMF(FIS,'u','constant',0,'Name','Zero');
FIS = addMF(FIS,'u','constant',10,'Name','SmallPositive');
FIS = addMF(FIS,'u','constant',20,'Name','LargePositive');
ruleList = [1 1 1 1 1;   % Rule 1                         输入+输出数量+权重+逻辑关系1-and，2or
            1 2 2 1 1;   % Rule 2
            1 3 3 1 1;   % Rule 3
            2 1 2 1 1;   % Rule 4
            2 2 3 1 1;   % Rule 5
            2 3 4 1 1;   % Rule 6
            3 1 3 1 1;   % Rule 7
            3 2 4 1 1;   % Rule 8
            3 3 5 1 1];  % Rule 9
FIS = addRule(FIS,ruleList);
gensurf(FIS);
%% configure the parameters of the PID controller and testing;
GE = 10; 
GCE = GE*(Kp-sqrt(Kp^2-4*Ki*Kd))/2/Ki;
GCU = Ki/GE;
GU = Kd/GCE;
Step = 10;
E = -10:Step:10;
CE = -10:Step:10;
N = length(E);
LookUpTableData = zeros(N);
for i=1:N
   for j=1:N
      % Compute output u for each combination of sample points.
      LookUpTableData(i,j) = evalfis(FIS,[E(i) CE(j)]);
   end
end
% open_system('sllookuptable/Fuzzy PID using Lookup Table');
sim('sllookuptable')
open_system('sllookuptable/Scope')

%%%%%%%%%%%%%Fuzzy PID with nonlinear control surface
FIS = sugfis;
FIS = addInput(FIS,[-10 10],'Name','E');
FIS = addMF(FIS,'E','gaussmf',[7 -10],'Name','Negative');
FIS = addMF(FIS,'E','gaussmf',[7 10],'Name','Positive');
FIS = addInput(FIS,[-10 10],'Name','CE');
FIS = addMF(FIS,'CE','gaussmf',[7 -10],'Name','Negative');
FIS = addMF(FIS,'CE','gaussmf',[7 10],'Name','Positive');
FIS = addOutput(FIS,[-20 20],'Name','u');
FIS = addMF(FIS,'u','constant',-20,'Name','Min');
FIS = addMF(FIS,'u','constant',0,'Name','Zero');
FIS = addMF(FIS,'u','constant',20,'Name','Max');
ruleList = [1 1 1 1 1;...   % Rule 1
            1 2 2 1 1;...   % Rule 2
            2 1 2 1 1;...   % Rule 3
            2 2 3 1 1];     % Rule 4
FIS = addRule(FIS,ruleList);
gensurf(FIS)
Step = 1;
E = -10:Step:10;
CE = -10:Step:10;
N = length(E);
LookUpTableData = zeros(N);
for i=1:N
   for j=1:N
      % Compute output u for each combination of sample points.
      LookUpTableData(i,j) = evalfis(FIS,[E(i) CE(j)]);
   end
end
sim('sllookuptable')