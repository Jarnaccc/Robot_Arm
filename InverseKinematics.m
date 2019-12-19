%% inverse kinematics

function solutions = InverseKinematics(x,y,z,Alpha,Beta,Gamma)

T06 = GetTransformationMatrix(x,y,z,Alpha,Beta,Gamma);

%black magic 
P4 =  T06*[0;0;-3;1];
%poder ter 2 solucoes, atencao acho que vaira em -pi ou +
o1 = atan2(P4(2),P4(1));


[o2,o3] = Get_o2_o3(o1,P4);

%case first and second possibilities for o2 and o3 
%test if is a impossable point
if(isreal(o2(1)) && isreal(o3(1)))
[o4,o5,o6] = Get_o4_o5_o6(T06,o1,o2(1),o3(1));
solutions.s1= [o1 o2(1) o3(1) o4(1) o5(1) o6(1)];
solutions.s2= [o1 o2(1) o3(1) o4(2) o5(2) o6(2)];
end

if(isreal(o2(2)) && isreal(o3(2)))
[o4,o5,o6] = Get_o4_o5_o6(T06,o1,o2(2),o3(2));
solutions.s3= [o1 o2(2) o3(2) o4(1) o5(1) o6(1)];
solutions.s4= [o1 o2(2) o3(2) o4(2) o5(2) o6(2)];
end

o1 = o1 - pi;
[o2,o3] = Get_o2_o3(o1,P4);

if(isreal(o2(1)) && isreal(o3(1)))
[o4,o5,o6] = Get_o4_o5_o6(T06,o1,o2(1),o3(1));
solutions.s5= [o1 o2(1) o3(1) o4(1) o5(1) o6(1)];
solutions.s6= [o1 o2(1) o3(1) o4(2) o5(2) o6(2)];
end

if(isreal(o2(2)) && isreal(o3(2)))
[o4,o5,o6] = Get_o4_o5_o6(T06,o1,o2(2),o3(2)); 
solutions.s7= [o1 o2(2) o3(2) o4(1) o5(1) o6(1)];
solutions.s8= [o1 o2(2) o3(2) o4(2) o5(2) o6(2)];
end


end

function [o4,o5,o6] = Get_o4_o5_o6(T06,o1,o2,o3)

T01 = [MyCos(o1) -MySin(o1) 0 0;MySin(o1) MyCos(o1) 0 0;0 0 1 99;0 0 0 1];
T12 = [MyCos(o2+pi/2) -MySin(o2+pi/2) 0 25;0 0 -1 0;MySin(o2+pi/2) MyCos(o2+pi/2) 0 0;0 0 0 1];
T23 = [MyCos(o3) -MySin(o3) 0 120;MySin(o3) MyCos(o3) 0 0;0 0 1 0;0 0 0 1];

T03 = T01*T12*T23;
%T03inv = inv(T03);

%T36 = T03inv*T06;
T36 = T03\T06;

Tleft =T36;

%O5 is + or -
o5 = real(double(acos(-Tleft(2,3))));
o5 = [o5 -o5];


[o4,o6] = Get_o4_o6(T03,T06,o5(1),Tleft);
[o4_neg,o6_neg] = Get_o4_o6(T03,T06,o5(2),Tleft);

o4=[o4 o4_neg];
o6=[o6 o6_neg];


end



function [o4,o6] = Get_o4_o6(T03,T06,o5,Tleft)

if(MySin(o5) == 0)
    %solucoes infinitas
    o4=0;
else

%tem duas solucoes, acho que troca o sinal mas se for zero pode ser pi
o4 = atan2(-Tleft(3,3)/MySin(o5), (2-Tleft(1,4))/(3*MySin(o5))) - pi/2;
    
end

 o6 =  Get_o6(T03,T06,o5,o4);

end

function o6 = Get_o6(T03,T06,o5,o4)
o4 = real(o4);
T34 = [MyCos(o4+pi/2) -MySin(o4+pi/2) 0 2;0 0 -1 -130;MySin(o4+pi/2) MyCos(o4+pi/2) 0 0;0 0 0 1];
T45 = [MyCos(o5) -MySin(o5) 0 0;0 0 -1 0;MySin(o5) MyCos(o5) 0 0;0 0 0 1];

T05 = T03*T34*T45;

%T05inv = inv(T05);

%T56L = T05inv*T06;
T56L = T05\T06;

%o sinal troca como no anterior
o6 = atan2(-T56L(1,2),T56L(1,1));

o6 = real(o6);
end

%supposedly GeometricMethod
function [o2_Value,o3_Value] = Get_o2_o3(o1,P4)
syms o2 o3;
T01 = [MyCos(o1) -MySin(o1) 0 0;MySin(o1) MyCos(o1) 0 0;0 0 1 99;0 0 0 1];
T12 = [MyCos(o2+pi/2) -MySin(o2+pi/2) 0 25;0 0 -1 0;MySin(o2+pi/2) MyCos(o2+pi/2) 0 0;0 0 0 1];
T23 = [MyCos(o3) -MySin(o3) 0 120;MySin(o3) MyCos(o3) 0 0;0 0 1 0;0 0 0 1];
%the rotation doesn't matter, so we put a random one, we only need the
%translation
o4=0 + pi/2;
T34 = [round(MyCos(o4)) round(-MySin(o4)) 0 2;0 0 -1 -130;round(MySin(o4)) round(MyCos(o4)) 0 0;0 0 0 1];

T04 = T01*T12*T23*T34;

Po= T04*[0;0;0;1];



[solo2 solo3] = SolveEquations(P4,Po,o1);

o2_Value = [double(solo2(1)) , double(solo2(2))];
o3_Value = [double(solo3(1)) , double(solo3(2))];


end

%problem when the 2 equations are dependet (the same)
function [solo2, solo3] = SolveEquations(P4,Po,o1)
syms o2 o3

x = P4(1);
y = P4(2);
z = P4(3);

vars = [o2 o3];

eqx =cos(o1)*(-2*sin(o2 + o3) + 130*cos(o2 + o3) - 120*sin(o2) + 25);
eqy =sin(o1)*(-2*sin(o2 + o3) + 130*cos(o2 + o3) - 120*sin(o2) + 25) ;
eqz =130*sin(o2 + o3) + 2*cos(o2 + o3) + 120*cos(o2) + 99;

eqns = [eqx == x ,eqy == y,eqz == z];
[solo2, solo3] = solve(eqns, vars); 



if(x  ~= 0 && y ~= 0)
    eqns = [ Po(1) == x ,Po(2) == y];
    [solo2, solo3] = solve(eqns, vars);
end

if(x  ~= 0 && z ~= 0)
    eqns = [ Po(1) == x ,Po(3) == z];
    [solo2, solo3] = solve(eqns, vars);
end

if(y  ~= 0 && z ~= 0)
    eqns = [Po(2) == y ,Po(3) == z];
    [solo2, solo3] = solve(eqns, vars);
end

end

function T = GetTransformationMatrix(x,y,z,a,b,g)

T=[(MyCos(a)*MyCos(b)*MyCos(g)-MySin(a)*MySin(g)) (-MyCos(a)*MyCos(b)*MySin(g)-MySin(a)*MyCos(g)) MyCos(a)*MySin(b) x;
   (MySin(a)*MyCos(b)*MyCos(g)+MyCos(a)*MySin(g)) (-MySin(a)*MyCos(b)*MySin(g)+MyCos(a)*MyCos(g)) MySin(a)*MySin(b) y;
   -MySin(b)*MyCos(g)                              MySin(b)*MySin(g)                              MyCos(b)          z;
    0                                              0                                              0                 1;];



end



%problem with zero....
function b = MyCos(a)
if(a == pi/2 || a == -pi/2)
    b=0;
else
    b = cos(a);
end    

end

function b = MySin(a)
if(a == 0 || a == pi)
    b=0;
else
    b = sin(a);
end   
end