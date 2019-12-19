%% direct kinematics

function [x,y,z,Alpha,Beta,Gamma] = DirectKinematics(o1,o2,o3,o4,o5,o6)

D_H = [0        0   99  o1;
       pi/2     25  0   (o2+pi/2);
       0        120 0   o3;
       pi/2     2   130 (o4+pi/2);
       pi/2     0   0   o5;
       (-pi/2)  0   3   o6];
   
T01 = GetT(D_H(1,1),D_H(1,2),D_H(1,3),D_H(1,4));
T12 = GetT(D_H(2,1),D_H(2,2),D_H(2,3),D_H(2,4));
T23 = GetT(D_H(3,1),D_H(3,2),D_H(3,3),D_H(3,4));
T34 = GetT(D_H(4,1),D_H(4,2),D_H(4,3),D_H(4,4));
T45 = GetT(D_H(5,1),D_H(5,2),D_H(5,3),D_H(5,4));
T56 = GetT(D_H(6,1),D_H(6,2),D_H(6,3),D_H(6,4));

T06 = T01*T12*T23*T34*T45*T56;

T06 = FixMatrix(T06);

x= T06(1,4);
y= T06(2,4);
z= T06(3,4);

%Euler angles Z  Y  Z
R= T06(1:3,1:3);

Beta = atan2( hypot( R(1,3),R(2,3) ) , R(3,3));

%case where sB is == 0
if R(3,3)^2 == 1
   [Alpha,Gamma] = Resolve(R);
else
    Gamma = GetGamma(R,Beta);
    Alpha = GetAlpha(R,Beta);  
    
end

end

function [Alpha,Gamma] = Resolve(R)

    %c? = 1
    if(R(3,3) == 1)
     Alpha=0;
     Gamma = atan2(R(2,1),R(1,1));
    else%(R(3,3) == -1)
     Alpha=0;
     Gamma = -atan2(-R(2,1),-R(1,1));     
    end
       
end
function Alpha = GetAlpha(R,Beta)

    if(MySin(Beta) > 0)
     Alpha= atan2(R(2,3),R(1,3));
    else%(MySin(Beta) < 0)
     Alpha= atan2(-R(2,3),-R(1,3));        
    end
    
end

function Gamma = GetGamma(R,Beta)

    if(MySin(Beta) > 0)
     Gamma= atan2(R(3,2),-R(3,1));
    else%(MySin(Beta) < 0)
     Gamma= atan2(-R(3,2),R(3,1));        
    end

end

function T = GetT(alpha,a,d,o)


T =  Rx(alpha)*Transx(a)*Rz(o)*Transd(d);

end


function m = Rx(alpha)

m= [1 0 0 0; 0 MyCos(alpha) -MySin(alpha) 0; 0 MySin(alpha) MyCos(alpha) 0; 0 0 0 1];

end
function m = Transx(a)

m= [1 0 0 a; 0 1 0 0; 0 0 1 0; 0 0 0 1];

end
function m = Rz(o)

m= [MyCos(o) -MySin(o) 0 0; MySin(o) MyCos(o) 0 0; 0 0 1 0; 0 0 0 1];

end
function m = Transd(d)

m= [1 0 0 0; 0 1 0 0; 0 0 1 d; 0 0 0 1];

end

%problem with zero....
function T = FixMatrix(T06)


for i = 1:4
  for j = 1:4
      if IsZero(T06(i,j))
         T06(i,j)=0;
      end
      T06(i,j) = round(T06(i,j),4);

  end
end

T=T06;


end


function b = MyCos(a)
if( IsZero(cos(a)))
    b=0;
else
    b = cos(a);
end    

end

function b = MySin(a)
if( IsZero(sin(a)))
    b=0;
else
    b = sin(a);
end   
end

function bool = IsZero(value)
bool = ( -0.00001 < value &&  value < 0.00001);
end

