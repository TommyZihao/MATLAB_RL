function out1 = legInvKin(L1,L2,x,y)
%LEGINVKIN
%    OUT1 = LEGINVKIN(L1,L2,X,Y)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    02-Nov-2018 22:21:58

t2 = L1.^2;
t3 = L2.^2;
t4 = x.^2;
t5 = y.^2;
t6 = L1.*L2.*2.0;
t7 = -t2-t3+t4+t5+t6;
t8 = t2+t3-t4-t5+t6;
t9 = t7.*t8;
t10 = sqrt(t9);
t11 = 1.0./t7;
t12 = t2-t3+t4+t5-L1.*y.*2.0;
t13 = 1.0./t12;
t14 = L1.*x.*2.0;
t15 = t4.*t10.*t11;
t16 = t5.*t10.*t11;
t17 = L1.*L2.*t10.*t11.*2.0;
t18 = t10.*t11;
t19 = atan(t18);
out1 = reshape([atan(t13.*(t14+t15+t16+t17-t2.*t10.*t11-t3.*t10.*t11)).*2.0,atan(t13.*(t14-t15-t16-t17+t2.*t10.*t11+t3.*t10.*t11)).*2.0,t19.*-2.0,t19.*2.0],[2,2]);
