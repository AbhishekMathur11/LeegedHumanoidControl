function [M,h,Jcm,Jfp,dJcmxdq,dJfpxdq] = QP_ComputeKinAndDynTerms(q,dq)
%QP_COMPUTEKINANDDYNTERMS
%    [M,H,JCM,JFP,DJCMXDQ,DJFPXDQ] = QP_COMPUTEKINANDDYNTERMS(Q1,Q2,Q3,Q4,Q5,DQ1,DQ2,DQ3,DQ4,DQ5)


% 
% This function was automatically generated from symbolic expressions by the command matlabFunction.
% The symbolic expressions are derived in the script HUMANOIDSYMBOLICMATH. The model parameters in
% these epressions are converted into numerical values based on the values given in the script
% DEFINEHUMANOIDPARAMS. Finally, the resulting functions of only q and dq are converted into this
% Matlab function QP_COMPUTEKINANDDYNTERMS.
% 
% NOTE: If the humanoid parameters are changed, this function becomes invalid, and it has to be
% generated anew following the outlined procedure.
% 
% In:
% q1,q2,q3,q4,q5 : joint angles
% dq1,dq2,dq3,dq4,dq5: joint angular velocities
% 
% Out:
% M : Humanoid mass matrix M(q)
% h : Coriolis and gravitational terms h(q,dq)=C(q,dq)*dq+N(q)
% Jcm : CoM Jacobian Jcm
% Jfp : Foot Point (FP) Jacobian Jfp
% dJcmxdq : Vector dJcm*dq meeded in QP constraint equation
% dJfpxdq : Vector dJfp*dq meeded in QP cost equation

%% assign individual angles and angular velocities from vector inputs
q1 = q(1); q2 = q(2);  q3= q(3);  q4= q(4);  q5= q(5);
dq1=dq(1); dq2=dq(2); dq3=dq(3); dq4=dq(4); dq5=dq(5);
 

%% Code below was automatically generated by command matlabFunction
% -----------------------------------------------------------------
 
t2 = q2.*-1.0;
t3 = q1+t2;
t4 = cos(t3);
t5 = t4.*5.44464;
t6 = q3.*-1.0;
t7 = q4.*-1.0;
t8 = q5.*-1.0;
t9 = q1+t6;
t10 = cos(t9);
t11 = t10.*1.6;
t12 = q2+t6;
t13 = cos(t12);
t14 = t13.*1.6;
t15 = q1+t7;
t16 = cos(t15);
t17 = t16.*9.5536e-1;
t18 = q2+t7;
t19 = cos(t18);
t20 = t19.*9.5536e-1;
t21 = q1+t8;
t22 = cos(t21);
t23 = t22.*1.6384e-1;
t24 = q2+t8;
t25 = cos(t24);
t26 = t25.*1.6384e-1;
t27 = q4+t8;
t28 = cos(t27);
t29 = t28.*1.6384e-1;
M = reshape([7.0547488,t5,t11,t17,t23,t5,6.2619492,t14,t20,t26,t11,t14,3.02,0.0,0.0,t17,t20,0.0,1.7726692,t29,t23,t26,0.0,t29,9.824288e-1],[5,5]);
if nargout > 1
    t30 = sin(t3);
    t31 = dq3.^2;
    t32 = dq4.^2;
    t33 = dq5.^2;
    t34 = dq1.^2;
    t35 = sin(t9);
    t36 = dq2.^2;
    t37 = sin(t12);
    t38 = sin(t15);
    t39 = sin(t18);
    t40 = sin(t21);
    t41 = sin(t24);
    t42 = sin(t27);
    t43 = cos(q1);
    t44 = cos(q2);
    t45 = cos(q3);
    t46 = cos(q4);
    t47 = cos(q5);
    h = [t43.*1.55904e1+t30.*t36.*5.44464+t31.*t35.*1.6+t32.*t38.*9.5536e-1+t33.*t40.*1.6384e-1;t44.*1.36116e1-t30.*t34.*5.44464+t31.*t37.*1.6+t32.*t39.*9.5536e-1+t33.*t41.*1.6384e-1;t45.*4.0-t34.*t35.*1.6-t36.*t37.*1.6;t46.*2.3884-t34.*t38.*9.5536e-1+t33.*t42.*1.6384e-1-t36.*t39.*9.5536e-1;t47.*4.096e-1-t32.*t42.*1.6384e-1-t34.*t40.*1.6384e-1-t36.*t41.*1.6384e-1];
end
if nargout > 2
    t48 = sin(q1);
    t49 = sin(q2);
    t50 = sin(q4);
    t51 = sin(q5);
    t52 = sin(q3);
    Jcm = reshape([t48.*-3.8976e-1,t43.*3.8976e-1,t49.*-3.4029e-1,t44.*3.4029e-1,t52.*-1.0e-1,t45.*1.0e-1,t50.*-5.971e-2,t46.*5.971e-2,t51.*-1.024e-2,t47.*1.024e-2],[2,5]);
end
if nargout > 3
    Jfp = reshape([t48.*-4.0e-1,t43.*4.0e-1,t49.*-4.0e-1,t44.*4.0e-1,0.0,0.0,t50.*-4.0e-1,t46.*4.0e-1,t51.*-4.0e-1,t47.*4.0e-1],[2,5]);
end
if nargout > 4
    dJcmxdq = [t31.*t45.*-1.0e-1-t34.*t43.*3.8976e-1-t32.*t46.*5.971e-2-t33.*t47.*1.024e-2-t36.*t44.*3.4029e-1;t32.*t50.*-5.971e-2-t34.*t48.*3.8976e-1-t31.*t52.*1.0e-1-t33.*t51.*1.024e-2-t36.*t49.*3.4029e-1];
end
if nargout > 5
    dJfpxdq = [t34.*t43.*-4.0e-1-t32.*t46.*4.0e-1-t33.*t47.*4.0e-1-t36.*t44.*4.0e-1;t32.*t50.*-4.0e-1-t34.*t48.*4.0e-1-t33.*t51.*4.0e-1-t36.*t49.*4.0e-1];
end
