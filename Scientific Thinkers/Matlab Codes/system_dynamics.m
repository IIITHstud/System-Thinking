function [dydt] = system_dynamics(t, y, m1, m2, l1, l2, g, KP1, KP2, KD1, KD2, KI1, KI2, q1f, q2f)
    q1 = y(1);
    dq1 = y(2);
    q2 = y(3);
    dq2 = y(4);
    int_e1=y(5);
    int_e2=y(6);
    
    e1 = q1f-q1;
    e2 = q2f-q2;

    %{
    %{
    persistent integral1 integral2
    
    if isempty(integral1)
        integral1 = 0;
    end
    
    if isempty(integral2)
        integral2 = 0;
    end
    %}
    integral1 = cumtrapz(t, e1);  % Integrate e1 over time
    integral2 = cumtrapz(t, e2);  % Integrate e2 over time
    integral1 = integral1 + e1;
    integral2 = integral2 + e2;
    %}
%{
    u1 = KP1 * e1 - KD1 * dq1 + KI1 * integral1;
    u2 = KP2 * e2 - KD2 * dq2 + KI2 * integral2; 
%}
    ans1=KP1 * e1 - KD1 * dq1 + KI1 *(int_e1);
    ans2=KP2 * e2 - KD2 * dq2 + KI2 *(int_e2);
    
    M11 = (m1 + m2) * (l1*l1) + m2 * l2*(l2 + 2 * l1 * cos(q2));
    M22 = m2 * l2*l2;
    M12=m2*l2*(l2+l1*cos(q2));
    M21=M12;
    
    C11 = -m2 * l1 * l2 * sin(q2) * dq2;
    C12 = -m2 * l1 * l2 * sin(q2) * (dq1 + dq2);
    C21 = 0;  
    C22 = m2 * l1 * l2 * sin(q2) * dq2;
    
    G1 = (m1 * l1 * g * cos(q1) + m2 * g * (l1 * cos(q1) + l2 * cos(q1 + q2)));
    G2 = m2 * l2 * cos(q1 + q2) * g;

    M=[M11,M12;M12,M22];
    T=[M]*[ans1;ans2];
    C=[C11,C12;C21,C22];
    G=[G1;G2];
    dq=[dq1;dq2];
    
    M_inverse=inv(M);
    Q=(M_inverse)*(T-C*dq-G);
    ddq1=Q(1);
    ddq2=Q(2);
    
   % ddq1 = (1/(M21*M12-M11*M22))*(M21*(u1-C11*dq1-C12*dq2-G1)-M11*(u2-C22*dq2-G2));
   % ddq2 = ((1/M22*M11-M12*M21))*(M22*(u1-C11*dq1-C12*dq2-G1)-M12*(u2-C22*dq2-G2));
    
    dydt = [dq1; ddq1;dq2;ddq2;e1;e2];
end
