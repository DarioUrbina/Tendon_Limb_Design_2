function J = Jinv(q_1, q_2, l1, l2,i)
degs=[q_1, q_2];
syms q1 q2

A1 = [ cos(q1) -sin(q1) 0 0;
        sin(q1) cos(q1) 0 0;
        0       0       1   0;
        0       0       0   1 ];
    
A2 = [ cos(q2) -sin(q2) 0 0;
        sin(q2) cos(q2) 0 -l1;
        0       0       1   0;
        0       0       0   1 ];

A3 = [ 1 0 0 0 ;
        0 1 0 -l2;
        0 0 1 0 ;
        0 0 0 1 ] ;

T0_1 = A1;
T0_2 = A1 * A2;
T0_3 = T0_2 * A3;

r = T0_3;
g = T0_3(1:2,4);
J = jacobian(g,[q1 q2]);
if (i==1)
    J = simplify(J);
end
J = simplify(J);

% q1 = degtorad(q_1); q2 = degtorad(q_2);
q1 = q_1; q2 = q_2;

J_sim = double(subs(J));  %Simplifies the representation of the Jacobian, substitutes the symbolic values with
                                    %asigned values and creates a duble
                                    %precision arrray with that information

J = inv(J_sim);                    %Get
end
