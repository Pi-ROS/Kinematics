function [q_des, iter] = LM_method(x_des, q0)

max_iter = 5000;
tresh = 0.001;
lambda = 1;
iter = 1;
q_k = q0;
[p, ~] = ur5Direct(q0);
res = [p.'];
while (iter<=max_iter)
    e_k = err(x_des, fk(q_k));
    e_k_norm = norm( e_k );
    if(e_k_norm<=tresh)
        break
    end
    [A_k_inv, g_k] = params(q_k, e_k, e_k_norm, lambda);
    q_k = q_k + A_k_inv * g_k;
%     J = ur5Jac(q_k);
%     q_k = q_k + inv(J.' * J) * g_k;

    [p, ~] = ur5Direct(q_k);
    res = [res; p.'];
    iter = iter + 1;
end

q_des = q_k;

% plot
figure(1);
x =[]; y = []; z = [];
for i = 1:iter
    rTmp = res(i,:);
    x = cat(2, x, rTmp(1));
    y = cat(2, y, rTmp(2));
    z = cat(2, z, rTmp(3));

end
plot3(x,y,z, 'r*-');
xlabel('x');
ylabel('y');
axis([-10 10 -10 10]);

end

% Utility functions

function [A_inv, g_k] = params(q_k, e_k, e_k_norm, lambda)
J = ur5Jac(q_k);
Jt = J.';
E = e_k_norm .* 0.5;
Wdamp = lambda .* E .* eye(6,6);
Atmp = Jt * J + Wdamp;
A_inv = inv(Atmp);
g_k = Jt * e_k;
end

function t = tau(x)
t = x(1:3,4);
end

function r = ro(x)
r = x(1:3,1:3);
end

function [e] = err(T_des, T_0)
    t = tau(T_des) - tau(T_0);
    r = ro(T_des) * ro(T_0.');

    % alpha - a_r
    l = [r(3,2) - r(2,3); r(1,3) - r(3,1); r(2,1) - r(1,2)];
    l_norm = norm(l);
    if (l == 0)
        if(r(1,1) == 1 && r(2,2) == 1 && r(3,3) == 1)
            a_r = [0;0;0];
        else
            a_r = pi/2 .* [r(1,1) + 1; r(2,2) + 1; r(3,3) +1];
        end
    else
        tmp = atan2(l_norm, r(1,1) + r(2,2) + r(3,3) -1) ./ l_norm;
        a_r = tmp .* l;
    end
    % ---

    e = [t; a_r];

end