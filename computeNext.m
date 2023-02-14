
% position
function q_next = computeNext(q, p_end, i, dt)
Dq = qdot(q, p_end, i);
q_next = q + Dq * dt;
end


% potenziale

function U = potential(p_cur, p_end)
r0 = 0.2;
r = sqrt(p_cur(1).^2 + p_cur(2).^2);
x = p_cur(1);
y = p_cur(2);



if(r > r0)
    U = [0 0]';
else
    alpha = atan2(p_cur(2), p_cur(1));
    if (alpha < 0)
        alpha = 2*pi + alpha;
    end
    theta = alpha + sign(p_end(1) - p_cur(1)) * pi/2;
    ux = cos(theta);
    uy = sin(theta);
    U = [ux uy];
    % U = [ux uy] ./ (sqrt(ux.^2 + uy.^2));
    U = (2*(2 - r/r0) .* U)';   
end

end

% scalar velocity
function v = vScalar(e_k, i)
N0 = 100;
if( i < N0)
    v = norm(e_k) * i ./ N0;
else
    v = norm(e_k);
end

end

% velocity
function v = velocity(i, p_cur, p_end)
lambda = 10;
e_k = p_end - p_cur;
vS = vScalar(e_k, i);
e_norm = norm(e_k);
u = potential(p_cur, p_end);

wz = u(1)*e_k(2) - u(2)*e_k(1);
if ( (p_end(1) > p_cur(1) && wz < 0) || (p_end(1) < p_cur(1) && wz > 0) )
    u = [0 0]';
end

num = e_k ./ e_norm + u;
den = norm(num);

v = lambda .* vS .* (num ./ den);

end

% qdot
function Dq = qdot(q, p_end, i)
J = jacobian(q);
J = J(1:2, :);
J = pinv(J);

[p_cur, ~] = ur5Direct(q);
p_cur = [p_cur(1) p_cur(2)]';
v = velocity(i, p_cur, p_end);
Dq = J * v;
end






