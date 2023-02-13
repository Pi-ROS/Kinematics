
% position
function q_next = computeNext(q, p_end, i, dt)
Dq = qdot(q, p_end, i);
q_next = q + Dq * dt;
end


% potenziale

function U = potential(p_cur, p_end)
r0 = 0.2;

if(sqrt(p_cur(1).^2 + p_cur(2).^2) > r0)
    U = [0 0]';
else
%     p_cur
    m = sign(p_end(1) - p_cur(1));
    x = sqrt(1 ./ (m.^2 +1));
    y = m * x;
    U = [x -y]';
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






