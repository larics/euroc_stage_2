%% initializations

% initial / final conditions
p_0 = 0;
p_T = 5;
v_0 = 0;
v_T = 0;
a_0 = 0;
a_T = 0;
j_0 = 0;
j_T = 0;
s_0 = 0;
s_T = 0;



v_max = 2;
a_max = 2;
j_max = 50; % jerk
s_max = 100; % snap

T = abs(p_T-p_0)*2/v_max;

N = 12;
dt = 0.01;

%% symbolic stuff

if(true)
    n = N;
    syms t scale real;
    
%     c = sym('c', [1,n]);
%     c = sym(c, 'real');
%     
%     p = [1 c]*(t.^(n:-1:0))';
    
    c = sym('c', [1,n]);
    c = sym(c, 'real');
    
    p = c*(t.^(n-1:-1:0))';
    p_a = c*((t*scale).^(n-1:-1:0))';
    
%     p = c*(exp(t*(ones(1,n)*5 + (0.1:0.1:n*0.1))'));
    
    v = diff(p, t);
    a = diff(v, t);
    j = diff(a, t);
    s = diff(j, t);
    
    cost = int(a^2, t);
    
    H = jacobian(cost, c);
    H = jacobian(H, c);
    
    A_eq = [ ...
        subs(p, t, 0); p; ...
        subs(v, t, 0); v; ...
        subs(a, t, 0); a; ...
        subs(j, t, 0); j; ...
        subs(s, t, 0); s ...
        ];
    
    A_eq = jacobian(A_eq, c);
    
    n_phi = 8;
    c_phi = sym('c_phi', [1,n_phi]);
    c_phi = sym(c_phi, 'real');
    
    phi = c_phi*(t.^(n_phi-1:-1:0))';
    om = diff(phi, t);
    om_d = diff(om, t);
    
    cost_phi = int(om_d^2, t);
    
    H_phi = jacobian(cost_phi, c_phi);
    H_phi = jacobian(H_phi, c_phi);
end



%% test

% T = 1;

H_ = subs(H, t, T);
A_eq_ = subs(A_eq, t, T);


b_eq = [ ...
        p_0 ; ...
        p_T ; ...
        v_0 ; ...
        v_T ; ...
        a_0 ; ...
        a_T ; ...
        j_0 ; ...
        j_T ; ...
        s_0 ; ... 
        s_T ;...
        ];
    
% b_eq(1) = b_eq(1)-p_0;    
% b_eq = b_eq / (p_T - p_0);   


constr = [1 2 3 4 5 6 7 8 9 10];
% constr = [1 2 3  5  7 8 9 10];
 
% just testting, not used right now ...
tic 
x_opt = quadprog(H_, [], [], [], A_eq_(constr, :), b_eq(constr));
toc

Q = [H_ A_eq_(constr, :)'; A_eq_(constr, :) zeros(length(constr))];
x = pinv(Q, 1e-12)*[zeros(N,1); b_eq(constr)];
x = x(1:N);

norm(x-x_opt)



t = 0:dt:T;




vn = (p_T-p_0)*eval(subs(v, c, x));
an = (p_T-p_0)*eval(subs(a, c, x));


scale_v = v_max / max(abs(vn));
scale_a = a_max / max(abs(an));
scale = min([scale_a scale_v]);

scale = 1;

sv = (scale.^(n-1:-1:0))';
x = x.*sv;
t = t/scale;

p_T = 1; p_0=0;

pn = p_0 + (p_T-p_0)*eval(subs(p, c, x));
vn = (p_T-p_0)*eval(subs(v, c, x)) ;
an = (p_T-p_0)*eval(subs(a, c, x)) ;
jn = (p_T-p_0)*eval(subs(j, c, x)) ;
sn = (p_T-p_0)*eval(subs(s, c, x)) ;



% x=x_opt;

% x =[4.32391e-14 -7.54034e-13  1.40006e-13 -1.36603e-14  -1.1577e-14       0.2352    -0.214171    0.0896914   -0.0217234   0.00313783 -0.000251026  8.55771e-06]';
% x = flipud(x);

%% plot
figure(25)
clf

dt_s = dt/scale;

% p_0=0;p_T=1;



subplot 511
hold on
plot(t, pn)
grid on
xlabel('t[s]')
ylabel('pos [m]')

subplot 512
hold on
plot(t, vn)
plot(t(2:end), diff(pn)/dt_s, 'r')
grid on
xlabel('t[s]')
ylabel('vel [m/s]')

subplot 513
hold on
plot(t, an)
plot(t(3:end), diff(pn,2)/dt_s^2, 'r')
grid on
xlabel('t[s]')
ylabel('acc [m/s^2]')

subplot 514
hold on
plot(t, jn)
plot(t(4:end), diff(pn, 3)/dt_s^3, 'r')
grid on
xlabel('t[s]')
ylabel('jerk [m/s^3]')

subplot 515
hold on
plot(t, sn)
plot(t(5:end), diff(pn,4)/dt_s^4, 'r')
grid on
xlabel('t[s]')
ylabel('snap [m/s^4]')
