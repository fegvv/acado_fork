% Description: Script that uses the MPT toolbox to go from a vertex
% representation of a polytope to a matrix inequality representation that
% can be input as a set of linear constraints in the acado toolbox

% init mpt toolbox and yalmip (edit to MPT dir)
disp('initializing yalmip and MPT')
addpath(genpath('/home/larsvens/Programs/multiparametrictoolbox'));
mpt_init;

%% define the polytope
n=8;
Fxf_max = 10000;
Fyf_max = 5000;

t_vertices = linspace(-pi,pi,n+1); 
Fx_vertices = Fxf_max*cos(t_vertices);
Fy_vertices = Fyf_max*sin(t_vertices);
P = Polyhedron([Fx_vertices(1:n)' Fy_vertices(1:n)']);

plot(P)
xlabel('Fx')
ylabel('Fyf')

% print a,b,c
a = P.A(:,1);
b = P.A(:,2);
c = P.b;

disp('')
disp('a = ')
printvector(a)
disp('b = ')
printvector(b)
disp('c = ')
printvector(c)

function printvector(v)
    s = '{';
    for i = 1:length(v)
        s = [s num2str(v(i)) ', '  ];
        
    end
    s = [s '}'];
    disp(s)
end

