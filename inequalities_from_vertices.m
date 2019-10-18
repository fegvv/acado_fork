% Description: Script that uses the MPT toolbox to go from a vertex
% representation of a polytope to a matrix inequality representation that
% can be input as a set of linear constraints in the acado toolbox
clear;close all;

% init mpt toolbox and yalmip (edit to MPT dir)
disp('initializing yalmip and MPT')
addpath(genpath('/home/larsvens/Programs/multiparametrictoolbox'));
mpt_init;

%% define the polytope
n=12;
Fxf_max = 1000;
Fyf_max = 1000;

t_vertices = linspace(-pi,pi,n+1); 
Fxf_vertices = Fxf_max*cos(t_vertices);
Fyf_vertices = Fyf_max*sin(t_vertices);
P = Polyhedron([Fxf_vertices(1:n)' Fyf_vertices(1:n)']);

figure
plot(P)
xlabel('Fxf')
ylabel('Fyf')
axis('equal')

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

%% test scaling
scalefactor = 1000;
Ptest = Polyhedron([a b],scalefactor*c);
figure
plot(Ptest)
xlabel('Fxf')
ylabel('Fyf')
axis('equal')

%% helper functions

function printvector(v)
    s = '{';
    for i = 1:length(v)
        s = [s num2str(v(i)) ', '  ];
    end
    s = [s '}'];
    disp(s)
end



