function test_nonlinear_derivation
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'src','plant'));
p=furuta_parameters("example");
rng(411);
maxResidual=0;
maxEnergyResidual=0;
h=1e-5;
for k=1:100
    q=[2*pi*(rand-0.5);2*pi*(rand-0.5)];
    v=4*(rand(2,1)-0.5);
    u=6*(rand-0.5);
    x=[q;v];
    dx=furuta_dynamics(x,u,p);
    acceleration=dx(3:4);
    plus=momentum(q+h*v+0.5*h^2*acceleration,v+h*acceleration,p);
    minus=momentum(q-h*v+0.5*h^2*acceleration,v-h*acceleration,p);
    momentumRate=(plus-minus)/(2*h);
    gradient=zeros(2,1);
    for j=1:2
        shifted=q;
        shifted(j)=shifted(j)+1i*1e-20;
        gradient(j)=imag(lagrangian(shifted,v,p))/1e-20;
    end
    torque=p.eta*p.N*p.Kt*(u-p.Ke*p.N*v(1))/p.R;
    residual=momentumRate-gradient+[p.b0*v(1);p.b1*v(2)]-[torque;0];
    maxResidual=max(maxResidual,norm(residual,inf));
    % Independently differentiate the Cartesian energy along the state flow.
    Hp=energy(q+h*v+0.5*h^2*acceleration,v+h*acceleration,p);
    Hm=energy(q-h*v+0.5*h^2*acceleration,v-h*acceleration,p);
    expected=torque*v(1)-p.b0*v(1)^2-p.b1*v(2)^2;
    maxEnergyResidual=max(maxEnergyResidual,abs((Hp-Hm)/(2*h)-expected));
end
assert(maxResidual<1e-6,'Euler-Lagrange residual is too large.');
assert(maxEnergyResidual<1e-6,'Energy-balance residual is too large.');
fprintf('100 random states passed. Max Euler-Lagrange residual %.3g N*m; energy residual %.3g W.\n',maxResidual,maxEnergyResidual);
end

function value=lagrangian(q,v,p)
[T,V]=energies(q,v,p);
value=T-V;
end

function value=energy(q,v,p)
[T,V]=energies(q,v,p);
value=T+V;
end

function result=momentum(q,v,p)
result=zeros(2,1);
for j=1:2
    shifted=v;
    shifted(j)=shifted(j)+1i*1e-20;
    result(j)=imag(lagrangian(q,shifted,p))/1e-20;
end
end

function [T,V]=energies(q,v,p)
er=[cos(q(1));sin(q(1));0];
et=[-sin(q(1));cos(q(1));0];
ez=[0;0;1];
s=sin(q(2));c=cos(q(2));
n=s*et+c*ez;
velocity=p.L0*v(1)*et+p.l1*(-s*v(1)*er+c*v(2)*et-s*v(2)*ez);
omega=v(1)*ez-v(2)*er;
tensor=p.J1cm*eye(3)+(p.J1ax-p.J1cm)*(n*n.');
T=0.5*p.J0*v(1)^2+0.5*p.m1*(velocity.'*velocity)+0.5*(omega.'*tensor*omega);
V=p.m1*p.g*p.l1*c;
end
