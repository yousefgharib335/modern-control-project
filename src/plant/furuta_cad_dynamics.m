function [dx,energy] = furuta_cad_dynamics(x,torque,p)
% General two-joint rigid-body model matching the CAD-group Simscape plant.
% State order is arm angle, upright pendulum angle, and both angular rates.
q=x(2); c=cos(q); s=sin(q);
rotation=[1 0 0;0 c s;0 -s c];
axis=[-1;0;0]; armAxis=[0;0;1];
worldUp=p.mountRotation.'*armAxis;
generator=[0 0 0;0 0 1;0 -1 0];
relativeCOM=rotation*p.pendulumCOMArmZero;
position=p.hingeTranslation+relativeCOM;
inertia=rotation*p.pendulumInertiaArmZero*rotation.';
positionDerivative=cross(axis,relativeCOM);
inertiaDerivative=generator*inertia-inertia*generator;
v0=cross(armAxis,position);
v1=cross(axis,relativeCOM);
dv0=cross(armAxis,positionDerivative);
dv1=cross(axis,positionDerivative);
m=p.pendulumMass;
M11=p.armInertia+m*(v0.'*v0)+armAxis.'*inertia*armAxis;
M12=m*(v0.'*v1)+armAxis.'*inertia*axis;
M22=m*(v1.'*v1)+axis.'*inertia*axis;
mass=[M11 M12;M12 M22];
dM11=2*m*(v0.'*dv0)+armAxis.'*inertiaDerivative*armAxis;
dM12=m*(dv0.'*v1+v0.'*dv1)+armAxis.'*inertiaDerivative*axis;
dM22=2*m*(v1.'*dv1)+axis.'*inertiaDerivative*axis;
velocity=[dM11*x(3)*x(4)+dM12*x(4)^2; ...
    -0.5*dM11*x(3)^2+0.5*dM22*x(4)^2];
gravity=[0;m*p.gravity*(worldUp.'*positionDerivative)];
friction=[p.armDamping*x(3);p.pendulumDamping*x(4)];
acceleration=mass\([torque;0]-velocity-gravity-friction);
dx=[x(3:4);acceleration];
energy=0.5*x(3:4).'*mass*x(3:4)+m*p.gravity*(worldUp.'*position);
end
