function p = furuta_parameters(source)
% Fill the measured column and record how each value was obtained.
% The example column is for simulation and software checks only.
arguments
    source (1,1) string {mustBeMember(source,["example","measured"])} = "example"
end
names = ["L0","l1","m1","J0","J1cm","J1ax","b0","b1","R","Kt","Ke","N","eta","g"];
% Parameter        Example     Measured   Unit and measurement method
values = [
    0.20           NaN       % L0, m. Motor axis to pendulum pivot, caliper/ruler.
    0.14           0.04537       % l1, m. User-supplied CAD estimate: 113.90 mm minus 68.53 mm.
    0.11           0.01500       % m1, kg. SolidWorks CAD mass of ALLEN_04_mm (Alloy Steel), 15.00 g.
    0.007          NaN       % J0, kg*m^2. Arm and reflected drivetrain inertia about vertical axis, CAD or identification.
    0.0010         NaN       % J1cm, kg*m^2. Pendulum transverse inertia about COM, CAD or identification.
    0.00001        NaN       % J1ax, kg*m^2. Pendulum axial inertia about its long axis, CAD or identification.
    0.0012         NaN       % b0, N*m*s/rad. Arm joint viscous friction excluding electrical damping, fit to coast-down.
    0.0004         NaN       % b1, N*m*s/rad. Pendulum pivot viscous friction, fit to free decay.
    4.0            NaN       % R, ohm. Motor terminal resistance, datasheet or meter.
    0.035          NaN       % Kt, N*m/A. Motor torque constant, datasheet.
    0.035          NaN       % Ke, V*s/rad. Motor back-EMF constant, datasheet.
    5.0            NaN       % N, motor speed / arm speed. Gear ratio, datasheet or count.
    0.85           NaN       % eta, unitless. Gear efficiency, datasheet or identify.
    9.81           9.81      % g, m/s^2. Standard approximation for gravity.
];
v = values(:,1);
if source == "measured"
    v = values(:,2);
end
missing = names(~isfinite(v));
assert(isempty(missing),'FurutaPlant:MissingParameters', ...
    'Enter values for: %s',char(strjoin(missing,', ')));
assert(all(v([1:5 9:14]) > 0) && all(v(6:8) >= 0) && v(13) <= 1, ...
    'FurutaPlant:InvalidParameters','Check units and physical parameter bounds.');
p = cell2struct(num2cell(v),cellstr(names(:)),1);
end
