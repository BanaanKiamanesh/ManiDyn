clear
close all
clc

%% Extract the Dynamic Parameters and DH Table
URDFPath = fullfile('urdf\kinova_gen3.urdf');
DynPar = DynStruct(URDFPath);

%% DH Table Generation
DHTable = table(DynPar.DH.alpha(:), DynPar.DH.a(:), DynPar.DH.d(:), ...
    DynPar.DH.theta(:), DynPar.DH.type(:), ...
    'VariableNames', {'alpha', 'a', 'd', 'theta', 'type'});

disp(DynPar)
disp(DHTable)
