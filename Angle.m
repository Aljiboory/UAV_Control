function [ErrAng] = Angle(ErrAng)
% Function "Angulo" that restricts the angle to [0 : pi] and [-pi : 0]

% 1) INPUT ARGUMENTS
% a) ErrAng ----> angle in radians

% 2) OUTPUT ARGUMENTS
% a) ErrAng ----> input angle limited to the range [0 : pi] and [-pi : 0]

if ErrAng>=1.00*pi
    while ErrAng>=1*pi
    ErrAng=ErrAng-2*pi;
    end
    return
end

if ErrAng<=-1.00*pi
    while ErrAng<=-1*pi
    ErrAng=ErrAng+2*pi;
    end
    return
end    
    ErrAng=ErrAng;
return

