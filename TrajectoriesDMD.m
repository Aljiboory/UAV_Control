function [xd, yd, zd, psid, xd_p, yd_p, zd_p, psid_p] = TrajectoriesDMD(n,t)

mul=2;
% 2) Selection of desired trajectory (POSITION and VELOCITIES)
     switch n
   % a)Churo Trajectory     
        case 1   
            xd= 0.06 *t.* cos(0.2*t) ;   
            xd_p= 0.06 * cos(0.2*t) + 0.02 *0.2 *t.* (-sin(0.2*t));
            xd_pp= -0.06 * sin(0.2*t) + 0.02 *0.2 * (-sin(0.2*t)) - 0.02 * 0.2 * 0.2 * t.* cos(0.2*t);
            yd= 0.06 *t.* sin (0.2 * t);  
            yd_p= 0.06 * sin(0.2*t) + 0.02 *0.2*t.* cos(0.2*t);
            yd_pp= 0.06 * 0.2 * cos(0.2*t) + 0.02 *0.2* cos(0.2*t) - 0.02 * 0.2 * 0.2 *t.* sin(0.2*t);
            zd= 1 * sin (0.3 * t) + 10 ;
            zd_p= 1 * 0.3* cos(0.3*t);
            zd_pp= -1 * 0.3 * 0.3 * sin(0.3*t); 
   % b) Sine Trajectory    
        case 2 
            yd= 4 * sin(0.2*t) + 1;           yd_p= 4*0.2 * cos(0.2*t);         yd_pp= -4* 0.2* 0.2* sin(0.2*t);
            xd= 0.2*t;                        xd_p= 0.2* ones(1,length(t));     xd_pp= 0* ones(1,length(t));
            zd= 2 * ones(1,length(t)) +6;     zd_p= 0 * ones(1,length(t)); 
   % c) Figure-eight Trajectory  
        case 3
            xd = 5 * sin(mul*0.04*t)+0.1;         xd_p = 5*mul*0.04*cos(mul*0.04*t);     xd_pp = -5*mul*mul*0.04*0.04*sin(mul*0.04*t);
            yd = 5 * sin(mul*0.08*t)+0.1;         yd_p = 5*mul*0.08*cos(mul*0.08*t);     yd_pp = -5*mul*mul*0.08*0.08*sin(mul*0.08*t);               
            zd = 1 * sin (0.08 * t) +2 ;          zd_p = 0.08*cos(0.08*t);
   % d) Saddle Trajectory
        case 4  
            xd= 0.0*ones(1, length(t));                xd_p= 1.0*ones(1, length(t));           xd_pp=0.0*ones(1, length(t));
            yd= 0*ones(1, length(t)) ;                yd_p=1.0*ones(1, length(t));            yd_pp=0.0*ones(1, length(t));
            zd= 0.0*ones(1, length(t));             zd_p=0.0*ones(1, length(t));              %zd_pp=-0.09*sin(0.3*t);
            
   % e) Otra opcion
        otherwise
            disp("None of the above");
     end

% 3) Orientation Calculation 
     psid= (atan2(yd_p,xd_p));
     psid_p = (1./((yd_p./xd_p).^2+1)).*((yd_pp.*xd_p-yd_p.*xd_pp)./xd_p.^2);
     
%      psid= 0.0*ones(1, length(t));
%      psid_p = 0.0*ones(1, length(t));
     psid_p(1)=0;
end