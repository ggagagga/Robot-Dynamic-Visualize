%{
***********************************************************************
**************         3D Printing Research Group        **************
***********************************************************************
************            Principal Invetigator (PI):        ************
*********               >>>   Dr. Yiwei Weng   <<<            *********
***********************************************************************
***               The Hong Kong Polytechnic University             ****
***              Department of Building and Real Estate            ****
***                         Hong Kong (PRC)                        ****
***********************************************************************
*---------------------------------------------------------------------*
*                 Lab Website: wengyiwei.github.io                    *
*---------------------------------------------------------------------*
*                                                                     *
***********************************************************************
****    Class for Graphical User Interface (GUI) in Robot Arm      ****
***********************************************************************
*---------------------------------------------------------------------*
*                   Start date:    July 29 2024                       *
*                   Last update:   July 29 2024                       *
*---------------------------------------------------------------------*
*                                                                     *
***********************************************************************
_______________________________________________________________________

                                                          Copyright (C)
                                                           2024-present
                                                            by LIU Tong
                                                              Hong Kong
                                             People's Republic of China
_______________________________________________________________________
%}
classdef ROBRTB
  
  methods(Static)
     % version 1 to build robot
    function Rtb = BuildRobot(Xval)
      deg = pi / 180 ; %
      % Define Link1
      m1 = Xval(1, 1) ; r1 = Xval(2:4, 1) ;       
      % [Ixx Ixy Ixz Iyy Iyz Izz]
      Iv1 = Xval(5:10, 1) ;
      I1 = ROBRTB.ConvertToSymmetric(Iv1) ;
      Jm1 = Xval(11, 1) ;  G1 = Xval(12, 1) ; B1 = Xval(13, 1) ; Tc1 = Xval(14:15, 1)  ;
      L(1) = Link('revolute','alpha',0,'a',0,'offset',0,'d', 0.445,'modified',...
        'm', m1, 'r', r1 , 'I', I1, 'Jm', Jm1, 'G', G1, 'B', B1, 'Tc', Tc1, ...
        'qlim', [-180 180]*deg );
      % Define Link2
      m2 = Xval(1, 2) ; r2 =  Xval(2:4,2) ; 
      Iv2 = Xval(5:10, 2) ;
      I2 = ROBRTB.ConvertToSymmetric(Iv2) ; 
      Jm2 = Xval(11, 2) ;  G2 =  Xval(12, 2) ; B2 = Xval(13, 2) ; Tc2 = Xval(14:15, 2)  ;
      L(2) = Link('revolute','alpha',-pi/2,'a',0.150,'offset',-pi/2,'d',0,'modified',...
        'm', m2, 'r', r2, 'I', I2 , 'Jm', Jm2, 'G', G2,'B', B2, 'Tc', Tc2,...
        'qlim', [-95 155]*deg ) ;
      m3 = Xval(1, 3); r3 = Xval(2:4,3) ;  Iv3 = Xval(5:10, 3) ;       
      Im3 = ROBRTB.ConvertToSymmetric(Iv3) ;
      Jm3 = Xval(11, 3) ; G3 = Xval(12, 3) ; B3 = Xval(13, 3) ;
       Tc3 = Xval(14:15,3)  ;
      L(3) = Link('revolute','alpha',0,'a',0.700,'offset',0,'d',0,'modified',...
        'm', m3, 'r', r3,  'I', Im3, 'Jm', Jm3, 'G', G3, 'B', B3, 'Tc', Tc3,...
        'qlim', [-180 75]*deg ) ;
      m4 = Xval(1, 4); r4 = Xval(2:4, 4) ;
      Iv4 =  Xval(5:10, 4) ;     Im4 = ROBRTB.ConvertToSymmetric(Iv4) ; 
      
      Jm4 =  Xval(11, 4) ;  G4 = Xval(12, 4) ; B4 = Xval(13, 4) ; 
        Tc4 = Xval([14 15], 4) ; 
      L(4) = Link('revolute','alpha',-pi/2,'a',0.115,'offset',0,'d',0.795,'modified',...
        'm', m4, 'r', r4,  'I', Im4, 'Jm', Jm4, 'G', G4, 'B',B4 , 'Tc', Tc4, ...         
        'qlim', [-400 400]*deg) ;
      m5 = Xval(1, 5) ;  r5 = Xval(2:4, 5) ;
      Iv5 = Xval(5:10, 5) ; Im5 = ROBRTB.ConvertToSymmetric(Iv5) ;       
       Jm5 = Xval(11, 5) ; G5 = Xval(12, 5)  ; B5 = Xval(13, 5) ; 
       Tc5 =Xval(14:15, 5) ;
      L(5) = Link('revolute','alpha',pi/2,'a',0,'offset',-pi,'d',0,'modified',...
        'm', m5, 'r', r5, 'I', Im5, 'Jm', Jm5, 'G', G5, 'B', B5, 'Tc', Tc5, ...
        'qlim', [-120 120]*deg ) ;
      m6 = Xval(1, 6) ;   r6 = Xval(2:4, 6) ; 
      Iv6 = Xval(5:10, 6) ; Im6 = ROBRTB.ConvertToSymmetric(Iv6) ;       
      Jm6 = Xval(11, 6) ; G6 = Xval(12, 6) ; B6 = Xval(13, 6) ;
      Tc6 = Xval(14:15, 6);
      L(6) = Link('revolute','alpha',pi/2,'a',0,'offset',0,'d',0.85,'modified',...
        'm', m6, 'r', r6, 'I', Im6, 'Jm', Jm6, 'G', G6, 'B', B6, 'Tc', Tc6, ...          
        'qlim', [-400 400]*deg ) ;
      Rtb = SerialLink(L,'name','ABB2600','comment','LL');
    end
    
    
    
    function InertiaMatG = ConvertToSymmetric(InertiaVector)
      % extract the values inside
      Ixx = InertiaVector(1) ;
      Ixy = InertiaVector(2) ;
      Ixz = InertiaVector(3) ;
      Iyy = InertiaVector(4) ;
      Iyz = InertiaVector(5) ;
      Izz = InertiaVector(6) ;
      % generate the symmetric matrix
      InertiaMatG = [Ixx, Ixy, Ixz ;
        Ixy, Iyy, Iyz ;
        Ixz, Iyz, Izz  ] ;
    end
    % objective function to be optimized
    function Eqs = Objfun(Xval)
      % Rtb = ROBRTB.BuildRobot(Xval) ;
      Rtb = ROBRTB.BuildRobotII(Xval) ; % second version to optimize
      filename = ROBGUI.GETSaveFileName ;
      load(filename) ;
      dt = 0.1 ;
      Difff = ROBGUI.CDF_Int(JointP(:,1)) ;
      Nump = size(JointP, 1) ;
      Time = linspace(1,Nump,Nump) *  dt ;
      deg = pi / 180 ;
      Q = JointP * deg ;
      Qd = Difff * Q /dt ;
      Qdd = Difff * Qd /dt ;
      Tau = Rtb.rne(Q,Qd, Qdd) ;
      % Eqs = sqrt(sum( (Tau(:,1)-JointT(:,1)).^2)/(Nump-1)) ;
      Eqs = sqrt(sum(sum((Tau - JointT).^2 /(6*Nump-1))));
    end
    


     % version 1 to build robot
    function Rtb = BuildRobotII(Xval)
      deg = pi / 180 ; %
      % Define Link1
      m1 = Xval(1, 1) ; r1 = Xval(2:4, 1) ;       
      % [Ixx Ixy Ixz Iyy Iyz Izz]
      Iv1 = Xval(5:10, 1) ;
      I1 = ROBRTB.ConvertToSymmetric(Iv1) ;
      Jm1 = Xval(11, 1) ;  G1 = 122 ; B1 = Xval(12, 1) ; Tc1 = Xval(13:14, 1)  ;
      L(1) = Link('revolute','alpha',0,'a',0,'offset',0,'d', 0.445,'modified',...
        'm', m1, 'r', r1 , 'I', I1, 'Jm', Jm1, 'G', G1, 'B', B1, 'Tc', Tc1, ...
        'qlim', [-180 180]*deg );
      % Define Link2
      m2 = Xval(1, 2) ; r2 =  Xval(2:4,2) ; 
      Iv2 = Xval(5:10, 2) ;
      I2 = ROBRTB.ConvertToSymmetric(Iv2) ; 
      Jm2 = Xval(11, 2) ;  G2 =  148 ; B2 = Xval(12, 2) ; Tc2 = Xval(13:14, 2)  ;
      L(2) = Link('revolute','alpha',-pi/2,'a',0.150,'offset',-pi/2,'d',0,'modified',...
        'm', m2, 'r', r2, 'I', I2 , 'Jm', Jm2, 'G', G2,'B', B2, 'Tc', Tc2,...
        'qlim', [-95 155]*deg ) ;
      m3 = Xval(1, 3); r3 = Xval(2:4,3) ;  Iv3 = Xval(5:10, 3) ;       
      Im3 = ROBRTB.ConvertToSymmetric(Iv3) ;
      Jm3 = Xval(11, 3) ; G3 = -125; B3 = Xval(12, 3) ;
       Tc3 = Xval(13:14,3)  ;
      L(3) = Link('revolute','alpha',0,'a',0.700,'offset',0,'d',0,'modified',...
        'm', m3, 'r', r3,  'I', Im3, 'Jm', Jm3, 'G', G3, 'B', B3, 'Tc', Tc3,...
        'qlim', [-180 75]*deg ) ;
      m4 = Xval(1, 4); r4 = Xval(2:4, 4) ;
      Iv4 =  Xval(5:10, 4) ;     Im4 = ROBRTB.ConvertToSymmetric(Iv4) ; 
      
      Jm4 =  Xval(11, 4) ;  G4 = -49 ; B4 = Xval(12, 4) ; 
        Tc4 = Xval([13 14], 4) ; 
      L(4) = Link('revolute','alpha',-pi/2,'a',0.115,'offset',0,'d',0.795,'modified',...
        'm', m4, 'r', r4,  'I', Im4, 'Jm', Jm4, 'G', G4, 'B',B4 , 'Tc', Tc4, ...         
        'qlim', [-400 400]*deg) ;
      m5 = Xval(1, 5) ;  r5 = Xval(2:4, 5) ;
      Iv5 = Xval(5:10, 5) ; Im5 = ROBRTB.ConvertToSymmetric(Iv5) ;       
       Jm5 = Xval(11, 5) ; G5 = -84  ; B5 = Xval(12, 5) ; 
       Tc5 =Xval(13:14, 5) ;
      L(5) = Link('revolute','alpha',pi/2,'a',0,'offset',-pi,'d',0,'modified',...
        'm', m5, 'r', r5, 'I', Im5, 'Jm', Jm5, 'G', G5, 'B', B5, 'Tc', Tc5, ...
        'qlim', [-120 120]*deg ) ;
      m6 = Xval(1, 6) ;   r6 = Xval(2:4, 6) ; 
      Iv6 = Xval(5:10, 6) ; Im6 = ROBRTB.ConvertToSymmetric(Iv6) ;       
      Jm6 = Xval(11, 6) ; G6 = -46 ; B6 = Xval(12, 6) ;
      Tc6 = Xval(13:14, 6);
      L(6) = Link('revolute','alpha',pi/2,'a',0,'offset',0,'d',0.85,'modified',...
        'm', m6, 'r', r6, 'I', Im6, 'Jm', Jm6, 'G', G6, 'B', B6, 'Tc', Tc6, ...          
        'qlim', [-400 400]*deg ) ;
      Rtb = SerialLink(L,'name','ABB2600','comment','LL');
    end



        % objective function to be optimized in torque respect 
    function Eqs = Objfun_File(Xval)
      % Rtb = ROBRTB.BuildRobot(Xval) ;
      Rtb = ROBRTB.BuildRobotII(Xval) ; % second version to optimize
      % set the correct path for loading data
      ImJ = @ ROBCal.ImportJointData;
      Readfile = ['D:\1STUDY\Phd1\Programes\Robot\WorkStation\RB2600Control\' ...
          'MatCode\IRB2600Control\DataWatcherV7\Data0802/'] ;
      Readfile = strrep(Readfile,'\','/') ;
      pathjp = [Readfile,'/Group_J6_30/','JointP_Record.txt'] ;
      pathjt = [Readfile,'/Group_J6_30/','JointT_Record1.txt'] ;
      JointP = ImJ(pathjp,[2 inf]) ;
      JointT = ImJ(pathjt,[2 inf]) ;


      dt = 0.1 ;
      Difff = ROBGUI.CDF_Int(JointP(:,1)) ;
      Nump = size(JointP, 1) ;
      % Time = linspace(1,Nump,Nump) *  dt ;
      deg = pi / 180 ;
      Q = JointP * deg ;
      Qd = Difff * Q /dt ;
      Qdd = Difff * Qd /dt ;
      Tau = Rtb.rne(Q,Qd, Qdd) ;
      % % Eqs = sqrt(sum( (Tau(:,1)-JointT(:,1)).^2)/(Nump-1)) ;
      Eqs = sqrt(sum(sum((Tau - JointT).^2 /(6*Nump-1))));
    end
  % objection function taken power into consideration
        function Eqs = Objfun_File_Power(Xval)
      % Rtb = ROBRTB.BuildRobot(Xval) ;
      Rtb = ROBRTB.BuildRobotII(Xval) ; % second version to optimize
      % set the correct path for loading data
      ImJ = @ ROBCal.ImportJointData;
      Readfile = ['D:\1STUDY\Phd1\Programes\Robot\WorkStation\RB2600Control\' ...
          'MatCode\IRB2600Control\DataWatcherV7\Data0802/'] ;
      Readfile = strrep(Readfile,'\','/') ;
      pathjp = [Readfile,'/Group_J6_60/','JointP_Record.txt'] ; % can be modified
      pathjt = [Readfile,'/Group_J6_60/','JointT_Record1.txt'] ; % can be modified
      JointP = ImJ(pathjp,[2 inf]) ;
      JointT = ImJ(pathjt,[2 inf]) ;


      dt = 0.1 ;
      Difff = ROBGUI.CDF_Int(JointP(:,1)) ;
      Nump = size(JointP, 1) ;
      % Time = linspace(1,Nump,Nump) *  dt ;
      rad = pi / 180 ;
      Q = JointP * rad ;
      Qd = Difff * Q /dt ;
      Qdd = Difff * Qd /dt ;
      Tau = Rtb.rne(Q,Qd, Qdd) ;
      % % Eqs = sqrt(sum( (Tau(:,1)-JointT(:,1)).^2)/(Nump-1)) ;
      Pexp = abs(JointT.*Qd) ;
      Pmod = abs(Tau.*Qd) ;
      Eqs = sqrt(sum(sum((Pexp - Pmod).^2 /(6*Nump-1))));
    end

    
  end
end

