%{
_______________________________________________________________________

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
****                  Class for the REAL IRB2600                   ****
***********************************************************************
*---------------------------------------------------------------------*
*                   Start date:    July 23 2024                       *
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
classdef IRB2600Real
  %IRB2600REAL Summary of this class goes here
  %   Detailed explanation goes here
  

  
  methods(Static)
    function DH = GetDH
      DH = [
        0,    0,    0, 445 ;   % offset coordinate of T00
      -90,  150,  -90,   0 ;   % offset coordinate of T10 rotate around z
        0,  700,    0,   0 ;   % offset coordinate of T21 rotate around y
      -90,  115,    0, 795 ;   % offset coordinate of T32 rotate around y
       90,    0, -180,   0 ;   % offset coordinate of T43 rotate around x
       90,    0,    0,  85 ;] ;% offset coordinate of End-effecto
      
%       DH = IRB2600Real.DH_table ;
    end
    %% Define the Base location via Transform Matrix
    function Tbase = GetTbase
       Tbase= transl( 0, 0, 0) ;
    end
    
    function [R] = RotationMatrix_Degree( axis, theta)
      %   function Rotr is to create rotation transformation matrix
      %   Input:
      %   Parameter 1: axis:
      %   to determine which axis shoule be rotated around
      %   when axis is 1, generate homogeneous rotation matrix arond x
      %   when axis is 2, generate homogeneous rotation matrix arond y
      %   when axis is 3, generate homogeneous rotation matrix arond z
      %   Parameter2 theta:
      %   This parameter determines how many rotation degree it is
      % (Unit : Degree)
      l = cosd(theta) ; m = sind(theta) ;
      switch axis
        case 1
          R = [  1  0  0  0;  0  l -m  0;  0  m  l  0;  0  0  0  1];
        case 2
          R = [  l  0  m  0;  0  1  0  0; -m  0  l  0;  0  0  0  1];
        case 3
          R = [  l -m  0  0;  m  l  0  0;  0  0  1  0;  0  0  0  1];
      end
    end
    %% To generate the siyuanshu
    function Q1234 = OrientData(Rin)
      Rori = Rin(1:3,1:3) ;
      %           q1 = sqrt(Rori(1,1) + Rori(2,2) + Rori(3,3) +1 ) /2  ;
      %           sq2 = sign(Rori(3,2) - Rori(2,3) ) ; % the sign of q2
      %           q2 = sqrt(Rori(1,1) - Rori(2,2) - Rori(3,3) +1 ) /2 *  sq2;
      %           sq3 = sign(Rori(1,3) - Rori(3,1) ) ;
      %           q3 = sqrt(Rori(2,2)  - Rori(1,1) - Rori(3,3) +1 ) /2 * sq3 ;
      %           sq4 = sign(Rori(2,1) - Rori(1,2) ) ;
      %           q4 = sqrt(Rori(3,3)  - Rori(1,1) - Rori(2,2) +1 ) /2 * sq4 ;
      %  % to generate 四元数 (Si4Yuan2Shu\ to representate the orientation parameter)
      %           Q1234 = [q1,q2,q3,q4] ;
      Q1234 = dcm2quat(Rori') ;
    end
    %%  Inverse Kinematics
    function [theta] = InnerInverseKinematics(P_target)
      %% Inverse Kinematics Function
      % to calculate joint angle from 1 to 3
      theta = zeros(1,6) ;
      DH = IRB2600Real.GetDH ;
      P_20_off = [DH(2,2) 0 DH(1,4)] ;
      l_x = norm(P_target([1 2]) )  - P_20_off(1) ;
      l_z = P_target(3) - P_20_off(3) ;
      r = sqrt( l_x^2 +  l_z^2) ;
      Off_2_x = DH(4,4) ;
      Off_2_y = DH(4,2) ;
      the_2_off = atan2d( Off_2_y, Off_2_x) ;
      l1 = DH(3,2) ;
      l2 = norm( [Off_2_x Off_2_y] ) ;
      A = (r*r + l1*l1 -l2*l2)/2/r/l1 ;
      B = (l1*l1 + l2*l2 -r*r )/2/l1/l2 ;
      phi = atan2d(l_x, l_z) ;
      alpha = acosd(A) ;   beta  = acosd(B) ;
      theta(1) = atan2d(P_target(2), P_target(1) )  ;
      theta(2) =  phi - alpha  ;
      theta(3) = ( 90-beta  + the_2_off )  ;
    end
    
    function [Q_theta] = OuterInverseKinematics(Point_Vector)
      %% Inverse Kinematics Function
      % to calculate joint angle from 1 to 5
      DH = IRB2600Real.GetDH ;
      RS = @IRB2600Real.TransMatrixCalculation ;
      P_target2 = Point_Vector(1:3) ;
      Approach_Vector = Point_Vector(4:6) ;
      Approach_Vector = Approach_Vector / norm(Approach_Vector) ;
      IKI = @ IRB2600Real.InnerInverseKinematics ;
      l56 = DH( 6, 4) ;
      P_target = P_target2 -  (l56 * Approach_Vector) ;
      Q_theta = IKI(P_target) ;
      
      Fun_temp = RS(Q_theta,transl(0,0,0)) ;
      Dot_Matrix = Fun_temp.T40(1:3,1:3) ;
      AV_T40 = Dot_Matrix' * Approach_Vector(:) ;
      AV_T40 = AV_T40 / norm(AV_T40) ;
      Pv1   = [norm(AV_T40([1 2])) AV_T40(3) ] ;  Pv1= Pv1 /norm(Pv1) ;
      Pv2   =  AV_T40([1 2]) / (norm(AV_T40([1 2])) + 1e-5) ;
      if AV_T40(3) <0
        Q_theta(5) = asind(Pv1(1))- 180 ;
      else
        Q_theta(5) = -asind(Pv1(1)) ;
      end
      Q_theta(4) =  atan2d(Pv2(2), Pv2(1)) ;
      
      % Q 6 is still be zero ;
    end
    % Inverse Kinematics Function given target point's coordinates and
    % orientation parameters Rori to calculate joint angle from 1 to 6
    function Qall = InverseKinematics(Ptar, Rori)
      OIK = @ IRB2600Real.OuterInverseKinematics ;
      Tcal = @ IRB2600Real.TransMatrixCalculation ;
      Av = Rori(1:3,3)' ;
      P_target = [Ptar, Av] ;
      Qall = OIK(P_target) ;
      R60 = Tcal(Qall,transl(0,0,0)).T60(1:3,1:3) ;
      Roff =  R60 \ Rori(1:3,1:3 ) ;
      Qall(6) = asind(Roff(2,1)) ;
    end
    % This function is to generate all transformation matrix given all
    % joint angle
    function [Tall] = TransMatrixCalculation(Q_theta)
      Rot = @ IRB2600Real.RotationMatrix_Degree ;
      DH = IRB2600Real.GetDH  ;
      Tall.T0b = IRB2600Real.GetTbase ;
      Tall.T10 = transl(0,0,DH(1,4)) * Rot(3, Q_theta(1) ) ;
      Tall.T10 = Tall.T0b * Tall.T10 ;
      Tall.T21 = Rot(1,DH(2,1)) * transl(DH(2,2),0,0) * ...
        Rot(3,DH(2,3)) * Rot(3, Q_theta(2) )  ;
      Tall.T32 = transl(DH(3,2), 0, 0) * Rot(3, Q_theta(3) ) ;
      Dis =  200 ; %%%%%%%%%%%%%%%%%
      Tall.T43 = Rot(1,DH(4,1)) * transl(DH(4,2), ...
        0, Dis)* Rot(3, Q_theta(4) )  ;
      Tall.T54 = transl(0,  0, DH(4,4) - Dis) ...
       * Rot(1,DH(5,1))   *  Rot(3,DH(5,3))    *Rot(3, Q_theta(5) ) ;
      Tall.T65 = Rot(1,DH(6,1))  * transl(0,0,DH(6,4)) * Rot(3, Q_theta(6) ) ;
      Tall.T20 = Tall.T10 *Tall.T21 ;
      Tall.T30 = Tall.T20 *Tall.T32 ;
      Tall.T40 = Tall.T30 *Tall.T43 ;
      Tall.T50 = Tall.T40 *Tall.T54 ;
      Tall.T60 = Tall.T50 *Tall.T65 ;
    end
    %% Generate the struct data type to record all trans matrix
    function Tstruct = Tall_To_Struct(Q_theta)
      Tall = IRB2600Real.TransMatrixCalculation(Q_theta) ;
      temp.Tm = [] ;  Tstruct= repmat(temp, [6 1]) ;
      Tstruct(1).Tm = Tall.T10 ;
      Tstruct(2).Tm = Tall.T20 ;
      Tstruct(3).Tm = Tall.T30 ;
      Tstruct(4).Tm = Tall.T40 ;
      Tstruct(5).Tm = Tall.T50 ;
      Tstruct(6).Tm = Tall.T60 ;
    end
    
    function P_new = LinkPoint(JointP,Linknum, P)
      TSt = @ IRB2600Real.Tall_To_Struct ; 
      Tstr = TSt(JointP) ;
      P_out = Tstr(Linknum).Tm * [P ,1]' ;
      P_new = zeros(1,3) ;
      for i = 1:3
        P_new(i) = P_out(i) ;
      end
      
    end
    
%     
    % Plot the Coordinate System in 3 Dimensional
    function  [l1,l2,l3] = Plot_Cord_3D( Tin, L)
      %   function PlotCord is to draw the homogeneous matrix in 3-D
      %   space ;
      %   Input :
      %   Prameter 1 : Tin ;
      %   Parameter L the length of the coordinate system ;
      %   Tin is the (4 X 4) homogeneous matrix ;
      width = 1.5 ;
      l1 = plot3( Tin(1,4) + [0  Tin(1,1)] * L, Tin(2,4) + ...
        [0  Tin(2,1)] * L, Tin(3,4)+ [0  Tin(3,1)] * L) ;
      %   Set the style of x-axis
      l1.LineStyle = '--'; l1.Color = 'r'; l1.LineWidth = width; hold on
      %   Draw y-axis
      l2 = plot3( Tin(1,4) + [0  Tin(1,2)]* L, Tin(2,4) + ...
        [0  Tin(2,2)] * L, Tin(3,4)+ [0  Tin(3,2)] * L);
      %   Set the style of y-axis
      l2.LineStyle = '--'; l2.Color = 'g'; l2.LineWidth = width;
      %   Draw the z-axis
      l3 = plot3( Tin(1,4) + [0  Tin(1,3)] * L, Tin(2,4) + ...
        [0  Tin(2,3)] * L, Tin(3,4)+ [0  Tin(3,3)] * L);
      %   Set the style of z-axis
      l3.LineStyle = '--'; l3.Color = 'b'; l3.LineWidth = width;
    end
    %% Plot the Base Coordinate
    function  Plot_Cord_Base_3D(  Tin, L)
      [l1,l2,l3] = IRB2600Real.Plot_Cord_3D( Tin, L);
      l1.LineStyle = '-';   l1.LineWidth = 2;
      l2.LineStyle = '-';   l2.LineWidth = 2;
      l3.LineStyle = '-';   l3.LineWidth = 2;
    end
    %% Draw Geometry
    % Draw Cylindar Function
    function Draw_Cylindar_Link(H,R,T_base)
      N = 25 ;  t = linspace(0, 2 * pi, N+1)' ; t(end) = [];
      x = R * cos ( t) ;
      y = R * sin ( t) ;
      z = t * 0 ;
      Vertex = [x y z-H/2 ; x y z+H/2] ;
      k = (1: N)' ; Faces = [k k+1 k+N+1 k+N] ;
      Faces(end,:) = [1 N N+N 1+N] ;
      Vertex_new= zeros(N+N,3);
      for i = 1 : (N+ N)
        p_new = T_base * [Vertex(i,:)' ; 1] ;
        Vertex_new(i,:) = p_new(1:3)' ;
      end
      color = [x.*0 ; x.*0 + 1] ;
      patch('Vertices',Vertex_new,'Faces',Faces, 'FaceVertexCData',...
        color,'FaceColor','interp','FaceAlpha',0.5); hold on
      %             [ x_new, y_new, z_new] = IRB2600.DrawDisk( R+1, [0 0 0], T1(1:3,4)) ;
      %              S = surf(x_new,y_new ,z_new);  S.FaceAlpha = 0.5 ;
      %
      %             [ x_new, y_new, z_new] = IRB2600.DrawDisk( R+1, [0 0 H], T1(1:3,4)) ;
      %              S = surf(x_new,y_new ,z_new);  S.FaceAlpha = 0.5 ;
    end
    % Draw Cubic Function
    function Draw_Cubic_Link(H,T1)
      % Input the start Point
      % Wid = H / 30;
      wid = 20 ;
      Start_point = [ -wid -wid  0] ;
      % Input the start Point
      Final_point = [ wid  wid  H] ;
      vertexIndex = [ 0 0 0; 0 0 1; 0 1 0; 0 1 1; 1 0 0; 1 0 1; 1 1 0; 1 1 1] ;
      cuboidSize  = Final_point-Start_point ;
      vertex      = repmat(Start_point,8,1) + vertexIndex.*repmat(cuboidSize,8,1) ;
      % Define vertex according to 6 faces respectively
      facet = [ 1 2 4 3; 1 2 6 5; 1 3 7 5; 2 4 8 6; 3 4 8 7; 5 6 8 7] ;
      color = [ 0; 1; 0; 1; 0; 1; 0; 1] ;
      Vertex_new = vertex.*0 ;
      for i = 1 : 8
        p_new = T1 * [vertex(i,:)' ; 1] ;
        Vertex_new(i,:) = p_new(1:3)' ;
      end
      patch('Vertices',Vertex_new,'Faces',...
        facet,'FaceVertexCData',color,'FaceColor','interp','FaceAlpha',0.5) ;
    end
        function [varargout] = DrawDisk( R, T_disk, axiss)
          % function predefine
          Rotd = @ IRB2600Real.RotationMatrix_Degree ;           
      Off = T_disk(1:3,4) ;
      if axiss == 'X'
        N_v = T_disk(1:3,1) ;
      elseif axiss == 'Y'
        N_v = T_disk(1:3,2) ;
      elseif axiss == 'Z'
        N_v = T_disk(1:3,3) ;
      end
      t = linspace(0,2*pi)' ;
      Coe = [ 0.1 0.2 0.4 0.6 0.8 1.0];
      x = cos(t) ;  x = ( x * Coe) * R ;
      y = sin(t);   y = ( y * Coe) * R ;
      z = t.*0;     z = ( z * Coe) * R ;
      Pv1 = [norm(N_v([1 2])) N_v(3)];  Pv1= Pv1 /norm(Pv1);
      Pv2 = N_v([1 2]) / (norm(N_v([1 2])) + 1e-5);
      if N_v(3) < 0
        beta = 180 - asind(Pv1(1)) ;
      else
        beta =  asind(Pv1(1)) ;
      end
      gamma = atan2d(Pv2(2), Pv2(1)) ;
      T_new =   Rotd( 3, gamma) * Rotd( 2, beta) ;
      x_new = x .* 0 ;  y_new = y .* 0 ;  z_new = z .* 0 ;
      for k = 1 : 600
        p_new = T_new * [x(k) y(k) z(k) 1]' ;
        x_new(k) =  p_new(1); y_new(k) =  p_new(2);
        z_new(k) =  p_new(3);
      end
      x_new = x_new +  Off(1) ; y_new = y_new +  Off(2) ;
      z_new = z_new +  Off(3) ;
      CO(:,:,1) = z_new.* 0  ;
      CO(:,:,2) = z_new.* 0 +1;
      CO(:,:,3) = z_new.* 0 +1;
      S = surf(x_new, y_new, z_new, CO) ; shading interp
      S.FaceAlpha = 0.2 ;
      varargout{1} = x_new ; varargout{2} = y_new;
      varargout{3} = z_new ;
    end
    function DrawRob_now(Joint_Theta)
      % function Citation
      Tcal =  @  IRB2600Real.TransMatrixCalculation ;
      PCB =  @  IRB2600Real.Plot_Cord_Base_3D ;
      PC = @  IRB2600Real.Plot_Cord_3D ;
      Disk = @ IRB2600Real.DrawDisk ;
      Cylindar = @ IRB2600Real.Draw_Cylindar_Link ; % Draw Cylindar 
      Cubic = @ IRB2600Real.Draw_Cubic_Link ;  % Draw Cubic
      Rotd = @ IRB2600Real.RotationMatrix_Degree ;
      DH = IRB2600Real.GetDH ;
      %% Start Drawing
      Tall = Tcal( Joint_Theta) ; 
      TL_Base = 50 ; TL = 50 ; % the length of the Base coordinate system axis
      % Draw Coordinates of Link Base       
%       PCB( Tall.T0b, TL_Base)
      TS = @ IRB2600Real.Tall_To_Struct ;
      Tstruc = TS(Joint_Theta) ;
      PCB( IRB2600Real.GetTbase, TL_Base)
      % Draw Coordinates of Link from 1 to 6  
      for Link = 1: 6
        PC(Tstruc(Link).Tm, TL) ;
      end 
      %%%% Draw_Link Base ---- Cubic + Cylindar
      R_base = 150 ;  H_base = 50 ;            
      Disk( R_base, Tall.T0b,'Z') ;
      Disk( R_base, Tall.T0b *transl(0 ,0 ,H_base),'Z') ;
      Cylindar(H_base, R_base, Tall.T0b * transl( 0, 0, H_base/2)) ;
      %%%% Draw_Link 1 ---- Cubic + Cylindar      
      Cubic(DH(2,2),Tall.T10 * Rotd(2,90)) ;
      Cubic( DH(1,4)-H_base  ,Tall.T10 * transl(0,0,H_base-DH(1,4) ) ) ; 
      Rlink1 = 40 ; Hlink1 = 50 ;
      Cylindar(Hlink1, Rlink1 ,Tall.T20)
      Disk( Rlink1, Tall.T20 * transl(0,0,Hlink1/2),'Z') ;
      Disk( Rlink1, Tall.T20 * transl(0,0,-Hlink1/2),'Z') ;
      %%%% Draw_Link 2 ---- Cubic + Cylindar   
      Cubic( DH(3,2), Tall.T20 * Rotd(2,90)) ;
      Rlink2 = 40 ; Hlink2 = 50 ;
      Cylindar(Hlink2, Rlink2 ,Tall.T30)
      Disk( Rlink2, Tall.T30 * transl(0,0,Hlink2/2),'Z') ;
      Disk( Rlink2, Tall.T30 * transl(0,0,-Hlink2/2),'Z') ;
      %%%% Draw_Link 3 ---- Cubic + Cylindar   
      Cubic( DH(4,2), Tall.T30 * Rotd(2,90)) ;
      Off = 200 ;  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      Cubic( Off, Tall.T30 *transl(DH(4,2),0,0)* Rotd(3,90) * Rotd(2,90) ) ;
      
      %%%% Draw_Link 4---- Cubic + Cylindar         
%       Cylindar(Hlink2, Rlink2 ,Tall.T40)
      Rlink4 = 40 ; Hlink4 = 40 ;
      Cylindar(Hlink4, Rlink4 ,Tall.T40)
      Disk( Rlink4, Tall.T40 * transl( 0, 0, Hlink4/2),'Z') ;
      Disk( Rlink4, Tall.T40 * transl( 0, 0,-Hlink4/2),'Z') ;
      Cubic( DH(4,4)-Off, Tall.T40 ) ;       
      %%%% Draw_Link 5----  Cubic + Cylindar 
      Rlink5= 40 ; Hlink5 = 40 ; 
      Cylindar(Hlink5, Rlink5 ,Tall.T50) ;
      Disk( Rlink4, Tall.T50 * transl( 0, 0, Hlink5/2),'Z') ;
      Disk( Rlink4, Tall.T50 * transl( 0, 0,-Hlink5/2),'Z') ;
      Cubic( 100, Tall.T50 * Rotd(1,90)) ;
      %%%% Draw_Link 6----  Cubic + Cylindar 
      Rlink6= 40 ; Hlink6 = 40 ; 
      Cylindar(Hlink6, Rlink6 ,Tall.T60)      
      Disk( Rlink4, Tall.T60 * transl( 0, 0, Hlink6/2),'Z') ;
      Disk( Rlink4, Tall.T60 * transl( 0, 0,-Hlink6/2),'Z') ;
      Points = [Tall.T0b(1:3,4)' ; Tall.T10(1:3,4)' ; Tall.T20(1:3,4)' ;
        Tall.T30(1:3,4)' ;   Tall.T40(1:3,4)' ;Tall.T50(1:3,4)' ; Tall.T60(1:3,4)'] ;
      plot3( Points(:,1), Points(:,2), Points(:,3), '--', 'Linewidth', 1.5)
      shading interp
      axis equal 
      view(120,30)
    end
    %% Dynamic
    % Force Analysis to realize the force transformation between two links
    function ForceT = ForceTrance(TranceMatrix , Force)
      ForceT = zeros( 1, 6) ;
      vlocal = TranceMatrix(:,4) ;
      Flocal = TranceMatrix*  [Force(1:3) 0]' ;
      Mlocal = TranceMatrix * [Force(4:6) 0]' ;
      ForceT(1:3) =  Flocal(1:3) ;
      ForceT(4:6) = reshape(Mlocal(1:3),1,3) + ...
        cross(vlocal(1:3),ForceT(1:3)) ;
    end
    function [Force] = Force_Analysis( F_end, JointP)
      % JointP joint position
      Tcal = @ IRB2600Real.TransMatrixCalculation ;
      FT =  @ IRB2600Real.ForceTrance ;
      Tall = Tcal( JointP, transl(0,0,0)) ;       
      Force = zeros( 6, 6) ;       
      %%% Transport force from Link <End> to 6
      F_E6 = zeros(1,6) ;
      F_local_E6 = Tall.T60 \ [F_end(1:3) 0]' ;
      M_local_E6 = Tall.T60 \ [F_end(4:6) 0]' ;
      V_lE6 = [0 0 0 1] ;
      F_E6(1:3) =  F_local_E6(1:3) ;
      F_E6(4:6) =  reshape(M_local_E6(1:3),1,3) + ...
        cross(V_lE6(1:3),F_E6(1:3)) ;  Force(6,:) = F_E6 ;
      %%% Transport force from Link 6 to 5       
      F_65 = FT(Tall.T65 , F_E6) ;    Force(5,:) = F_65 ;       
      %%% Transport force from Link 5 to 4  
      F_54 = FT(Tall.T54 , F_65) ;    Force(4,:) = F_54 ;
      %%% Transport force from Link 4 to 3  
      F_43 = FT(Tall.T43 , F_54) ;    Force(3,:) = F_43 ;
      %%% Transport force from Link 3 to 2  
      F_32 = FT(Tall.T32 , F_43) ;    Force(2,:) = F_32 ;
      %%% Transport force from Link 2 to 1  
      F_21 = FT(Tall.T21 , F_32) ;    Force(1,:) = F_21 ; 
    end
    
    function [Torque] = Torque_Output( F_end, JointP)
      FA = @ IRB2600Real.Force_Analysis ;
      Force = FA( F_end, JointP) ;
      %       Moment_index =[ [0 0 0 0 0 1] ; [0 0 0 0 0 1] ;  [0 0 0 0 0 1] ;
      %         [0 0 0 0 0 1] ; [0 0 0 0 0 1] ;  [0 0 0 0 0 1] ; ] ;
      %       Torque = sum(Moment_index.*Force,2)' ;
      Torque = Force(:,6)' ;
    end
    function [Force] = Force_AnalysisII(F_end, JointP)
      Tcal = @ IRB2600Real.TransMatrixCalculation ;
      FT =  @ IRB2600Real.ForceTrance ;
      Tall = Tcal( JointP, transl(0,0,0)) ;       
      Gg = 9.8 ;    % (unit : m/s^2��;
      Force = zeros( 6, 6) ; 
      %%% Transport force from Link <End> to 6
      F_E6 = zeros(1,6) ;
      F_local_E6 = Tall.T60 \ [F_end(1:3) 0]' ;
      M_local_E6 = Tall.T60 \ [F_end(4:6) 0]' ;
      V_lE6 = [0 0 0 1] ;
      F_E6(1:3) =  F_local_E6(1:3) ;
      F_E6(4:6) =  reshape(M_local_E6(1:3),1,3) + ...
        cross(V_lE6(1:3),F_E6(1:3)) ;  
      %%% Cm6
      Cm6 = [0 0 10] ; % Mass Center of Joint 6
      Mass6 = 2 ;  % mass of the
      Grav6 = [0 0 -Mass6*Gg]  ;
      Fmg6 = zeros(1,6) ;
      Mg6_local = Tall.T60 \ [Grav6 0]' ;
      Fmg6(1:3) = Mg6_local(1:3) ;
      Fmg6(4:6) = cross( Cm6, Fmg6(1:3)) ;
      F_E6 = F_E6 + Fmg6 ; Force(6,:) = F_E6 ;
      %%% Transport force from Link 6 to 5       
      F_65 = FT(Tall.T65 , F_E6) ;   
      %%% Cm5
      Cm5 = [0 -40 0] ; % Mass Center of Joint 6
      Mass5 = 2.5 ;  % mass of the
      Grav5 = [0 0 -Mass5*Gg]  ;
      Fmg5 = zeros(1,6) ;
      Mg5_local = Tall.T50 \ [Grav5 0]'  ;
      Fmg5(1:3) = Mg5_local(1:3) ;
      Fmg5(4:6) = cross( Cm5, Fmg5(1:3))  ;
      F_65 = F_65 + Fmg5 ;  Force(5,:) = F_65 ;  
      %%% Transport force from Link 5 to 4  
      F_54 = FT(Tall.T54 , F_65) ;   
      %%% Cm4
      Cm4 = [0 0 400] ; % Mass Center of Joint 6
      Mass4= 20 ;  % mass of the
      Grav4 = [0 0 -Mass4*Gg]  ;
      Fmg4 = zeros(1,6) ;
      Mg4_local = Tall.T40 \ [Grav4 0]'  ;
      Fmg4(1:3) = Mg4_local(1:3) ;
      Fmg4(4:6) = cross( Cm4, Fmg4(1:3)) ;
      F_54 = F_54 +  Fmg4 ; Force(4,:) = F_54 ;
      %%% Transport force from Link 4 to 3  
      F_43 = FT(Tall.T43 , F_54) ;    
      %%% Cm3       
      Cm3 = [100 100 0] ; % Mass Center of Joint 6
      Mass3= 20 ;  % mass of the
      Grav3 = [0 0 -Mass3*Gg]  ;
      Fmg3 = zeros(1,6) ;
      Mg3_local = Tall.T30 \ [Grav3 0]'  ;
      Fmg3(1:3) = Mg3_local(1:3) ;
      Fmg3(4:6) = cross( Cm3, Fmg3(1:3))   ;
      F_43 = F_43 +  Fmg3 ; Force(3,:) = F_43 ;
      %%% Transport force from Link 3 to 2  
      F_32 = FT(Tall.T32 , F_43) ;    
      %%% Cm2
      Cm2 = [400 0 0] ; % Mass Center of Joint 6
      Mass2= 30 ;  % mass of the link2
      Grav2 = [0 0 -Mass2*Gg]  ;
      Fmg2 = zeros(1,6) ;
      Mg2_local = Tall.T20 \ [Grav2 0]'  ;
      Fmg2(1:3) = Mg2_local(1:3) ;
      Fmg2(4:6) = cross( Cm2, Fmg2(1:3))  ;
      F_32 = F_32 +  Fmg2 ;  Force(2,:) = F_32 ;
      %%% Transport force from Link 2 to 1  
      F_21 = FT(Tall.T21 , F_32) ;
      %%% Cm1 
      Cm1 = [0 0 -100] ; % Mass Center of Joint 6
      Mass1= 40 ;  % mass of the link2
      Grav1 = [0 0 -Mass1*Gg]  ;
      Fmg1 = zeros(1,6) ;
      Mg1_local = Tall.T10 \ [Grav1 0]'  ;
      Fmg1(1:3) = Mg1_local(1:3) ;
      Fmg1(4:6) = cross( Cm1, Fmg1(1:3))  ;
      F_21 = F_21 + Fmg1 ;  Force(1,:) = F_21 ;
    end     
    function [Torque] = Torque_OutputII( F_end, JointP)
      FA = @ IRB2600Real.Force_AnalysisII ;
      Force = FA( F_end, JointP) ;
      %       Moment_index =[ [0 0 0 0 0 1] ; [0 0 0 0 0 1] ;  [0 0 0 0 0 1] ;
      %         [0 0 0 0 0 1] ; [0 0 0 0 0 1] ;  [0 0 0 0 0 1] ; ] ;
      %       Torque = sum(Moment_index.*Force,2)' ;
      Torque = Force(:,6)' ;
    end
   %% Skew Symmetric Matrix
   
   function Smatric = SkewSymmetricMatrix(Vector3)
%      Smatric=zeros(3,3) ;
     
     Smatric = [0, -Vector3(3), Vector3(2);
                Vector3(3), 0, -Vector3(1); 
                -Vector3(2), Vector3(1), 0] ;
%      Smatric(1,2) = -Vector3(3) ;  Smatric(2,1) =  Vector3(3) ; 
%      Smatric(1,3) = Vector3(2) ;  Smatric(3,1) = - Vector3(2) ;
%      Smatric(2,3) = - Vector3(1) ; Smatric(3,2) =  Vector3(1) ;
     
%      Smatric = Smatric- Smatric.' ;
   end
   
   
   %% Numerical Section
    function [D] = CDF_Int(X)
      %CDF create a differential matrix
      len = length(X) ;
      Di = [ 1,        1:len-1,           2:len, len] ;
      Dj = [ 1,          2:len,         1:len-1, len] ;
      Dv = [-0.5, ones(1, len-1)- 0.5, -ones(1, len-1) + 0.5,   0.5] ;
      Dv(1,[1 2]) = [-1 1]; Dv(end,[end-1 end]) = [-1 1] ;
      D = sparse( Di, Dj, Dv, len, len) ;
    end
    % Spiral line generation
    function P = SPIRAL(Off, Nc, H, R, N)
      %  N = 100 ;
      t = linspace( 0, 2 * pi, N)' ;
      x = R * cos (Nc * t) +  Off( 1) ;
      y = R * sin (Nc * t)+  Off( 2)  ;
      z = t/2/pi * H +  Off( 3) ;
      P = [ x, y, z] ;
    end
  end
end



