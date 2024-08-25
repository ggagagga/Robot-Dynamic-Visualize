classdef ROBDEMO % modified by IRB2600 REAL
  %ROBDEMO This is to help the GUIMETHOD
  %
  
  
  methods(Static)
    function DH = GetDH
      DH = [
        0,    0,    0, 445 ;  % offset coordinate of T00
        -90,  150,  -90,   0 ;  % offset coordinate of T10 rotate around z
        0,  700,    0,   0 ;  % offset coordinate of T21 rotate around y
        -90,  115,    0, 795 ;  % offset coordinate of T32 rotate around y
        90,    0, -180,   0 ;  % offset coordinate of T43 rotate around x
        90,    0,    0,  85 ;] ;% offset coordinate of End-effecto
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
      %   (Unit : Degree)
      l = cosd(theta); m = sind(theta);
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
      %  % to generate ËÄÔªÊý (Si4Yuan2Shu\ to representate the orientation parameter)
      %           Q1234 = [q1,q2,q3,q4] ;
      Q1234 = dcm2quat(Rori') ;
    end
    
    % This function is to generate all transformation matrix given all
    % joint angle
    function [Tall] = TransMatrixCalculation(Q_theta)
      Rot = @ ROBDEMO.RotationMatrix_Degree ;
      DH = ROBDEMO.GetDH  ;
      Tbase = transl(0,0,0) ;
      Tall.T0b = Tbase ;
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
    
    
        % Plot the Coordinate System in 3 Dimensional
    function  [l1,l2,l3] = Plot_Cord_3D( Tin, L, Destination)
      %   function PlotCord is to draw the homogeneous matrix in 3-D
      %   space ;
      %   Input :
      %   Prameter 1 : Tin ;
      %   Parameter L the length of the coordinate system ;
      %   Tin is the (4 X 4) homogeneous matrix ;
      width = 1.5 ;
      l1 = plot3(Destination, Tin(1,4) + [0  Tin(1,1)] * L, Tin(2,4) + ...
        [0  Tin(2,1)] * L, Tin(3,4)+ [0  Tin(3,1)] * L) ;
      %   Set the style of x-axis
      l1.LineStyle = '--'; l1.Color = 'r'; l1.LineWidth = width; hold(Destination, 'on')
      %   Draw y-axis
      l2 = plot3(Destination, Tin(1,4) + [0  Tin(1,2)]* L, Tin(2,4) + ...
        [0  Tin(2,2)] * L, Tin(3,4)+ [0  Tin(3,2)] * L);
      %   Set the style of y-axis
      l2.LineStyle = '--'; l2.Color = 'g'; l2.LineWidth = width;
      %   Draw the z-axis
      l3 = plot3(Destination, Tin(1,4) + [0  Tin(1,3)] * L, Tin(2,4) + ...
        [0  Tin(2,3)] * L, Tin(3,4)+ [0  Tin(3,3)] * L);
      %   Set the style of z-axis
      l3.LineStyle = '--'; l3.Color = 'b'; l3.LineWidth = width;
    end
    
        %% Plot the Base Coordinate
    function  Plot_Cord_Base_3D(  Tin, L, Destination)
      [l1,l2,l3] = ROBDEMO.Plot_Cord_3D( Tin, L, Destination);
      l1.LineStyle = '-';   l1.LineWidth = 2;
      l2.LineStyle = '-';   l2.LineWidth = 2;
      l3.LineStyle = '-';   l3.LineWidth = 2;
    end
        %% Draw Geometry
    % Draw Cylindar Function
    function Draw_Cylindar_Link(H,R,T_base, Destination)
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
      patch(Destination, 'Vertices',Vertex_new,'Faces',Faces, 'FaceVertexCData',...
        color,'FaceColor','interp','FaceAlpha',0.5); hold on
    end
    % Draw Cubic Function
    function Draw_Cubic_Link(H,T1, Destination)
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
      patch(Destination,'Vertices',Vertex_new,'Faces',...
        facet,'FaceVertexCData',color,'FaceColor','interp','FaceAlpha',0.5) ;
    end
    %% function to draw disk
    function [varargout] = DrawDisk( R, T_disk, axiss, Destination)
      Rotd = @ ROBDEMO.RotationMatrix_Degree ;
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
      S = surf(Destination, x_new, y_new, z_new, CO) ; shading(Destination,'interp')
      S.FaceAlpha = 0.2 ;
      varargout{1} = x_new ; varargout{2} = y_new;
      varargout{3} = z_new ;
    end
    
    %%

    
    
    function DrawRob_now(Joint_Theta, Destination)
      % function citation
      Tcal =  @  ROBDEMO.TransMatrixCalculation ;
      Rotd = @ ROBDEMO.RotationMatrix_Degree ;
      Disk = @ ROBDEMO.DrawDisk ;
      PCB = @  ROBDEMO.Plot_Cord_Base_3D ;
      PC = @  ROBDEMO.Plot_Cord_3D ;
      Cylindar = @ ROBDEMO.Draw_Cylindar_Link ; % Draw Cylindar
      Cubic = @ ROBDEMO.Draw_Cubic_Link ;  % Draw Cubic       
      DH = ROBDEMO.GetDH ;       
      Tin = Rotd(3,0) ;  L = 20  ;
      PCB(Tin, L, Destination) ;       
     %% Start Drawing
      Tall = Tcal( Joint_Theta) ;       
      TL_Base = 50 ; TL = 50 ; % the length of the Base coordinate system axis
      % Draw Coordinates of Link Base  
      PCB( Tall.T0b, TL_Base, Destination)
      % Draw Coordinates of Link 1
      PC(Tall.T10, TL, Destination) ;
      % Draw Coordinates of Link 2
      PC(Tall.T20, TL, Destination) ;
      % Draw Coordinates of Link 3
      PC(Tall.T30, TL, Destination) ;
      % Draw Coordinates of Link 4
      PC(Tall.T40, TL, Destination) ;
      % Draw Coordinates of Link 5
      PC(Tall.T50, TL, Destination) ;
      % Draw Coordinates of Link 6
      PC(Tall.T60, TL, Destination) ;
      %%%% Draw_Link Base ---- Cubic + Cylindar
      R_base = 150 ;  H_base = 50 ;     
      Disk( R_base, Tall.T0b,'Z', Destination) ;
      Disk( R_base, Tall.T0b *transl(0 ,0 ,H_base),'Z', Destination) ;
      Cylindar(H_base, R_base, Tall.T0b * transl( 0, 0, H_base/2), Destination ) ;
      %%%% Draw_Link 1 ---- Cubic + Cylindar 
      Cubic(DH(2,2),Tall.T10 * Rotd(2,90), Destination) ;
      Cubic( DH(1,4)-H_base  ,Tall.T10 * transl(0,0,H_base-DH(1,4) ), Destination ) ; 
      Rlink1 = 40 ; Hlink1 = 50 ;
      Cylindar(Hlink1, Rlink1 ,Tall.T20, Destination) ;
      Disk( Rlink1, Tall.T20 * transl(0,0,Hlink1/2),'Z', Destination) ;
      Disk( Rlink1, Tall.T20 * transl(0,0,-Hlink1/2),'Z', Destination) ;
      %%%% Draw_Link 2 ---- Cubic + Cylindar   
      Cubic( DH(3,2), Tall.T20 * Rotd(2,90), Destination) ;
      Rlink2 = 40 ; Hlink2 = 50 ;
      Cylindar(Hlink2, Rlink2 ,Tall.T30, Destination)
      Disk( Rlink1, Tall.T30 * transl(0,0,Hlink1/2),'Z', Destination) ;
      Disk( Rlink1, Tall.T30 * transl(0,0,-Hlink1/2),'Z', Destination) ;
      %%%% Draw_Link 3 ---- Cubic + Cylindar   
      Cubic( DH(4,2), Tall.T30 * Rotd(2,90), Destination) ;
      Off = 200 ;  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      Cubic( Off, Tall.T30 *transl(DH(4,2),0,0)* Rotd(3,90) * Rotd(2,90) ,Destination) ;
       %%%% Draw_Link 4---- Cubic + Cylindar  
       Rlink4 = 40 ; Hlink4 = 40 ;
      Cylindar(Hlink4, Rlink4 ,Tall.T40, Destination)
      Disk( Rlink4, Tall.T40 * transl( 0, 0, Hlink4/2),'Z', Destination) ;
      Disk( Rlink4, Tall.T40 * transl( 0, 0,-Hlink4/2),'Z', Destination) ;
      Cubic( DH(4,4)-Off, Tall.T40 , Destination) ;
      %%%% Draw_Link 5----  Cubic + Cylindar 
      Rlink5= 40 ; Hlink5 = 40 ; 
      Cylindar(Hlink5, Rlink5 ,Tall.T50, Destination) ;
      Disk( Rlink4, Tall.T50 * transl( 0, 0, Hlink5/2),'Z', Destination) ;
      Disk( Rlink4, Tall.T50 * transl( 0, 0,-Hlink5/2),'Z', Destination) ;
      Cubic( 100, Tall.T50 * Rotd(1,90), Destination) ;
      %%%% Draw_Link 6----  Cubic + Cylindar 
      Rlink6= 40 ; Hlink6 = 40 ; 
      Cylindar(Hlink6, Rlink6 ,Tall.T60, Destination)      
      Disk( Rlink4, Tall.T60 * transl( 0, 0, Hlink6/2),'Z', Destination) ;
      Disk( Rlink4, Tall.T60 * transl( 0, 0,-Hlink6/2),'Z', Destination) ;
      Points = [Tall.T0b(1:3,4)' ; Tall.T10(1:3,4)' ; Tall.T20(1:3,4)' ;
        Tall.T30(1:3,4)' ;   Tall.T40(1:3,4)' ;Tall.T50(1:3,4)' ; Tall.T60(1:3,4)'] ;
      plot3(Destination, Points(:,1), Points(:,2), Points(:,3), '--', 'Linewidth', 1.5)
      shading(Destination, 'interp') ,  axis(Destination, 'equal')  ;
      axis(Destination, [-300 1500 -800 800 0 1500] )
      view(Destination, 120,30)
    end
    % this function is to change the struct in to cell format.
    function TCell = Tall_To_Cell(Q_theta)
      Tall = ROBDEMO.TransMatrixCalculation(Q_theta) ;
      TCell=cell(1,6) ;
      TCell{1} = Tall.T10 ;
      TCell{2} = Tall.T20 ;
      TCell{3} = Tall.T30 ;
      TCell{4} = Tall.T40 ;
      TCell{5} = Tall.T50 ;
      TCell{6} = Tall.T60 ;
    end
    

    

    
    function Points_all = TraceCalculation(JointP)
      % function citation
      Tcal =  @  ROBDEMO.TransMatrixCalculation ;
      Num_points = size(JointP,1) ;
      Points_all = zeros(Num_points,3) ;
      for i = 1: Num_points
       Points_all(i, : ) = Tcal(JointP(i,:)).T60(1:3,4)' ;
      end
       
       
    end
    
    
    
    
  end
end

