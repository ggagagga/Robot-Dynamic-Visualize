classdef RBARM2600
    % Class RBARM2600 is to simulate the robot arm with the
    % combination of robot studio
    %
    
    
    properties
        % DH table alpha, a, theta, d (unit : degree, mm)
        DH_Table = [
            0,    0,    0, 445 ;  % offset coordinate of T00
            -90,  150,  -90,   0 ;  % offset coordinate of T10 rotate around z
            0,  700,    0,   0 ;  % offset coordinate of T21 rotate around y
            -90,  115,    0, 795 ;  % offset coordinate of T32 rotate around y
            90,    0, -180,   0 ;  % offset coordinate of T43 rotate around x
            90,    0,    0,  85 ;] ;% offset coordinate of End-effector
        T00   % Base coordinate
%         T10   % Transform Coordinate from Base to Link 1
%         T21   % Transform Coordinate from Link-1 to Link-2
%         T32   % Transform Coordinate from Link-2 to Link-3
%         T43   % Transform Coordinate from Link-3 to Link-4
%         T54   % Transform Coordinate from Link-4 to Link-5
%         T65   % Transform Coordinate from Link-5 to Link-6
%         TE6   % Transform Coordinate from Link-6 to End_Effector
%         T20   % Transform Coordinate from Base to Link-2
%         T30   % Transform Coordinate from Base to Link-3
%         T40   % Transform Coordinate from Base to Link-4
%         T50   % Transform Coordinate from Base to Link-5
%         T60   % Transform Coordinate from Base to Link
%         TE0   % Transform Coordinate from Base to End_Effector
    end
    
    

    
    
    
    methods
        function obj = RBARM2600
            obj.T00 = transl(0,0,0);
            
        end
        




         

        
        
        
    end
    
    methods(Static)
        %% Rotation Transformation Funciton (unit: degree)
                %% DH parameter
        function DH = GetDH
                    DH = [
              0,    0,    0, 445 ;  % offset coordinate of T00
            -90,  150,  -90,   0 ;  % offset coordinate of T10 rotate around z
              0,  700,    0,   0 ;  % offset coordinate of T21 rotate around y
            -90,  115,    0, 795 ;  % offset coordinate of T32 rotate around y
             90,    0, -180,   0 ;  % offset coordinate of T43 rotate around x
             90,    0,    0,  85 ;] ;% offset coordinate of End-effecto
        end
        
        
        function JointLim = Getlim
          JointLim = [-180 180;
                   -95 155; -180 75; -400 400; -120 120; -400, 400 ] ;
        end
        
        function [R] = Rotation_Degree( axis, theta)
            %   function Rotr is to create rotation transformation matrix
            %   Input:
            %   Parameter 1: axis:
            %   to determine which axis shoule be rotated around
            %   when axis is 1, generate homogeneous rotation matrix arond x
            %   when axis is 2, generate homogeneous rotation matrix arond y
            %   when axis is 3, generate homogeneous rotation matrix arond z
            %   Parameter2 theta:
            %   This parameter determines how many rotation degree it is
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
        
        function Q1234 = OrientData(Rin)
          Rori = Rin(1:3,1:3) ;
%           q1 = sqrt(Rori(1,1) + Rori(2,2) + Rori(3,3) +1 ) /2  ;
%           sq2 = sign(Rori(3,2) - Rori(2,3) ) ; % the sign of q2
%           q2 = sqrt(Rori(1,1) - Rori(2,2) - Rori(3,3) +1 ) /2 *  sq2;
%           sq3 = sign(Rori(1,3) - Rori(3,1) ) ;
%           q3 = sqrt(Rori(2,2)  - Rori(1,1) - Rori(3,3) +1 ) /2 * sq3 ;           
%           sq4 = sign(Rori(2,1) - Rori(1,2) ) ;
%           q4 = sqrt(Rori(3,3)  - Rori(1,1) - Rori(2,2) +1 ) /2 * sq4 ;
%           
%           Q1234 = [q1,q2,q3,q4] ;
          Q1234 = dcm2quat(Rori') ;
        end
        
        %%  Inverse Kinematics
        function [theta] = InnerInverseKinematics(P_target)
          %% Inverse Kinematics Function
           % to calculate joint angle from 1 to 3
            theta = zeros(1,6) ;
            DH = RBARM2600.GetDH ;
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
             DH = RBARM2600.GetDH ;     
             RS = @RBARM2600.TransMatrixCalculation ;
             P_target2 = Point_Vector(1:3) ;
             Approach_Vector = Point_Vector(4:6) ;
             Approach_Vector = Approach_Vector / norm(Approach_Vector) ;
             IKI = @ RBARM2600.InnerInverseKinematics ;
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
        
        function Qall = InverseKinematics(Ptar, Rori)
          OIK = @ RBARM2600.OuterInverseKinematics ;     
          Tcal = @ RBARM2600.TransMatrixCalculation ;
          Av = Rori(1:3,3)' ;
          P_target = [Ptar, Av] ;
          Qall = OIK(P_target) ;
          R60 = Tcal(Qall,transl(0,0,0)).T60(1:3,1:3) ;
          Roff =  R60 \ Rori(1:3,1:3 ) ;
          Qall(6) = asind(Roff(2,1)) ;

        end
        
        
        
%         function [Q_target] = IKINE(Ptar, Rori) 
%             m = size(P_target,1) ;
%             IK = @RBARM2600.InverseKinematics ;            
%             Q_target = zeros(m,6) ;
%             for i = 1:m
%                 Q_target(i,:) = IK(P_target(i,:)) ;
%             end
%         end
        

        
        function [Tall] = TransMatrixCalculation(Q_theta,Tbase)
            Rot = @ RBARM2600.Rotation_Degree ;
            DH = RBARM2600.GetDH  ;                
            Tall.T0b = Tbase ;             
            Tall.T10 = transl(0,0,DH(1,4)) * Rot(3, Q_theta(1) ) ;
            Tall.T10 = Tall.T0b * Tall.T10 ;
            Tall.T21 = Rot(1,DH(2,1)) * transl(DH(2,2),0,0) * ...
                Rot(3,DH(2,3)) * Rot(3, Q_theta(2) )  ;
            Tall.T32 = transl(DH(3,2), 0, 0) * Rot(3, Q_theta(3) ) ;
            Tall.T43 = Rot(1,DH(4,1)) * transl(DH(4,2), ...
                0, DH(4,4))* Rot(3, Q_theta(4) )  ;
            Tall.T54 = Rot(1,DH(5,1))  * Rot(3,DH(5,3)) * Rot(3, Q_theta(5) ) ;
            Tall.T65 = Rot(1,DH(6,1))  * transl(0,0,DH(6,4)) * Rot(3, Q_theta(6) ) ;  
            Tall.T20 = Tall.T10 *Tall.T21 ;
            Tall.T30 = Tall.T20 *Tall.T32 ;
            Tall.T40 = Tall.T30 *Tall.T43 ;
            Tall.T50 = Tall.T40 *Tall.T54 ;
            Tall.T60 = Tall.T50 *Tall.T65 ;
        end
        
        
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
            [l1,l2,l3] = RBARM2600.Plot_Cord_3D( Tin, L);
            l1.LineStyle = '-';   l1.LineWidth = 2; 
            l2.LineStyle = '-';   l2.LineWidth = 2; 
            l3.LineStyle = '-';   l3.LineWidth = 2; 
        end
        %% Draw Cylindar Function
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
        %% Draw Cubic Function
        function Draw_Cubic_Link(H,T1)
            % Input the start Point
            % Wid = H / 30;
            wid = 2.5 ;
            Start_point = [ -wid -wid  0] ;
            % Input the start Point
            Final_point = [ wid  wid  H] ;
            vertexIndex = [ 0 0 0; 0 0 1; 0 1 0; 0 1 1; 1 0 0; 1 0 1; 1 1 0; 1 1 1] ;
            cuboidSize  = Final_point-Start_point ;
            vertex      = repmat(Start_point,8,1) + vertexIndex.*repmat(cuboidSize,8,1) ;
            % Define vertex according to 6 faces respectively
            
            facet = [ 1 2 4 3; 1 2 6 5; 1 3 7 5; 2 4 8 6; 3 4 8 7; 5 6 8 7];
            color = [ 0; 1; 0; 1; 0; 1; 0; 1] ;
            Vertex_new = vertex.*0;
            for i = 1 : 8
                p_new = T1 * [vertex(i,:)' ; 1];
                
                Vertex_new(i,:) = p_new(1:3)';
            end
            patch('Vertices',Vertex_new,'Faces',...
                facet,'FaceVertexCData',color,'FaceColor','interp','FaceAlpha',0.5);
        end
        %% Plot Sphere
        function PlotSphere(O,R)
            [x,y,z] = sphere;
            CO(:,:,1) = z.* 0 ;
            CO(:,:,2) = z.* 0 + 1;
            CO(:,:,3) = z.* 0 ;
            S_Sphere = surf( R * x + O(1), R * y + O(2), R * z + O(3),CO);
            
            S_Sphere.FaceAlpha = 0.2 ;  S_Sphere.EdgeColor = 'none';
        end
        %% Draw Half Sphere Function
        function [varargout] = DrawHalfSPhere(R_sphere,T_in)             
            color = [0 0.5 0.5];
            N = 50 ; N2 = N * N ;
            r = linspace(0, 1, N) ;
            theta = linspace(0, pi*2, N) ;
            [R,THETA] = meshgrid(r,theta);
            X =  R.* cos(THETA) ;
            Y =  R.* sin(THETA) ;
            Z = sqrt (1+1e-5 - X.^2 -Y.^2 ) ;
            X = X *R_sphere ; Y = Y *R_sphere ;
            Z = Z *R_sphere ;
            
            ColM=zeros(N,N,3);        ColM(:,:,1) =  color(1) ;
            ColM(:,:,2) =  color(2) ; ColM(:,:,3) = color(3) ;
            
            P_old = [X(:), Y(:), Z(:) , ones(N2,1)] ;
            P_new = P_old.*0 ;
            for k = 1 : N2
                P_new(k,:) = (T_in * P_old(k,:)')' ;
            end
            X_new = reshape(P_new(:,1), size(X)) ;
            Y_new = reshape(P_new(:,2), size(X)) ;
            Z_new = reshape(P_new(:,3), size(X)) ;
            S = surf(X_new, Y_new, Z_new, ColM);
            S.FaceAlpha = 0.5 ;
            shading interp
            varargout{1} = X_new;
            varargout{2} = Y_new;
            varargout{3} = Z_new ;
        end
        function [varargout] = DrawDisk( R, T_disk, axiss)
            Off = T_disk(1:3,4) ;
            if axiss == 'X'
                N_v = T_disk(1:3,1) ;
            elseif axiss == 2
                N_v = T_disk(1:3,2) ;
            elseif axiss == 3
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
            T_new =   Funlib.Rotd( 3, gamma) * Funlib.Rotd( 2, beta) ;
            x_new = x .* 0 ;  y_new = y .* 0 ;  z_new = z .* 0 ;
            for k = 1 : 600
                p_new = T_new * [x(k) y(k) z(k) 1]' ;
                x_new(k) =  p_new(1); y_new(k) =  p_new(2);
                z_new(k) =  p_new(3);
            end
            x_new = x_new +  Off(1) ; y_new = y_new +  Off(2) ;
            z_new = z_new +  Off(3) ;
            CO(:,:,1) = z_new.* 0 + 0.8 ;
            CO(:,:,2) = z_new.* 0;
            CO(:,:,3) = z_new.* 0 ;
            S = surf(x_new, y_new, z_new, CO) ; shading interp
            S.FaceAlpha = 0.4 ;
            varargout{1} = x_new ; varargout{2} = y_new;
            varargout{3} = z_new ;
        end
        
        
        
        %%  Function to Update Point Coordinates via Tin Matrix
        function [Points_new] = Updata_Coordinate(Points,Tin)
            Points_new = ...
                (Tin * [Points' ; ones( 1, size(Points,1) )])' ;
            Points_new(:,4) = [] ;
        end
        
        
        
        
        %%
        function [P_C_S] = Concrete_Cross_Section(w,h)
            x_cross_section = [h h -h -h h];
            y_cross_section = [w -w -w w w] ;
            z_cross_section = [0  0  0 0 0] ;
            P_C_S = ...
                [x_cross_section ;
                y_cross_section ;
                z_cross_section] ;
        end
        
        %% CDF Function for Integration
        function [D] = CDF_Int(X)
            %CDF create a differential matrix
            len = length(X);
            Di = [ 1,        1:len-1,           2:len, len];
            Dj = [ 1,          2:len,         1:len-1, len];
            Dv = [-0.5, ones(1, len-1)- 0.5, -ones(1, len-1) + 0.5,   0.5];
            Dv(1,[1 2]) = [-1 1]; Dv(end,[end-1 end]) = [-1 1];
            D = sparse( Di, Dj, Dv, len, len);
        end
        
        function [D_forward] = CDF_Forward(N)
            ik = [1:N-1,1:N-1,N]' ;
            jk = [1:N-1,2:N,N]';
            sk = [-ones(N-1,1);ones(N-1,1);0];
            D_forward = sparse(ik,jk,sk) ;
        end
        % Calculate the transformation according Z axis
        function [T_Z] = Calculate_T_z(N_v)
            Pv1 = [norm(N_v([1 2])) N_v(3)];  Pv1= Pv1 /norm(Pv1);
            Pv2 = N_v([1 2]) / (norm(N_v([1 2])) + 1e-5);
            if N_v(3) < 0
                beta = 180 - asind(Pv1(1)) ;
            else
                beta =  asind(Pv1(1)) ;
            end
            gamma = atan2d(Pv2(2), Pv2(1)) ;
            T_Z = rotz(  gamma) * roty(  beta) ;
            %             T_Z =   [[T_Z, zeros(3,1)] ;zeros(1,3) 1] ;
        end
        
        function PlotSTLModal(Link, Trans, ColorV)
            ConP = Link.Points ;
            ConT = Link.ConnectivityList ;
            ConP =  RBARM2600.Updata_Coordinate(ConP, Trans);
            ColorM = Link.Points.*0 + ColorV ;
            P = patch( 'Faces',ConT,...
                'Vertices', ConP,'FaceVertexCData',ColorM,'FaceColor','interp') ;
            P.EdgeColor = 'none' ;  P.FaceAlpha = 0.5 ;
        end
        
        function [STLData] = ImportSTL
            Temp.Link = [] ;
            STLData= repmat(Temp,[7 1]) ;
            for i=1:7
                fName=  sprintf('Modal/link%d.STL',i-1);
                STLData(i).Link = stlread(fName);
            end
        end
        
        function DemoRobot(JointAngle)
            % function predefine
            PSTL = @ RBARM2600.PlotSTLModal ;      
            Tcal =  @ RBARM2600.TransMatrixCalculation ;
            PCB = @ RBARM2600.Plot_Cord_Base_3D ;
            Tbase = transl(0,0,300) ; 
%             T_all = Tcal(Q0,Tbase) ;
            PlatForm = stlread('Modal/PlatForm.STL') ;
            WorkingSpace = stlread('Modal/WorkingSpace.STL') ;             
            STLData = RBARM2600.ImportSTL ;
            % new version for importing stl file
            Link0 = STLData(1).Link ;
            Link1 = STLData(2).Link ;
            Link2 = STLData(3).Link ;
            Link3 = STLData(4).Link ;
            Link4 = STLData(5).Link ;
            Link5 = STLData(6).Link ;
            Link6 = STLData(7).Link ;
            % old version for importing stl file
%             Link0 = stlread('Modal/link0.STL') ;
%             Link1 = stlread('Modal/link1.STL') ;
%             Link2 = stlread('Modal/link2.STL') ;
%             Link3 = stlread('Modal/link3.STL') ;
%             Link4 = stlread('Modal/link4.STL') ;
%             Link5 = stlread('Modal/link5.STL') ;
%             Link6 = stlread('Modal/link6.STL') ;


            Colv = [0.5 0.7 0.8 ] ; 
            Numframe =size(JointAngle,1) ;
            for i = 1: Numframe
            clf ; hold on
            T_all = Tcal(JointAngle(i,:),Tbase) ;
            PSTL(PlatForm, transl(0,0,0), [0.1 0.1 0.1]) ;  
            PSTL(WorkingSpace, transl(900,0,0), [0 0 0]) ; 
            PSTL(Link0, T_all.T0b, Colv);  
            PSTL(Link1, T_all.T10, Colv);
            PSTL(Link2, T_all.T20* transl(0,0,-145) , Colv)
            PSTL(Link3, T_all.T30* transl(0,0,-155), Colv)
            PSTL(Link4, T_all.T40, Colv)
            PSTL(Link5, T_all.T50, Colv)
            PSTL(Link6, T_all.T60, Colv) ;   PCB(transl(0,0,0),300) ; 
            
            axis equal ;  light;  view( 130, 25); drawnow;
            
            
            end
            
        end
        
        
        %% Part III Code Script 
        
        function  OutputRapid(Targetpoints)
          % this function is to create rapid script
          % function predefine
          Rotd = @ RBARM2600.Rotation_Degree ;
          IK = @ RBARM2600.InverseKinematics ;
          Ttool60 = transl( 0,  0, 0) ;
          % 
          Rori = Rotd( 2, 0)  ;           
          Nump = size( Targetpoints, 1)  ;           
          JQ = zeros( Nump, 6) ;
          Q146 = zeros( Nump, 4) ;
          for i = 1 : Nump
            P60 =  Ttool60   \  [Targetpoints(i,:) , 1]'  ;
            Ptar = P60(1:3)' ;
            JQ(i,:) = IK(Ptar, Rori) ;
            Q146(i,1:3) = floor(JQ(i,[1 4 6]) /90 + 4 ) -4 ;
          end
          Cfg = Q146 ;  Cfg(:,4) = 1 ;
          
          %%  Section III Script Editing to a txt file
          temp.line=[] ;   Script = repmat(temp,[Nump 1 ]);
          Statement1_1 = sprintf('VAR robtarget Path{%3d} ;', Nump ) ;
          Statement1_2 = sprintf('VAR num PAll:= %d ;', Nump ) ;
          Statement1_3 = sprintf('VAR num Pind ;') ;
          Statement2_3 = 'FOR Pind FROM 1 TO PAll DO';
          Statement2_2 = 'MoveL Path{Pind},v100,z5,tool0;';
          Statement2_1 = 'ENDFOR';
          
          for iline = 1 : Nump
            Parm1 = sprintf('[[%4.5f, %4.5f, %4.5f],', Targetpoints(iline,:)) ;
            Parm2 = sprintf('[%4.5f, %4.5f, %4.5f, %4.5f],', [0 0 1 0]) ;
            Parm3 = sprintf('[%4.5f, %4.5f, %4.5f, %4.5f],', Cfg(iline,:)) ;
            Parm4 = '[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]] ;';
            P_all = ['Path{%3d}:=',Parm1, Parm2, Parm3, Parm4] ;
            Script(iline).line = sprintf(P_all, iline  ) ;
          end
          %%  Editing Manuscript
          Lines_all = size(Script,1) ;
          FilePointer = fopen('RapidScript.txt','w') ;
          
          fprintf(FilePointer, [Statement1_1,'\n']) ;
          fprintf(FilePointer, [Statement1_2,'\n']) ;
          fprintf(FilePointer, [Statement1_3,'\n']) ;
          for iline = 1: Lines_all
            fprintf(FilePointer, [Script(iline).line,'\n']) ;
          end
          
          fprintf(FilePointer, [Statement2_3,'\n']) ;
          fprintf(FilePointer, [Statement2_2,'\n']) ;
          fprintf(FilePointer, [Statement2_1,'\n']) ;
          
          
          fclose(FilePointer);
          
        end

        function OutputMOveABJ(Filename, JointP)
            Nump = size( JointP, 1)  ;
            FrontStatement{1} = sprintf('VAR jointtarget JointPabs{%3d} ;\n', Nump ) ;
            FrontStatement{2} = sprintf('VAR num PAllabs:= %d ;\n', Nump ) ;
            FrontStatement{3} = 'VAR num Pindabs ;\n';
            EndStatement{1} = 'FOR Pindabs FROM 1 TO PAllabs DO';
            EndStatement{2} = 'MoveAbsJ JointPabs{Pindabs},v100,fine,tool0;';
            EndStatement{3} =  'ENDFOR';
            Mainbody = cell(Nump,1) ;
            for ind = 1: Nump
                part1 = sprintf('JointPabs{%3d} := [', ind) ;
                part2 = sprintf('[%4.5f, %4.5f, %4.5f, %4.5f, %4.5f, %4.5f]', JointP(ind,:) ) ;
                part3 = ',[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; ' ;
                Mainbody{ind} = [part1,part2,part3,'\n'] ;
            end
            FilePointer = fopen(Filename,'w') ;
            for line = 1:3
                fprintf(FilePointer, FrontStatement{line}) ;
            end
            for line = 1: Nump
                fprintf(FilePointer, Mainbody{line}) ;
            end
            for line = 1:3
                fprintf(FilePointer, [EndStatement{line},'\n']) ;
            end
            fclose(FilePointer);


        end


        
        

        
    end
    
end

