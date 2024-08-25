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
****        Class for the Recursive Newton-Euler Algorithm         ****
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
classdef NewtonEulerRec < IRB2600Real
  %{
***********************************************************************
  Summary of the Class:
  
***********************************************************************
* Properties:                                                         *
*                                                                     *
* Functions:                                                          *
*  NewtonEulerRec.DemoVelocity   ---   To demo the velocity           *
*  NewtonEulerRec.LinkPoint   ---   Calculate the Link Points         *
*  NewtonEulerRec.ForwardRecursive ---  Calculate the velocity        *
*  NewtonEulerRec.RotInertiaCm   ---  Calculate the Inertial Matrix   *
*  NewtonEulerRec.NetForce   ---  Calculate the Net Force for Link    *
*---------------------------------------------------------------------*
  %}
  methods(Static)
    %{
***********************************************************************
* NewtonEulerRec.DemoVelocity ::                                      *
* This function is to compare the velocity and acceleration calculted *
* via Differential method and Newton-Euler method                     *
*                                                                     *
* of mass for body {i} in Frame i coordinates.                        *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'Time' : The Time Vector                                           *
*  'dPlink' : A Cell contains velocity in all links calculated by     *
differential                                                          *
*  'Vworld': The velocity in world frame                              *
*  'AxisNumber' : The number index of axis in robot arm               *
* OUTPUT:                                                             *
*   No output                                                         *
* WARNING: none                                                       *
* History: 07/23/2024, edited                                         *
***********************************************************************
    %}
    function DemoVelocity(Time,DiffCal, NECal ,AxisNumber)
      plot(Time, DiffCal{AxisNumber}(:,1), 'r') ;
      plot(Time, DiffCal{AxisNumber}(:,2), 'g') ;
      plot(Time, DiffCal{AxisNumber}(:,3), 'b') ;
      plot(Time, NECal{AxisNumber}(:,1), 'or') ;
      plot(Time, NECal{AxisNumber}(:,2), 'og') ;
      plot(Time, NECal{AxisNumber}(:,3), 'ob') ;
    end
    %{
***********************************************************************
* NewtonEulerRec.LinkPoint ::                                         *
* This function is to compare the velocity and acceleration calculted *
* via Differential method and Newton-Euler method                     *
*                                                                     *
* of mass for body {i} in Frame i coordinates.                        *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'Time' : The Time Vector                                           *
*  'dPlink' : A Cell contains velocity in all links calculated by     *
differential                                                          *
*  'Vworld': The velocity in world frame                              *
*  'AxisNumber' : The number index of axis in robot arm               *
* OUTPUT:                                                             *
*   No output                                                         *
* WARNING: none                                                       *
* History: 07/23/2024, edited                                         *
***********************************************************************
    %}
    function Plink = LinkPoint(Q_theta, Linknum)
      TCell = ROBDEMO.Tall_To_Cell(Q_theta) ;
      Plink = TCell{Linknum}(1:3,4)' ;
    end
    
    %{
***********************************************************************
* NewtonEulerRec.ForwardRecursive ::                                  *
* This function is to compare the velocity and acceleration calculted *
* via Differential method and Newton-Euler method                     *
*                                                                     *
* of mass for body {i} in Frame i coordinates.                        *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'JointP' : The Time Vector                                         *
*  'dt' :  a small value of time internel time   differential         *
*---------------------------------------------------------------------*
* OUTPUT:                                                             *
*  'Vlink': The velocity reference to the link frame via NR rec       *
*  'Vworld': The velocity reference to the world frame via NR rec     *
*  'Alink': The accelerationreference to the link frame via NR rec    *
*  'Aworld': The acceleration reference to the world frame via NR rec *
*                                                                     *
* WARNING:                                                            *
*  Vlink, Vworld, Alink, Aworld are included the linear               *
*  and angular velocity  according to the DH parameter table          *
* History: 07/29/2024, edited                                         *
***********************************************************************
    %}
    function [Vlink, Vworld,Alink,Aworld] = ForwardRec(JointP,dt)
      CDF = @ NewtonEulerRec.CDF_Int ;
      Tcal = @ NewtonEulerRec.TransMatrixCalculation ;
      Nump = size(JointP,1) ;
      Vlink = cell(1,6) ;
      Vworld = cell(1,6) ;
      Alink = cell(1,6) ;
      Aworld = cell(1,6) ;
      
      DH = ROBCal.GetDH ;
      Dm = CDF(JointP) ;
      for i = 1:6
        Vlink{i}  =  zeros(Nump, 6) ;
        Vworld{i} =  zeros(Nump, 6) ;
        Alink{i}  =  zeros(Nump, 6) ;
        Aworld{i} = zeros(Nump, 6) ;
      end
      
      deg = 180/pi ; rad = 1/ deg ;
      % JointS is the Joint speed calculated from the finite central differential
      JointS = Dm * JointP /dt * rad ;
      % JointA is the Joint acceleration calculated from the finite central differential
      JointA = Dm * JointS/dt ;
      r1 = [0 0 DH(1,4)] ;    r2 = [DH(2,2) 0 0] ;
      r3 = [DH(3,2) 0 0] ;    r4 = [DH(4,2) DH(4,4) 0] ;
      r5 = [0 0 0] ;          r6 = [0 -DH(6,4) 0] ;
      for ind = 1 : Nump
        % ind = 100 ;
        %% Calculate the current robot pose and orientation.
        q = JointP(ind,:) ;
        Tind = Tcal(q) ;
        R10 = Tind.T10(1:3,1:3) ;
        R21 = Tind.T21(1:3,1:3) ; R20 = Tind.T20(1:3,1:3) ;
        R30 = Tind.T30(1:3,1:3) ; R32 = Tind.T32(1:3,1:3) ;
        R40 = Tind.T40(1:3,1:3) ; R43 = Tind.T43(1:3,1:3) ;
        R54 = Tind.T54(1:3,1:3) ; R50 = Tind.T50(1:3,1:3) ;
        R65 = Tind.T65(1:3,1:3) ; R60 = Tind.T60(1:3,1:3) ;
        %% Velocity Calculation
        V0 = zeros(1, 6) ;
        V1 = zeros(1, 6) ;  V2 = zeros(1, 6) ; V3 = zeros(1, 6) ;
        V4 = zeros(1, 6) ;  V5 = zeros(1, 6) ; V6 = zeros(1, 6) ;
        %%% Calculation the velocity in Link_1 %%%
        V1(1:3) =  (V0(1:3) + cross(V0(4:6), r1)) * R10 ;
        V1(4:6) =  V0(4:6) * R10 + [0 0 JointS(ind,1) ]  ;
        %%% Calculation the velocity in Link_2 %%%
        V2(1:3) =  (V1(1:3) + cross(V1(4:6), r2) )* R21 ;
        V2(4:6) =   V1(4:6) * R21 + [0 0 JointS(ind,2)] ;
        %%% Calculation the velocity in Link_3 %%%
        V3(4:6) =  V2(4:6) * R32 + [0 0 JointS(ind,3)] ;
        V3(1:3) = ( V2(1:3) + cross(V2(4:6), r3) )* R32 ;
        %%% Calculation the velocity in Link_4 %%%
        V4(4:6) =  V3(4:6) * R43 + [0 0 JointS(ind,4)] ;
        V4(1:3) = ( V3(1:3) + cross(V3(4:6), r4) )* R43 ;
        %%% Calculation the velocity in Link_5 %%%
        V5(4:6) = V4(4:6) * R54 + [0 0 JointS(ind,5)] ;
        V5(1:3) = ( V4(1:3) + cross(V4(4:6), r5) )* R54 ;
        %%% Calculation the velocity in Link_6 %%%
        V6(4:6) = V5(4:6) * R65 + [0 0 JointS(ind,6)] ;
        V6(1:3) = ( V5(1:3) + cross(V5(4:6), r6) ) * R65 ;
        %%% record the volocity in each link frame.
        Vlink{1}(ind,:) = V1 ;  Vlink{2}(ind,:) = V2 ;
        Vlink{3}(ind,:) = V3 ;  Vlink{4}(ind,:) = V4 ;
        Vlink{5}(ind,:) = V5 ;  Vlink{6}(ind,:) = V6 ;
        %%% record the volocity in the base frame.
        Vworld{1}(ind,1:3) = Vlink{1}(ind,1:3) * R10' ;
        Vworld{2}(ind,1:3) = Vlink{2}(ind,1:3) * R20' ;
        Vworld{3}(ind,1:3) = Vlink{3}(ind,1:3) * R30' ;
        Vworld{4}(ind,1:3) = Vlink{4}(ind,1:3) * R40' ;
        Vworld{5}(ind,1:3) = Vlink{5}(ind,1:3) * R50' ;
        Vworld{6}(ind,1:3) = Vlink{6}(ind,1:3) * R60' ;
        %% Acceleration Calculation
        A0 = zeros(1,6) ;
        A1 = zeros(1,6) ; A2 = zeros(1,6) ; A3 = zeros(1,6) ;
        A4 = zeros(1,6) ; A5 = zeros(1,6) ; A6 = zeros(1,6) ;
        %%% Calculation the acceleration in Link_1 %%%
        A1(1:3) =(A0(1:3) + cross(A0(4:6), r1 ) + ...
          cross (  V0(4:6), cross(V0(4:6), r1 ) ) )  * R10 ;
        A1(4:6) =  A0(4:6) * R10 + [0 0 JointA(ind,1)] + ...
          cross( (V0(4:6) * R10 ) ,[0 0 JointS(ind,1)]) ;
        %%% Calculation the acceleration in Link_2 %%%
        A2(1:3) =(A1(1:3) + cross(A1(4:6), r2 ) + ...
          cross (  V1(4:6), cross(V1(4:6), r2 ) ) )  * R21 ;
        A2(4:6) =  A1(4:6) * R21 + [0 0 JointA(ind,2)] + ...
          cross( (V1(4:6) * R21 ) ,[0 0 JointS(ind,2)]) ;
        %%% Calculation the acceleration in Link_3 %%%
        A3(1:3) =(A2(1:3) + cross(A2(4:6), r3 ) + ...
          cross (  V2(4:6), cross(V2(4:6), r3 ) ) )  * R32 ;
        A3(4:6) =  A2(4:6) * R32 + [0 0 JointA(ind,3)] + ...
          cross( (V2(4:6) * R32 ) ,[0 0 JointS(ind,3)]) ;
        %%% Calculation the acceleration in Link_4 %%%
        A4(1:3) =(A3(1:3) + cross(A3(4:6), r4 ) + ...
          cross (  V3(4:6), cross(V3(4:6), r4 ) ) )  * R43 ;
        A4(4:6) =  A3(4:6) * R43 + [0 0 JointA(ind,4)] + ...
          cross( (V3(4:6) * R43 ) ,[0 0 JointS(ind,4)]) ;
        %%% Calculation the acceleration in Link_5 %%%
        A5(1:3) =(A4(1:3) + cross(A4(4:6), r5 ) + ...
          cross (  V4(4:6), cross(V4(4:6), r5 ) ) )  * R54 ;
        A5(4:6) =  A4(4:6) * R54 + [0 0 JointA(ind,5)] + ...
          cross( (V4(4:6) * R54 ) ,[0 0 JointS(ind,5)]) ;
        %%% Calculation the acceleration in Link_6 %%%
        A6(1:3) =(A5(1:3) + cross(A5(4:6), r6 ) + ...
          cross (  V5(4:6), cross(V5(4:6), r6 ) ) )  * R65 ;
        A6(4:6) =  A5(4:6) * R65 + [0 0 JointA(ind,6)] + ...
          cross( (V5(4:6) * R65 ) ,[0 0 JointS(ind,6)]) ;
        %%% record the acceleration in each link frame.
        Alink{1}(ind,:) = A1 ; Alink{2}(ind,:) = A2 ;
        Alink{3}(ind,:) = A3 ; Alink{4}(ind,:) = A4 ;
        Alink{5}(ind,:) = A5 ; Alink{6}(ind,:) = A6 ;
        %%% record the acceleration in the base frame.
        Aworld{1}(ind,1:3) = Alink{1}(ind,1:3) * R10' ;
        Aworld{2}(ind,1:3) = Alink{2}(ind,1:3) * R20' ;
        Aworld{3}(ind,1:3) = Alink{3}(ind,1:3) * R30' ;
        Aworld{4}(ind,1:3) = Alink{4}(ind,1:3) * R40' ;
        Aworld{5}(ind,1:3) = Alink{5}(ind,1:3) * R50' ;
        Aworld{6}(ind,1:3) = Alink{6}(ind,1:3) * R60' ;
      end

    end
    
    
    
    
    %{
***********************************************************************
* NewtonEulerRec.ForwardRecursive ::                                  *
* This function is to compare the velocity and acceleration calculted *
* via Differential method and Newton-Euler method                     *
*                                                                     *
* of mass for body {i} in Frame i coordinates.                        *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'JointP' : The Time Vector                                         *
*  'dt' :  a small value of time internel time   differential         *
*---------------------------------------------------------------------*
* OUTPUT:                                                             *
*  'Vlink': The velocity reference to the link frame via NR rec       *
*  'Vworld': The velocity reference to the world frame via NR rec     *
*  'Aworld': The acceleration reference to the world frame via NR rec *
*  'Plink': The points in all links (a cell format)                   *
*  'dPlink': The velocity in all links via differential method        *
*  'ddPlink': The acceleration in all links via differential method   *
*                                                                     *
* WARNING:                                                            *
*  Vlink, Vworld,  Aworld  are included the linear                    *
*  and angular velocity                                               *
* History: 07/27/2024, edited                                         *
***********************************************************************
    %}
    function [Vlink, Vworld, Alink, Aworld, Plink, dPlink, ddPlink] ...
        = ForwardRecursive(JointP,dt)
      CDF = @ NewtonEulerRec.CDF_Int ;
      Tcal = @ NewtonEulerRec.TransMatrixCalculation ;
      Nump = size(JointP,1) ;
      Dm = CDF(JointP) ;
      Plink = cell(1,6) ;
      dPlink = cell(1,6) ;
      ddPlink = cell(1,6) ;
      Vlink = cell(1,6) ;
      Vworld = cell(1,6) ;
      Alink = cell(1,6) ;
      Aworld = cell(1,6) ;
      for i = 1:6
        Plink{i}  =  zeros(Nump, 3) ;
        dPlink{i}  =  zeros(Nump, 3) ;
        ddPlink{i}  =  zeros(Nump, 3) ;
        Vlink{i}  =  zeros(Nump, 6) ;
        Vworld{i} =  zeros(Nump, 6) ;
        Alink{i}  =  zeros(Nump, 6) ;
        Aworld{i} = zeros(Nump, 6) ;
      end
      deg = 180/pi ; rad = 1/ deg ;
      % JointS is the Joint speed calculated from the finite central differential
      JointS = Dm * JointP /dt * rad ;
      % JointA is the Joint acceleration calculated from the finite central differential
      JointA = Dm * JointS/dt ;
      r1 = [150 0 0] ;
      r2 = [700 0 0] ;
      r3 = [115 200 0] ;
      r4 = [0 0 595] ;
      r5 = [0 -85 0] ;
      %vb = [0 0 0 0 0 0] ;
      for ind = 1 : Nump
        q = JointP(ind,:) ;
        %% Calculate the current robot pose and orientation.
        Tind = Tcal(q) ;
        R10 = Tind.T10(1:3,1:3) ;
        R21 = Tind.T21(1:3,1:3) ; R20 = Tind.T20(1:3,1:3) ;
        R30 = Tind.T30(1:3,1:3) ; R32 = Tind.T32(1:3,1:3) ;
        R40 = Tind.T40(1:3,1:3) ; R43 = Tind.T43(1:3,1:3) ;
        R54 = Tind.T54(1:3,1:3) ; R50 = Tind.T50(1:3,1:3) ;
        R65 = Tind.T65(1:3,1:3) ; R60 = Tind.T60(1:3,1:3) ;
        %% Velocity Calculation
        V1 = zeros(1, 6) ;  V2 = zeros(1, 6) ; V3 = zeros(1, 6) ;
        V4 = zeros(1, 6) ;  V5 = zeros(1, 6) ; V6 = zeros(1, 6) ;
        %%% Calculation the velocity in Link_1 %%%
        V1(1:3) = [0 0 0] ;         V1(4:6) = [0 0 JointS(ind,1) ] ;
        %%% Calculation the velocity in Link_2 %%%
        V2(1:3) =(V1(1:3) + cross(V1(4:6), r1) )* R21 ;
        V2(4:6) =   V1(4:6) * R21 + [0 0 JointS(ind,2)] ;
        %%% Calculation the velocity in Link_3 %%%
        V3(4:6) =  V2(4:6) * R32 + [0 0 JointS(ind,3)] ;
        V3(1:3) = ( V2(1:3) + cross(V2(4:6), r2) )* R32 ;
        %%% Calculation the velocity in Link_4 %%%
        V4(4:6) =  V3(4:6) * R43 + [0 0 JointS(ind,4)] ;
        V4(1:3) = ( V3(1:3) + cross(V3(4:6), r3) )* R43 ;
        %%% Calculation the velocity in Link_5 %%%
        V5(4:6) = V4(4:6) * R54 + [0 0 JointS(ind,5)] ;
        V5(1:3) = ( V4(1:3) + cross(V4(4:6), r4) )* R54 ;
        %%% Calculation the velocity in Link_6 %%%
        V6(4:6) = V5(4:6) * R65 + [0 0 JointS(ind,6)] ;
        V6(1:3) = ( V5(1:3) + cross(V5(4:6), r5) ) * R65 ;
        %%% record the volocity in each link frame.
        Vlink{1}(ind,:) = V1 ;  Vlink{2}(ind,:) = V2 ;
        Vlink{3}(ind,:) = V3 ;  Vlink{4}(ind,:) = V4 ;
        Vlink{5}(ind,:) = V5 ;  Vlink{6}(ind,:) = V6 ;
        %%% record the volocity in the base frame.
        Vworld{1}(ind,1:3) = Vlink{1}(ind,1:3) * R10' ;
        Vworld{2}(ind,1:3) = Vlink{2}(ind,1:3) * R20' ;
        Vworld{3}(ind,1:3) = Vlink{3}(ind,1:3) * R30' ;
        Vworld{4}(ind,1:3) = Vlink{4}(ind,1:3) * R40' ;
        Vworld{5}(ind,1:3) = Vlink{5}(ind,1:3) * R50' ;
        Vworld{6}(ind,1:3) = Vlink{6}(ind,1:3) * R60' ;
        %% Acceleration Calculation
        A1 = zeros(1,6) ; A2 = zeros(1,6) ; A3 = zeros(1,6) ;
        A4 = zeros(1,6) ; A5 = zeros(1,6) ; A6 = zeros(1,6) ;
        %%% Calculation the acceleration in Link_1 %%%
        A1(1:3) = [0 0 0] ; A1(4:6) = [0 0 JointA(ind,1) ] ;
        %%% Calculation the acceleration in Link_2 %%%
        A2(1:3) =(A1(1:3) + cross(A1(4:6), r1 ) + ...
          cross (  V1(4:6), cross(V1(4:6), r1 ) ) )  * R21 ;
        A2(4:6) =  A1(4:6) * R21 + [0 0 JointA(ind,2)] + ...
          cross( (V1(4:6) * R21 ) ,[0 0 JointS(ind,2)]) ;
        %%% Calculation the acceleration in Link_3 %%%
        A3(1:3) =(A2(1:3) + cross(A2(4:6), r2 ) + ...
          cross (  V2(4:6), cross(V2(4:6), r2 ) ) )  * R32 ;
        A3(4:6) =  A2(4:6) * R32 + [0 0 JointA(ind,3)] + ...
          cross( (V2(4:6) * R32 ) ,[0 0 JointS(ind,3)]) ;
        %%% Calculation the acceleration in Link_4 %%%
        A4(1:3) =(A3(1:3) + cross(A3(4:6), r3 ) + ...
          cross (  V3(4:6), cross(V3(4:6), r3 ) ) )  * R43 ;
        A4(4:6) =  A3(4:6) * R43 + [0 0 JointA(ind,4)] + ...
          cross( (V3(4:6) * R43 ) ,[0 0 JointS(ind,4)]) ;
        %%% Calculation the acceleration in Link_5 %%%
        A5(1:3) =(A4(1:3) + cross(A4(4:6), r4 ) + ...
          cross (  V4(4:6), cross(V4(4:6), r4 ) ) )  * R54 ;
        A5(4:6) =  A4(4:6) * R54 + [0 0 JointA(ind,5)] + ...
          cross( (V4(4:6) * R54 ) ,[0 0 JointS(ind,5)]) ;
        %%% Calculation the acceleration in Link_6 %%%
        A6(1:3) =(A5(1:3) + cross(A5(4:6), r5 ) + ...
          cross (  V5(4:6), cross(V5(4:6), r5 ) ) )  * R65 ;
        A6(4:6) =  A5(4:6) * R65 + [0 0 JointA(ind,6)] + ...
          cross( (V5(4:6) * R65 ) ,[0 0 JointS(ind,6)]) ;
        %%% record the acceleration in each link frame.
        Alink{1}(ind,:) = A1 ; Alink{2}(ind,:) = A2 ;
        Alink{3}(ind,:) = A3 ; Alink{4}(ind,:) = A4 ;
        Alink{5}(ind,:) = A5 ; Alink{6}(ind,:) = A6 ;
        %%% record the acceleration in the base frame.
        Aworld{1}(ind,1:3) = Alink{1}(ind,1:3) * R10' ;
        Aworld{2}(ind,1:3) = Alink{2}(ind,1:3) * R20' ;
        Aworld{3}(ind,1:3) = Alink{3}(ind,1:3) * R30' ;
        Aworld{4}(ind,1:3) = Alink{4}(ind,1:3) * R40' ;
        Aworld{5}(ind,1:3) = Alink{5}(ind,1:3) * R50' ;
        Aworld{6}(ind,1:3) = Alink{6}(ind,1:3) * R60' ;
      end
      %% This is a comparative velocity calcuation.
      for i = 1: Nump
        for j = 1: 6
          Plink{j}(i,:) = NewtonEulerRec.LinkPoint(JointP(i,:),j) ;
        end
      end
      
      for i = 1:6
        dPlink{i}  = Dm * Plink{i} /dt ;
        ddPlink{i} = Dm * dPlink{i} /dt ;
      end
    end
    
    
    
    %{
*************************************************************************
* NewtonEulerRec.RotInertiaCm ::
* This function is to calculate the rotational inertia about center
* of mass for body {i} in Frame i coordinates.
*-----------------------------------------------------------------------
* INPUT:
*  'Mass' : the mass of link i
*  'MassCenter' : the center of mass in Link i
* OUTPUT:
*   'Icm' :  rotational inertia
*    Icm = [Ixx -Ixy -Ixz ;
*    *   Iyy -Iyz ;
*    *    *   Izz ;   ]
* WARNING:
    This is wrong
* History: 07/26/2024, edited
*************************************************************************
    %}
    
    function Icm = RotInertiaCm( Mass, MassCenter)
      cx = MassCenter(1) ;   % x component of mass center
      cy = MassCenter(2) ;   % y component of mass center
      cz = MassCenter(3) ;   % z component of mass center
      MassCenter = zeros(3,3) ;
      % Calculation of the inertia matrix
      MassCenter(1,1) = norm([cy cz]) ; % Calculating Ixx
      MassCenter(1,2) = -cx*cy ; % Calculating  -Ixy
      MassCenter(2,1) =  MassCenter(1,2) ; % symmetric
      MassCenter(1,3) = -cx*cz ; % Calculating  -Ixy
      MassCenter(3,1) = MassCenter(1,3)  ; % symmetric
      MassCenter(2,2) = norm([cx cz]) ;
      MassCenter(2,3) = -cy*cz ; % Calculating  -Iyz
      MassCenter(3,2) = MassCenter(2,3) ; % symmetric
      MassCenter(3,3) = norm([cx cy]) ;
      Icm = Mass*MassCenter ;
    end
    %{
*************************************************************************
* NewtonEulerRec.NetForce ::                                            *
* This function is to calculate the net force for body {i}              *
* in Frame i coordinates.                                               *
*-----------------------------------------------------------------------*
* INPUT:                                                                *
*  'Mass' : the mass of link i                                          *
*  'MassCenter' : the center of mass in Link i                          *
*  'GeneralVel'  : General Velocity including linear and angular.       *
*  'GeneralAcc'  : General acceleration includint linear and angular.   *
* OUTPUT:                                                               *
*  'GernealForce' :  the netforce required w.r.t.
*   current speed and acceleration.
*  GernealForce = [Fx Fy Fz Mx My Mz] ;
* WARNING:
*  This function is written 
*  The size of 'GernealForce' should be equal to 1x6, a column vector;
* History: 07/29/2024, edited
*************************************************************************
    %}
    function  GernealForce = NetForce(Mass, MassCenter, IMO, GeneralVel, GeneralAcc)
      %  Velocity, Acceleration, Omega, Alpha
      % Get the linear and angular velocity
      Velocity = GeneralVel(1:3) ; Omega = GeneralVel(4:6) ;
      % Get the linear and angular acceleration.
      Acceleration = GeneralAcc(1:3) ; Alpha = GeneralAcc(4:6) ;

        fnet = Mass * (Acceleration +  cross( Alpha , MassCenter ) +...
    cross( Omega, cross(Omega , MassCenter ))) ;
  mnet = Alpha*IMO + cross(Alpha, Alpha*IMO ) ;
  GernealForce = [fnet, mnet] ;

    end
    
    
    
    
  end
end

