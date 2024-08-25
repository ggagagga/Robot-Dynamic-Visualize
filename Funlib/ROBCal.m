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
****        Class for the Bacis Calculation in Robot Arm           ****
***********************************************************************
*---------------------------------------------------------------------*
*                   Start date:    July 26 2024                       *
*                   Last update:   Augest 01 2024                     *
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
classdef ROBCal
  %{
***********************************************************************
  Summary of the Class:
  This Class is created for basic calculation about a robot
  pose and orientation for all links.
  
***********************************************************************
* Properties:                                                         *
*                                                                     *
*  none                                                               *
*                                                                     *
* Methods(Static)                                                     *
*                                                                     *
*  ROBCal.GetDH   ---   To get the DH parameter of a robot arm        *
*  ROBCal.RotationMatrix_Degree  ---   Rotation Matrix                *
*  ROBCal.TransMatrixCalculation  --- Pose and Orientation Struct     *
*  ROBCal.Tall_To_Cell --- Cell format Trans Matrix                   *
*  ROBCal.TraceCalculation  --- Calculate the trace                   *
*                                                                     *
*---------------------   Inverse Kinematics --------------------------*
*  ROBCal.InverseKinematics  --- Calculate 1 ~ 6 Joint Config         *
*  ROBCal.InnerInverseKinematics  --- Calculate 1,2,3 Joint Config    *
*  ROBCal.OuterInverseKinematics  ---  Calculate 4,5 Joint Config     *
*  ROBCal.ImportJointData   --- To Import data                        *
*  ROBCal.ExportJointData   --- To Export data                        *
*  ROBCal.CDF_Int   --- Generati differential Matrix                  *
*  ROBCal.UpdatePoint --- Update the points                           *   
*_____________________________________________________________________*
*

*---------------------------------------------------------------------*
  %}
  methods(Static)
    %{
***********************************************************************
* ROBCal.DemoVelocity ::                                       *
* This function is to compare the velocity and acceleration calculted *
* via Differential method and Newton-Euler method                     *
*                                                                     *
* of mass for body {i} in Frame i coordinates.                        *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*                                                                     *
* OUTPUT:                                                             *
*   'DH' --- The Dehavit-Hartenberg parameters                        *
*                                                                     *
* WARNING: DH is matrix with size of (6,4)                            *
* History: 07/26/2024, edited                                         *
***********************************************************************
    %}
    function DH = GetDH
      DH = [
        0,    0,    0,      445 ;   % offset coordinate of T00
        -90,  150,  -90,     0 ;   % offset coordinate of T10 rotate around z
        0,  700,    0,       0 ;   % offset coordinate of T21 rotate around y
        -90,115,    0,   795 ;   % offset coordinate of T32 rotate around y
        90,    0, -180,        0 ;   % offset coordinate of T43 rotate around x
        90,    0,    0,    85 ;] ;% offset coordinate of End-effecto
    end
    %{
***********************************************************************
* ROBCal.RotationMatrix_Degree ::                             *
* This function is to calculate the rotation transformation matrix    *
*                                                                     *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'AxisNumber' to determine which axis shoule be rotated around      *
*    when axis is 1, generate homogeneous rotation matrix arond x     *
*    when axis is 2, generate homogeneous rotation matrix arond y     *
*    when axis is 3, generate homogeneous rotation matrix arond z     *
* OUTPUT:                                                             *
*   'RotationMatrix' --- The rotation matrix                          *
*                                                                     *
* WARNING:                                                            *
*   RotationMatrix is matrix with size of (4,4)                       *
*   !!!!!!!!!!  the unit is degree !!!!!!!!!!!!!                      *
* History: 07/26/2024, edited                                         *
***********************************************************************
    %}
    function [RotationMatrix] = RotationMatrix_Degree( AxisNumber, Degree)
      %   function Rotr is to create rotation transformation matrix
      %   Input:
      %   Parameter 1:
      %   to determine which axis shoule be rotated around
      %   when axis is 1, generate homogeneous rotation matrix arond x
      %   when axis is 2, generate homogeneous rotation matrix arond y
      %   when axis is 3, generate homogeneous rotation matrix arond z
      %   Parameter2 theta:
      %   This parameter determines how many rotation degree it is
      %   (Unit : Degree)
      l = cosd(Degree) ; m = sind(Degree) ;
      switch AxisNumber
        case 1 % Rotate around x axis
          RotationMatrix = [  1  0  0  0 ;
            0  l -m  0 ;
            0  m  l  0 ;
            0  0  0  1 ; ] ;
        case 2 % Rotate around y axis
          RotationMatrix = [  l  0  m  0 ;
            0  1  0  0 ;
            -m  0  l  0 ;
            0  0  0  1 ; ] ;
        case 3 % Rotate around z axis
          RotationMatrix = [  l -m  0  0 ;
            m  l  0  0 ;
            0  0  1  0 ;
            0  0  0  1 ; ] ;
      end
    end

    %{
***********************************************************************
* ROBCal.QuaternionCal ::                                             *
* This function is calculate Quaternion number to represent the       *
* orientation of a robot.
*                                                                     *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'Rin':  the target pose and orientation matrix reference to world  *
* OUTPUT:                                                             *
*   'Quaternion' ---  a result calculated via Rin  4 x 4              * 
*                                                                     *
* WARNING:                                                            *
*   Rin is reference to the world                                     *
*   !!!!!!!!!!  the unit is degree !!!!!!!!!!!!!                      *
* History: 07/26/2024, edited                                         *
***********************************************************************
    %}

    function Quaternion = QuaternionCal(Rin)
        Rori = Rin(1:3,1:3) ;
        Quaternion = dcm2quat(Rori') ;
    end

    %{
***********************************************************************
* ROBCal.TransMatrixCalculation ::                                    *
* This function is calculate all Transpose matrix with given          *
the current joint configuretion                                       *
*                                                                     *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'JointConfig':   a 1 6 column vector about the                     *
      current robot joint configuration                               *
* OUTPUT:                                                             *
*   'TransAll' --- The struct containing all transformation matrix    *
*                   of a robot arm                                    *
*                                                                     *
* WARNING:                                                            *
*   The size of JointConfig should  is (1,6)                          *
*   !!!!!!!!!!  the unit is degree !!!!!!!!!!!!!                      *
* History: 07/26/2024, edited                                         *
***********************************************************************
    %}
    function [TransAll] = TransMatrixCalculation(JointConfig)
      Rot = @ ROBCal.RotationMatrix_Degree ;
      DH = ROBCal.GetDH  ;
      Tbase = transl(0,0,0) ; %% a base link pose matrix
      TransAll.T0b = Tbase ; %
      TransAll.T10 = transl(0,0,DH(1,4)) * Rot(3, JointConfig(1) ) ;
      TransAll.T10 = TransAll.T0b * TransAll.T10 ;
      TransAll.T21 = Rot(1,DH(2,1)) * transl(DH(2,2),0,0) * ...
        Rot(3,DH(2,3)) * Rot(3, JointConfig(2) )  ;
      TransAll.T32 = transl(DH(3,2), 0, 0) * Rot(3, JointConfig(3) ) ;
      TransAll.T43 = Rot(1,DH(4,1)) * transl(DH(4,2), 0, DH(4,4) )* Rot(3, JointConfig(4) )  ;
      TransAll.T54 = transl(0,  0, 0) ...
        * Rot(1,DH(5,1))   *  Rot(3,DH(5,3))    *Rot(3, JointConfig(5) ) ;
      TransAll.T65 = Rot(1,DH(6,1))  * transl(0,0,DH(6,4)) * Rot(3, JointConfig(6) ) ;
      TransAll.T20 = TransAll.T10 *TransAll.T21 ;
      TransAll.T30 = TransAll.T20 *TransAll.T32 ;
      TransAll.T40 = TransAll.T30 *TransAll.T43 ;
      TransAll.T50 = TransAll.T40 *TransAll.T54 ;
      TransAll.T60 = TransAll.T50 *TransAll.T65 ;
    end
    %{
***********************************************************************
* ROBCal.Tall_To_Cell ::                                              *
* This function is to convert the All transmatrix into cell format    *
with the given joint configuretion                                    *
*                                                                     *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'JointConfig':   a 1 x 6 column vector about the                   *
      current robot joint configuration                               *
* OUTPUT:                                                             *
*   'TCell' ---  a size of (6,2) cell contains the homogeneous        *
*  transformation matrix from link 1 to link 6                        *
*  The detailed contained matrix is self explained                    *
* WARNING:                                                            *
*   none                                                              *
* History: 07/26/2024, edited                                         *
***********************************************************************
    %}
    function TCell = Tall_To_Cell(JointConfig)
      Tall = ROBCal.TransMatrixCalculation(JointConfig) ;
      TCell=cell(6,2) ;
      TCell{1,1} = Tall.T10 ;  TCell{1,2} = Tall.T0b ;
      TCell{2,1} = Tall.T20 ;  TCell{2,2} = Tall.T21 ;
      TCell{3,1} = Tall.T30 ;  TCell{3,2} = Tall.T32 ;
      TCell{4,1} = Tall.T40 ;  TCell{4,2} = Tall.T43 ;
      TCell{5,1} = Tall.T50 ;  TCell{5,2} = Tall.T54 ;
      TCell{6,1} = Tall.T60 ;  TCell{6,2} = Tall.T65 ;
    end
    %{
***********************************************************************
* ROBCal.TraceCalculation ::                                          *
* This function is to convert the All transmatrix into cell format    *
with the given joint configuretion                                    *
*                                                                     *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'JointConfig':   a (n, 6) column vector about the                  *
*   current robot joint configuration                                 *
* OUTPUT:                                                             *
*   'Points_all' --- the trace according to the JointConfig           *
* WARNING:                                                            *
*   Input JointConfig is (n, 6) column vector, Pay attention          *
* History: 07/26/2024, edited                                         *
***********************************************************************
    %}
    function Points_all = TraceCalculation(JointP)
      % function citation
      Tcal =  @  ROBCal.TransMatrixCalculation ;
      Num_points = size(JointP,1) ;
      Points_all = zeros(Num_points,3) ;
      for i = 1: Num_points
        Points_all(i, : ) = Tcal(JointP(i,:)).T60(1:3,4)' ;
      end
    end
    %{
***********************************************************************
* ROBCal.InnerInverseKinematics ::                                    *
* This function is calculate joint angle in axes 1,2,3 for inverse    *
* kinematics                                                          *
*                                                                     *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'P_target':   [Px, Py, Pz]                                         *
* OUTPUT:                                                             *
*   'JointCfg' --- JontCfg(1), JontCfg(2), JontCfg(3) will            *
*              be calculated  of a robot arm                          *
*                                                                     *
* WARNING:                                                            *
*   none                                                              *
* History: 07/26/2024, edited                                         *
***********************************************************************
    %}
    %%  Inverse Kinematics
    function [JointCfg] = InnerInverseKinematics(P_target)
      %% Inverse Kinematics Function
      % to calculate joint angle from 1 to 3
      JointCfg = zeros(1,6) ;
      DH = ROBCal.GetDH ;
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
      JointCfg(1) = atan2d(P_target(2), P_target(1) )  ;
      JointCfg(2) =  phi - alpha  ;
      JointCfg(3) = ( 90-beta  + the_2_off )  ;
    end
    %{
***********************************************************************
* ROBCal.OuterInverseKinematics ::                                    *
* This function is calculate joint angle in axes 4£¬5 for inverse     *
* kinematics                                                          *
*                                                                     *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'Point_Vector':   [Px, Py, Pz£¬ AVx, AVy, AVz]                     *
*  AV is the Approaching Vector                                       *
* OUTPUT:                                                             *
*   'JointCfg' --- JontCfg(4), JontCfg(5),  will                      *
*              be calculated  of a robot arm                          *
*                                                                     *
* WARNING:                                                            *
*  JontCfg(6) is still equal to 0 !!!!!!                              *
* History: 07/26/2024, edited                                         *
***********************************************************************
    %}
    function [JointConfig] = OuterInverseKinematics(Point_Vector)
      %% Inverse Kinematics Function
      % to calculate joint angle from 1 to 5
      DH = ROBCal.GetDH ;
      RS = @ROBCal.TransMatrixCalculation ;
      P_target2 = Point_Vector(1:3) ;
      Approach_Vector = Point_Vector(4:6) ;
      Approach_Vector = Approach_Vector / norm(Approach_Vector) ;
      IKI = @ ROBCal.InnerInverseKinematics ;
      l56 = DH( 6, 4) ;
      P_target = P_target2 -  (l56 * Approach_Vector) ;
      JointConfig = IKI(P_target) ;
      
      Fun_temp = RS(JointConfig) ;
      Dot_Matrix = Fun_temp.T40(1:3,1:3) ;
      AV_T40 = Dot_Matrix' * Approach_Vector(:) ;
      AV_T40 = AV_T40 / norm(AV_T40) ;
      Pv1   = [norm(AV_T40([1 2])) AV_T40(3) ] ;  Pv1= Pv1 /norm(Pv1) ;
      Pv2   =  AV_T40([1 2]) / (norm(AV_T40([1 2])) + 1e-5) ;
      if AV_T40(3) <0
        JointConfig(5) = asind(Pv1(1))- 180 ;
      else
        JointConfig(5) = -asind(Pv1(1)) ;
      end
      JointConfig(4) =  atan2d(Pv2(2), Pv2(1)) ;
      
      % Q 6 is still be zero ;
    end
    %{
***********************************************************************
* ROBCal.InverseKinematics ::                                         *
* This function is calculate joint angle in axes 4£¬5 for inverse     *
* kinematics                                                          *
*                                                                     *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'Ptar':   [Px, Py, Pz]                                             *
*  'Rori':   Orientation Homogeneous Matrix size (4,4)                *
*  AV is the Approaching Vector                                       *
* OUTPUT:                                                             *
*   'JointCfg' --- JontCfg(1) ~ JontCfg(2),  will                     *
*              be calculated  of a robot arm                          *
*                                                                     *
* WARNING:                                                            *
*  Rori  is the pose matris can not be transl(0, 0, 0)                *
*   the initial position is Rotd(2, 90) !!!!!!!!!!!                   *
* History: 07/26/2024, edited                                         *
***********************************************************************
    %}
    function JointConfig = InverseKinematics(Ptar, Rori)
      OIK = @ ROBCal.OuterInverseKinematics ;
      Tcal = @ ROBCal.TransMatrixCalculation ;
      Av = Rori(1:3,3)' ;
      P_target = [Ptar, Av] ;
      JointConfig = OIK(P_target) ;
      R60 = Tcal(JointConfig).T60(1:3,1:3) ;
      Roff =  R60 \ Rori(1:3,1:3 ) ;
      JointConfig(6) = asind(Roff(2,1)) ;
    end
    %{
***********************************************************************
* ROBCal.ImportJointData ::                                           *
* This function is import  data from a text file                      *
*                                                                     *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'filename':   the path to be file                                  *
*  'dataLines':   the line range of imported data                     *
* OUTPUT:                                                             *
*   'Dataoutput' --- The imported data with size of (n,6)             *
*              be calculated  of a robot arm                          *
*  EXAMPLE£º                                                          *
*     [Joint] = ImportJointData("path.txt", [2, Inf]);                *
*                                                                     *
* WARNING:                                                            *
*  The file should use   '£»'  to seperate the data                   *
* History: Auto-generated by MATLAB on 2024-05-28 18:48:39            *
***********************************************************************
    %}
    
    function [Dataoutput] = ImportJointData(filename, dataLines)
      %% Input handling
      % If dataLines is not specified, define defaults
      if nargin < 2
        dataLines = [2, Inf];
      end
      
      %% Setup the Import Options and import the data
      opts = delimitedTextImportOptions("NumVariables", 6);
      % Specify range and delimiter
      opts.DataLines = dataLines;
      opts.Delimiter = ";";
      % Specify column names and types
      opts.VariableNames = ["JointT1", "JointT2", "JointT3", "JointT4", "JointT5", "JointT6"];
      opts.VariableTypes = ["double", "double", "double", "double", "double", "double"];
      % Specify file level properties
      opts.ExtraColumnsRule = "ignore";
      opts.EmptyLineRule = "read";
      % Import the data
      tbl = readtable(filename, opts);
      %% Convert to output type
      JointT1 = tbl.JointT1 ;
      JointT2 = tbl.JointT2 ;
      JointT3 = tbl.JointT3 ;
      JointT4 = tbl.JointT4 ;
      JointT5 = tbl.JointT5 ;
      JointT6 = tbl.JointT6 ;
      Dataoutput = [JointT1,JointT2,JointT3,JointT4,JointT5,JointT6] ;
    end
    %{
***********************************************************************
* ROBCal.ExportJointData ::                                           *
* This function is export  data about joint configuration.txt         *
*                                                                     *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'filename':   the path to be file                                  *
*  'JointP':   the joint P matrix for size :: N x 6                   *
*  EXAMPLE£º                                                          *
*     [Joint] = ExportJointData("JointP.txt", JointP);                *
*                                                                     *
* WARNING:                                                            *
*                                                                     *
* History: 08/01/2024, edited                                         *
***********************************************************************
    %}

    function ExportJointData( Filename, JointP)
        FilePointer = fopen( Filename, 'w') ;
        Content = sprintf('%4.5f ; %4.5f ;%4.5f; %4.5f ; %4.5f ;%4.5f; \n ', JointP') ;
        fprintf(FilePointer, Content) ;
        fclose(FilePointer);
    end


    %{
***********************************************************************
* ROBCal.CDF_Int ::                                                   *
* This function is to generate the differential for integration       *
*                                                                     *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'X':  vector                                                       *
* OUTPUT:                                                             *
*   'D' --- differential matrix sparse                                *
*                                                                     *
* WARNING:                                                            *
*  none                                                               *
* History: 07/27/2024, edited                                         *
***********************************************************************
    %}
    function [D ] = CDF_Int(X)
      len = length(X) ;
      Di = [ 1,        1:len-1,           2:len, len] ;
      Dj = [ 1,          2:len,         1:len-1, len] ;
      Dv = [-0.5, ones(1, len-1)- 0.5, -ones(1, len-1) + 0.5,   0.5] ;
      Dv(1,[1 2]) = [-1 1]; Dv(end,[end-1 end]) = [-1 1] ;
      D = sparse( Di, Dj, Dv, len, len) ;
    end
    
    %{
***********************************************************************
* ROBCal.UpdatePoint ::                                               *
* This function is to update the  joints according to the             *
*   transformation matrix                                             *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'Points':  size (n x 3)  matrix  [px(:), py(:), pz(:)]             *
*  'TransMatrix':  size (4 x 4)  homogenesous matrix                  *
* OUTPUT:                                                             *
*   'PointsNew' --- updated points                                    *
*                                                                     *
* WARNING:                                                            *
*  Points is a struct                                                 *
* History: 08/05/2024, edited                                         *
***********************************************************************
    %}
        % function to update the Points
        function PointsNew = UpdatePoint(Points, TransMatrix)
%           PointsNew = 0 
        [m, n] = size(Points.X) ;
        POld =[ Points.X(:), Points.Y(:), Points.Z(:), ones(m*n,1) ] ;
        PNew = POld * TransMatrix' ;
        PointsNew.X = reshape(PNew(:,1), [m, n] ) ;
        PointsNew.Y = reshape(PNew(:,2), [m, n] ) ;
        PointsNew.Z = reshape(PNew(:,3), [m, n] ) ;
        end
    
  end
  
  
  
  
end

