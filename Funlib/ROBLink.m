%{
***********************************************************************
**************         3D Printing Research Group        **************
***********************************************************************
************            Principal Invetigator (PI):        ************
*********               >>>   Dr. Yiwei Weng   <<<            *********
***********************************************************************
***               The Hong Kong Polytechnic University             ****
***              Department of Building and Real Estate            ****
***                       Hong Kong (P.R.C.)                       ****
***********************************************************************
*---------------------------------------------------------------------*
*                 Lab Website: wengyiwei.github.io                    *
*---------------------------------------------------------------------*
*                                                                     *
***********************************************************************
****           _______________________________________             ****
****           Class for Robot Link Dynamic Parameters             ****
****           ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~             ****
***********************************************************************
*---------------------------------------------------------------------*
*                   Start date:    July 28 2024                       *
*                   Last update:   July 28 2024                       *
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

classdef ROBLink
  %{
***********************************************************************
  Summary of the Class:
***********************************************************************
* Properties:                                                         *
*    dyn  display link dynamic parameters                             *
*    friction      friction force                                     *
*                                                                     *
*    overload operators ::                                            *
*    operator1 :  >  Forward Recursive                                *
*    operator2 :  <  Backward Recursive                               *
*---------------------------------------------------------------------*
*    Dynamic parameters ::                                            *
*    M       |  link mass                                             *
*    MC      |  Center of Mass  ( size : 1 x 3 )                      *
*    IMc     |  Inertia Matrix, ( size : 1 x 6 ) and convert to       *
*            |  symmetric matrix (3 x 3) for further calculation      *
*            |  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ *
*    Fv      |  Viscious Friction (Motor referred)                    *
*    Fc      |  Cuolomb Friction                                      *
*    GR      |  Gear ratio                                            *
*    Jm      |  Motor Inertia (Motor referred)                        *
*                                                                     *
* Functions:                                                          *
*    ROBLink.ConvertToSymmetric                                       *
*    ROBLink.InertiaMatOrigin                                         *
*---------------------------------------------------------------------*
  %}
  properties
    M        %  link mass
    Mc       %  Center of Mass  ( size : 1 x 3 )
    IMg      %  Inertia Matrix, [Ixx Ixy Ixz Iyy Iyz Izz] (1x6)
    IMO      %  Inertia Matrix according to O point
    Fv       %  Viscious Friction (Motor referred)
    Fc       %  Cuolomb Friction
    GR       %  Gear ratio
    Jm       %  Motor Inertia (Motor referred)
  end
  methods
    % method to create the class
    function obj = ROBLink(M, Mc, IMg, Fv, Fc, GR, Jm)
      %ROBLINK Construct an instance of this class
      %   Detailed explanation goes here
      obj.M = M ;
      obj.Mc = Mc ;
      obj.IMg = IMg ;
      obj.IMO = ROBLink.InertiaMatOrigin(IMg, M, Mc) ;
      obj.Fv = Fv ;
      obj.Fc = Fc ;
      obj.GR = GR ;
      obj.Jm = Jm ;
    end
    % Operator overload
    function result = plus(obj1, obj2)
      disp('hello')
      result = obj1.M + obj2.M  ;
    end
  end
  methods(Static)
    %{
***********************************************************************
* ROBLink.DemoVelocity ::                                             *
* This function is to convert the Inertia Vector to Inertia Matrix    *
*                                                                     *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'InertiaVector' : [Ixx Ixy Ixz Iyy Iyz Izz]                        *
*---------------------------------------------------------------------*
* OUTPUT:                                                             *
*  'InertiaMatrix' : 6 x 6 symmetric matrix                           *
* WARNING: none                                                       *
* History: 07/28/2024, edited                                         *
***********************************************************************
    %}
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
    %{
***********************************************************************
* ROBLink.InertiaMatOrigin ::                                         *
* This function is to combine the Inertia Matrix and center of mass   *
* into a Inertia Matrix reference the origin point                    *
*                                                                     *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'IMg' : The Inertia vector rererence to center of mass point       *
*           1 x 6 vector [Ixx Ixy Ixz Iyy Iyz Izz]                    *
*---------------------------------------------------------------------*
* OUTPUT:                                                             *
*  'InertiaMatrix' : 6 x 6 symmetric matrix reference                 *
*                     to origin point in Frame i                      *
* WARNING: none                                                       *
* History: 07/29/2024, edited                                         *
***********************************************************************
    %}
    
    % InertiaMatrix reference to original point
    % warning
    function InertiaMatO = InertiaMatOrigin(IMg, M, Mc)
      
      SYM = @ ROBLink.ConvertToSymmetric ;
      
      px = Mc(1) ;
      py = Mc(2) ;
      pz = Mc(3) ;
      parV = M * [norm([py pz]) , -px*py, -px*pz, norm([px pz]), -py*pz, norm([px py]) ];
      InertiaMatO = SYM(IMg)  +   SYM(parV) ;
      
      
    end
    
    
  end
  
  
  
end

