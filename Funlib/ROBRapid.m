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
*                   Start date:    July 30 2024                       *
*                   Last update:   July 30 2024                       *
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
classdef ROBRapid < ROBCal 
    %{
***********************************************************************
  Summary of the Class:
   This Class is created generate the ABB Rapid Scripts

***********************************************************************
* Properties:                                                         *
*                                                                     *
*  none                                                               *
*                                                                     *
* Methods(Static)                                                     *
*                                                                     *
*  ROBRapid.OutputRapid        output MovL Rapid according to Points  *
*  ROBRapid.OutputMOveABJ  ---  output abs joint configuration        *
*                                                                     *
*                                                                     *
*                                                                     *
*_____________________________________________________________________*
*                                                                     *
*                                                                     *
*---------------------------------------------------------------------*
    %}


    methods(Static)      
        %{
***********************************************************************
* ROBRapid.OutputRapid ::                                             *
* function OutputRapid is to output the Rapid Script for MoveL        *
*---------------------------------------------------------------------*
* INPUT:                                                              *

* OUTPUT:                                                             *
*   No output                                                         *
* WARNING: orientation is not complete                                *
* History: 07/25/2024, edited                                         *
***********************************************************************
        %}



        %{
***********************************************************************
* ROBRapid.OutputRapid ::                                             *
* function OutputRapid is to output the Rapid Script for MoveL        *
*---------------------------------------------------------------------*
* INPUT:                                                              *

* OUTPUT:                                                             *
*   No output                                                         *
* WARNING: orientation is not complete                                *
* History: 08/04/2024, edited                                         *
***********************************************************************
        %}

        function  OutputRapid(Filename, Targetpoints)
            Rotd = @ ROBRapid.RotationMatrix_Degree;
            IK = @ ROBRapid.InverseKinematics ;
            Ttool60 = transl( 0,  0, 0) ;
            Rori = Rotd( 2, 90)  ;
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
     
            FrontStatement{1} = sprintf('VAR robtarget Path{%3d} ;', Nump ) ;
            FrontStatement{2} = sprintf('VAR num PAll:= %d ;', Nump ) ;
            FrontStatement{3} = sprintf('VAR num Pind ;') ;

            EndStatement{1} = 'FOR Pind FROM 1 TO PAll DO';
            EndStatement{2} = 'MoveL Path{Pind},v100,z5,tool0;' ;
            EndStatement{3} = 'ENDFOR';
            Mainbody = cell(Nump,1) ;
           Quaternion = ROBRapid.QuaternionCal(Rori) ;
            for iline = 1 : Nump
                Parm1 = sprintf('[[%4.5f, %4.5f, %4.5f],', Targetpoints(iline,:)) ;
                Parm2 = sprintf('[%4.5f, %4.5f, %4.5f, %4.5f],', Quaternion) ;
                Parm3 = sprintf('[%4.5f, %4.5f, %4.5f, %4.5f],', Cfg(iline,:)) ;
                Parm4 = '[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]] ;';
                P_all = ['Path{%3d}:=',Parm1, Parm2, Parm3, Parm4] ;
                Mainbody{iline} = sprintf(P_all, iline  ) ;
            end
            % Filename = 'RapidScript_Movel.txt';

            FilePointer = fopen(Filename,'w') ;
            % edit the script
            for ind = 1:3
                fprintf(FilePointer, [FrontStatement{ind},'\n']) ;
            end

            for iline = 1: Nump
                fprintf(FilePointer, [Mainbody{iline},'\n']) ;
            end
            for ind = 1:3
                fprintf(FilePointer, [EndStatement{ind},'\n']) ;
            end

            fclose(FilePointer);

        end

        %{
***********************************************************************
* ROBRapid.OutputMOveABJ ::                                           *
* function OutputRapid is to output the Rapid Script for MoveJabs     *
*---------------------------------------------------------------------*
* INPUT:                                                              *
   'Filename'  :  Example : 'JointP.txt'
   'JointP'    :  Joint Position 
* OUTPUT:                                                             *
*   No output                                                         *
* WARNING: orientation is not complete                                *
* History: 07/25/2024, edited                                         *
***********************************************************************
        %}

        function OutputMOveABJ(Filename, JointP)
            Nump = size( JointP, 1)  ;
            FrontStatement{1} = sprintf('VAR jointtarget JointPabs{%3d} ;\n', Nump ) ;
            FrontStatement{2} = sprintf('VAR num PAllabs:= %d ;\n', Nump ) ;
            FrontStatement{3} = 'VAR num Pindabs ;\n';
            EndStatement{1} = 'FOR Pindabs FROM 1 TO PAllabs DO';
            EndStatement{2} = 'MoveAbsJ JointPabs{Pindabs},v100\\T:=0.3,z5,tool0;';
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

