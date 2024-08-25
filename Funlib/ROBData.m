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
****                    Class for Data Analysis                    ****
***********************************************************************
*---------------------------------------------------------------------*
*                   Start date:    July 27 2024                       *
*                   Last update:   July 27 2024                       *
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
classdef ROBData < ROBGUI
    %{
***********************************************************************
  Summary of the Class:
   This Class is created for analysising experimental data

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

        function [JointP, JointT,Result] = CompareData(pFile, pJointP, pJointT,Xval)
            ImJ = @ ROBData.ImportJointData ;

            path_jp = [pFile, '/',pJointP] ;
            path_jt = [pFile, '/',pJointT] ;

            JointP = ImJ(path_jp,[2 inf]) ;
            JointT = ImJ(path_jt,[2 inf]) ;
            Difff = ROBData.CDF_Int(JointP(:,1)) ;
            dt = 0.1 ;
            Nump= size(JointP, 1) ;
            Time= linspace(1, Nump, Nump)' * dt ;
            rad = pi / 180 ;
            Q = JointP * rad ;
            Qd = Difff * Q /dt ;
            Qdd = Difff * Qd /dt ;
            Rtb = ROBRTB.BuildRobotII(Xval) ;
            Tau = Rtb.rne(Q,Qd, Qdd) ;

            figure(4) ; clf
            for i = 1:6
                subplot(2,3,i)
                plot(Time,Tau(:,i),'-or','linewidth' , 1.5 ) ; hold on
                plot(Time,JointT(:,i), 'k', 'linewidth', 1.5)
                legend(['Modeling T', num2str(i)], ['Measure T', num2str(i)])
                title(['Torque Modelling for Joint ', num2str(i)],'fontsize',12) ;
            end
            Label_Position = [
                0.267    0.504   0.07    0.05;
                0.545    0.504   0.07    0.05;
                0.83     0.504   0.07    0.05;
                0.267    0.028   0.07    0.05;
                0.545    0.028   0.07    0.05;
                0.83     0.028   0.07    0.05;
                ];
            Power_ext = abs(JointT.*Qd) ;
            Power_mod = abs(Tau.*Qd) ;
            figure(5) ; clf
            for i = 1:6
                subplot(2,3,i)
                plot(Time,Power_mod(:,i),'-or','linewidth' , 1.5 ) ; hold on
                plot(Time,Power_ext(:,i), 'k', 'linewidth', 1.5)
                L= legend(['Modeling T', num2str(i)], ['Measure T', num2str(i)],...
                    'FontSize',14);
                title(['Power Modelling for Joint ', num2str(i)],'fontsize',22) ;
                L.Position = Label_Position(i,:);

                xlabel('Time (s)','FontSize',14)
                ylabel('Power (w）','FontSize',14)
                grid on; grid minor ;
            end

            Eexp = sum(sum(abs(JointT.*Qd)*dt)) ;

            Emod = sum(sum(abs(Tau.*Qd)*dt));

            Error = (Eexp-Emod)/Emod * 100;

            Result = [Eexp,Emod,Error] ;

        end






    end
end

