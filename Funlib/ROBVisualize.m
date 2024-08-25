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
classdef ROBVisualize < ROBCal 
  %{
***********************************************************************
  Summary of the Class:
***********************************************************************
  
  This Class is created for visulaize the robot motion status.
***********************************************************************
  
***********************************************************************
* Properties:                                                         *
*                                                                     *
*  none                                                               *
*                                                                     *
* Methods(Static)                                                     *
*                                                                     *
*  ROBVisualize.PlotCord   ---   Draw the Coordinate System           *
*_____________________________________________________________________*
*

*---------------------------------------------------------------------*
  %}
  methods(Static)
    %{
***********************************************************************
* ROBVisualize.PlotCord ::                                            *
* function PlotCord is to draw the homogeneous matrix in 3-D space    *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'TransMatrix' : The homogeneous (4x4) trans matrix                 *
*  'LineLength' :  The length of the line in x, y, z axis             *
*  'LineStyle' : '-' for solid line and '--' for dash line            *
*  'Destination': There to draw the coordinate system                 *
* OUTPUT:                                                             *
*   No output                                                         *
* WARNING: none                                                       *
* History: 07/25/2024, edited                                         *
***********************************************************************
    %}
    % Plot the Coordinate System in 3 Dimensional
    function PlotCord(Destination, TransMatrix, LineLength, LineStyle )
      width = 1.5 ;
      %   Draw x-axis
      l1 = plot3(Destination, TransMatrix(1,4) + ...
        [0  TransMatrix(1,1)]*LineLength, TransMatrix(2,4) + ...
        [0  TransMatrix(2,1)] * LineLength, TransMatrix(3,4)+ ...
        [0  TransMatrix(3,1)] * LineLength) ;
      %   Set the style of x-axis
      l1.LineStyle = LineStyle; l1.Color = 'r';
      l1.LineWidth = width; hold(Destination, 'on')
      %   Draw y-axis
      l2 = plot3(Destination, TransMatrix(1,4) + ...
        [0  TransMatrix(1,2)]* LineLength, TransMatrix(2,4) + ...
        [0  TransMatrix(2,2)] * LineLength, TransMatrix(3,4)+ ...
        [0  TransMatrix(3,2)] * LineLength);
      %   Set the style of y-axis
      l2.LineStyle = LineStyle; l2.Color = 'g'; l2.LineWidth = width;
      %   Draw the z-axis
      l3 = plot3(Destination, TransMatrix(1,4) + ...
        [0  TransMatrix(1,3)] * LineLength, TransMatrix(2,4) + ...
        [0  TransMatrix(2,3)] * LineLength, TransMatrix(3,4)+ ...
        [0  TransMatrix(3,3)] * LineLength);
      %   Set the style of z-axis
      l3.LineStyle = LineStyle; l3.Color = 'b'; l3.LineWidth = width;
    end
    %% need to be refine
        %{
***********************************************************************
* ROBVisualize.DrawRob_now ::                                         *
* function DrawRob_now is to demo the current Situation of a robot    *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'Destination': There to draw the coordinate system                 *
*  'JointConfig': Joint configuration of a robot arm                  *  
* OUTPUT:                                                             *
*   No output                                                         *
* WARNING: The parameter set is not from DH parameter                 *
* History: 07/27/2024, edited                                         *
***********************************************************************
    %}
    function DrawRob_now(Destination, JointConfig )
       TCellCal =  @  ROBVisualize.Tall_To_Cell ;
       Rotd = @ ROBVisualize.RotationMatrix_Degree ;
       PC = @  ROBVisualize.PlotCord ;
       TCell= TCellCal(JointConfig) ;
       hold(Destination, 'off') % clear the 
       TL = 150 ;
       PC(Destination,TCell{1,2},TL ,'-' ); % base
       for i = 1: 6
       PC(Destination,TCell{i,1},TL ,'--' );% link
       end
       LinkLine = zeros(7,3) ;
       LinkLine(1,:) = TCell{1,2}(1:3,4)'  ;
       
        for ind = 1:6
          LinkLine(ind+1,:) = TCell{ind,1}(1:3,4)' ;
        end
%         plot3(Destination, LinkLine(:,1), LinkLine(:,2), LinkLine(:,3),...
%           '--k','Linewidth', 1.5)
        %%% Draw Link1 
        RVC = @ ROBVisualize.Cylinder ;
        RVUP = @ ROBVisualize.UpdatePoint ;
        RVD = @ ROBVisualize.DrawCylindar ;
        LinkPoints = cell(6, 3) ;
        % Part :: Generate the Points
        LinkPoints{1,1} = RVC(100, 20, 25, 10) ;
        LinkPoints{1,2} = RVUP( RVC(20, 445, 5, 10) , TCell{1,1}*transl(0,0,-445)) ;  
        LinkPoints{1,3} = RVUP( RVC(20, 150, 5, 10) , TCell{1,1}*Rotd(2,90)  ) ;
        LinkPoints{2,1} = RVUP( RVC(40, 40, 25, 10) , TCell{2,1}*transl(0,0,-20)) ; 
        LinkPoints{2,2} = RVUP( RVC(20, 700, 5, 10) , TCell{2,1}*Rotd(2,90)  ) ; 
        LinkPoints{3,1} = RVUP( RVC(40, 40, 25, 10) , TCell{3,1}*transl(0,0,-20)) ; 
        LinkPoints{3,2} = RVUP( RVC(20, 115, 5, 10) , TCell{3,1}*Rotd(2,90)   ) ;
        LinkPoints{3,3} = RVUP( RVC(20, 200, 5, 10) , TCell{3,1}*Rotd(1,-90)*transl(115,0,0)   ) ;
        LinkPoints{4,1} = RVUP( RVC(20, 595, 5, 10) , TCell{4,1}*transl(0,0, -595)) ;
        LinkPoints{4,2} = RVUP( RVC(40, 40, 25, 10) , TCell{4,1}*transl(0,0, -595)) ;
        LinkPoints{5,1} = RVUP( RVC(40, 40, 25, 10) , TCell{5,1} * transl( 0, 0,-20)) ;
        LinkPoints{5,2} = RVUP( RVC(20, 85,  5, 10) , TCell{5,1} * Rotd(1, 90)) ;
        LinkPoints{6,1} = RVUP( RVC(40, 40, 25, 10) , TCell{6,1}  ) ;
        % Generate the Points
        ColR =[255 25 8]/255  ; ColS = [15 10 255]/255  ;        
        %---------Draw Function Part -----------------------
        RVD(Destination, LinkPoints{1,1},ColR ) % Draw base
        %---------------------------------------------------
        RVD(Destination, LinkPoints{1,2},ColS ) % Draw Link1 
        RVD(Destination, LinkPoints{1,3},ColS ) % Draw Link1
        %---------------------------------------------------
        RVD(Destination, LinkPoints{2,1},ColR ) % Draw Link2 
        RVD(Destination, LinkPoints{2,2},ColS ) % Draw Link2 
        %---------------------------------------------------
        RVD(Destination, LinkPoints{3,1}, ColR) % Draw Link3
        RVD(Destination, LinkPoints{3,2}, ColS) % Draw Link3
        RVD(Destination, LinkPoints{3,3}, ColS) % Draw Link3
        %---------------------------------------------------
        RVD(Destination, LinkPoints{4,1}, ColS) % Draw Link4
        RVD(Destination, LinkPoints{4,2}, ColR) % Draw Link4
        %---------------------------------------------------
        RVD(Destination, LinkPoints{5,1}, ColR) % Draw Link5
        RVD(Destination, LinkPoints{5,2}, ColS) % Draw Link5
        %---------------------------------------------------
        RVD(Destination, LinkPoints{6,1}, ColR) % Draw Link5
        shading(Destination, 'interp') ;
         axis(Destination, 'equal')  ;
      axis(Destination, [-400 1800 -1000 1000 0 2000] )
      axis(Destination,'off')
      view(Destination, 115,25)
    end
    %{
***********************************************************************
* ROBVisualize.Cylinder ::                                            *
* function Cylinder is to generate the Points of a Cylindar           *
*---------------------------------------------------------------------*
* INPUT:                                                              *
*  'Radius' :   Radius of the Cylindar                                *
*  'Height' :  The length of the line in x, y, z axis                 *
*  'nNodes' :  the nodes around thr radius  5 for the square shape    *
*  'Destination': There to draw the coordinate system                 *
* OUTPUT:                                                             *
*   No output                                                         *
* WARNING: none                                                       *
* History: 07/25/2024, edited                                         *
***********************************************************************
    %}
           % Cylindar 
        function Points = Cylinder(Radius,Height,nNodes,nCS)
            %To create a Cylinder
            r = Radius * ones(1, nNodes) ;
            th = linspace(0, 2 * pi, nNodes) ;
            z = linspace( 0, Height, nCS)'  ;
            [   x,y] = pol2cart(th+pi/4, r) ;
            X = repmat(x, nCS,1) ;
            Y = repmat(y, nCS,1) ;
            Z = repmat(z, 1,nNodes) ;           
            x_lid = zeros(2,nNodes) ;
            y_lid = zeros(2,nNodes) ;
            z_lid = repmat([0;Height], 1,nNodes)  ;            
            X = [x_lid(1,:) ; X ; x_lid(2,:) ] ;
            Y = [y_lid(1,:) ; Y ; y_lid(2,:) ] ; 
            Z = [z_lid(1,:) ; Z ; z_lid(2,:) ] ;
            Points.X = X ; 
            Points.Y = Y ;
            Points.Z = Z ;
        end

        % function to draw the cylindar
        function DrawCylindar(Destinsation, Points, Color)
          C0 = Points.Z* 0 ;
          for i = 1: 3 ;  CO(:, :, i) = C0 + Color(i) ; end    
           surf(Destinsation, Points.X,Points.Y,Points.Z, CO)
%           light
%            shading(Destinsation,'interp')
        end
    
  end
end

