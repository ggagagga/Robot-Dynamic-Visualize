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
classdef ROBGUI < ROBVisualize
    %{
***********************************************************************
  Summary of the Class:
   This Class is created to generate the Graphical User Interface
   to Manage the experimental data.
***********************************************************************
* Properties:                                                         *
*                                                                     *
*  none                                                               *
*                                                                     *
* Methods(Static)                                                     *
*                                                                     *
*  ROBGUI.CreateHandle   ---  Create the GUI window                   *
*  ROBGUI.RotationMatrix_Degree  ---   Rotation Matrix                *
*                                                                     *
*                                                                     *
*                                                                     *
*_____________________________________________________________________*
*                                                                     *
*                                                                     *
*---------------------------------------------------------------------*
    %}


    methods(Static)
        function Handle = CreateHandle
            %% Create the window
            Handle.Window = figure(1) ;
            Handle.Window.Color = [1 1 1]*0.96 ; Handle.Window.MenuBar = 'none' ;
            Handle.Window.Name = 'DATAWATCHER' ;  Handle.Window.NumberTitle = 'off' ;
            % Handle.Window.Position = FigurePositon(1,:) ;
            Handle.Window.Units = 'normalized';
            Handle.Window.Position =[0.02  0.05    0.85    0.85] ;
            %% create close button
            Handle.Button.CloseButton = uicontrol(Handle.Window,'Callback','close all','fontsize',18) ;
            Handle.Button.CloseButton.String = 'Exit';
            Handle.Button.CloseButton.Units = 'normalized' ;
            Handle.Button.CloseButton.Position = [0.85 0.01 0.15 0.05] ;
            %%% refresh button
            Handle.Button.Refresh = uicontrol(Handle.Window,'string','Refresh','Callback','clc','fontsize',18) ;
            Handle.Button.Refresh.Units = 'normalized' ;
            Handle.Button.Refresh.Position = [0.85 0.06 0.15 0.05] ;

            %%% Static Text Select
            Select = uicontrol(Handle.Window,'style','text','String','Select Data:','units','normalized','fontsize',12) ;
            Select.Position = [0.85 0.92 0.1 0.03] ; Select.BackgroundColor = [1 1 1]*0.94 ;
            Select.HorizontalAlignment = 'left' ;
            %%% Static Text Select Joint Position
            JointPosition = uicontrol(Handle.Window,'style','text','String','Select Joint Postion:','units','normalized','fontsize',12) ;
            JointPosition.Position = [0.85 0.85 0.12 0.03] ; JointPosition.BackgroundColor = [1 1 1]*0.94 ;
            JointPosition.HorizontalAlignment = 'left' ;
            %%% Static Text Select Joint Torque
            JointTorque = uicontrol(Handle.Window,'style','text','String','Select Joint Torque:','units','normalized','fontsize',12) ;
            JointTorque.Position = [0.85 0.78 0.12 0.03] ; JointTorque.BackgroundColor = [1 1 1]*0.94 ;
            JointTorque.HorizontalAlignment = 'left' ;


            % payattention to the create sequence ;
            Handle = ROBGUI.CreateFileListBox(Handle) ;
            Handle = ROBGUI.PopMenu(Handle) ;
            Handle = ROBGUI.IMportButton(Handle) ; % Create Getdata
            % Handle = ROBGUI.CreateGetdata(Handle) ; % Create Getdata
            Handle = ROBGUI.CreateScreen(Handle) ;  % Create Screen

            % Button Group
            Handle = ROBGUI.CreateButtonGroup(Handle) ;

            % Viewbutton

            Handle = ROBGUI.CreateViewButton(Handle) ;


            Handle = ROBGUI.CreateSlider(Handle) ;%  Create Slider
            Handle = ROBGUI.CreatePlotButton(Handle) ;% create Plot button
            %       Handle = ROBGUI.CreateScreen(Handle) ;  % Create Screen



        end
        %% Import BUtton Part
        function Handle = IMportButton(Handle)
            Handle.PushButton.Import =  uicontrol(Handle.Window,...
                'String','IMPORT','units','normalized',...
                'position',[0.85 0.95 0.15 0.05],'fontsize',14) ;
            Handle.PushButton.Import.Callback = @(HObject, Eventdata)...
                ROBGUI.ImportMethod(HObject, Eventdata, Handle ) ;
        end

        function HObject = ImportMethod(HObject, Eventdata, Handle)
            %       HObject.UserData = uigetdir() ;
            path = strrep(uigetdir() ,'\','/') ;
            INFO = dir(path);
            Numfile = size(INFO,1) ;
            Handle.Popmenu.String = cell(Numfile,1) ;
            for i = 1: Numfile
                Handle.Popmenu.String{i} = [path,'/',INFO(i).name] ;
            end



        end
        %% Import BUtton Part
        % to display what is inside
        function Handle = PopMenu(Handle)
            Handle.Popmenu = uicontrol(Handle.Window,'style','popupmenu','fontsize',12) ;
            Handle.Popmenu.String = {'Wairing for file selection','Wairing for file selection', ...
                'Wairing for file selection'} ;
            Handle.Popmenu.Units = 'normalized' ;
            Handle.Popmenu.Position = [0.85 0.88 0.15 0.045] ;
            Handle.Popmenu.Callback = @(HObject, Eventdata)...
                ROBGUI.PopMenuMethod(HObject, Eventdata, Handle ) ;
        end

        function HObject = PopMenuMethod(HObject, Eventdata, Handle)
            disp('this is popmenu')
            Handle.FileListBox{1}.String = cell(10,1) ;
            path = HObject.String{HObject.Value} ;
            INFO = dir(path);
            Numfile = size(INFO,1) ;
            for j = 1:2
                Handle.FileListBox{j}.String = cell(Numfile,1) ;
                for i = 1: Numfile
                    Handle.FileListBox{j}.String{i} = INFO(i).name ;
                end
            end

        end
        %% Part FileList Box
        function Handle = CreateFileListBox(Handle)
            Handle.FileListBox{1} = uicontrol(Handle.Window, 'style','popupmenu', 'units','normalized',...
                'position',[0.85  0.75 0.15 0.1],...
                'fontsize',12) ;

            Handle.FileListBox{2} = uicontrol(Handle.Window, 'style','popupmenu', 'units','normalized',...
                'position',[0.85  0.68 0.15 0.1],...
                'fontsize',12) ;

            N = 10 ;
            for j = 1:2
                Handle.FileListBox{j}.String =cell(1,N) ;
                for i = 1: N
                    Handle.FileListBox{j}.String{i} = 'No file inside' ;
                end
            end
        end

        %% Create Getdata Button
        function Handle = CreateGetdata(Handle)
            Handle.PushButton.Getdata = uicontrol(Handle.Window, 'style',...
                'pushbutton', 'string','GetData','units','normalized',...
                'position',[0.85  0.55 0.15 0.05],...
                'fontsize',14) ;
            Handle.PushButton.Getdata.Callback = @(HObject, Eventdata)...
                ROBGUI.GetdataMethod(HObject, Eventdata, Handle);
        end
        % to get the file name
        function FileName = GETSaveFileName
            FileName = 'JointTPS_Experiment.mat' ;
        end


        % callback function to get data
        function HObject = GetdataMethod(HObject, Eventdata, Handle)
            ImJ = @ ROBGUI.ImportJointData ;
            Vjoint_position = Handle.Popmenu.Value ;
            Readfile = Handle.Popmenu.String{Vjoint_position} ;
            JointP = ImJ([Readfile,'/JointP_Record.txt'], [2 Inf]) ;

            Vjoint_torque = Handle.FileListBox.Value ; % related to filelist box
            Jointfile = Handle.FileListBox.String{Vjoint_torque} ;
            JointT = ImJ([Readfile,'/',Jointfile ], [2 inf]) ;
            savename = ROBGUI.GETSaveFileName ;

            % Delete some element
            num1 = size(JointP,1) ;num2 = size(JointT,1) ;

            minnum = min(num1, num2) ;
            JointP = JointP(1:minnum,:) ;
            JointT =  JointT(1:minnum,:) ;


            Difff = ROBGUI.CDF_Int(JointP(:,1)) ;
            NumP = size( JointT, 1) ;
            dt = 0.1 ;
            Time  = linspace(1,NumP,NumP) * dt ;
            JointS = Difff * JointP /dt ;
            JointA = Difff * JointS / dt ;
            save(savename, 'JointP','JointS','JointT','JointA','Time','dt') ;
        end
        %% Create PushButton Froup
        function Handle = CreateButtonGroup(Handle)
            Handle.ButtonGroup.Father = uibuttongroup(Handle.Window,...
                'position', [0.01 0.95 0.8 0.05] )  ;
            % Greate sub button 1
            Handle.ButtonGroup.Son{1} =uicontrol(Handle.ButtonGroup.Father,...
                'style', 'radiobutton', 'string','Joint Configuration','fontsize',14) ;
            Handle.ButtonGroup.Son{1}.Units = 'normalized' ;
            Handle.ButtonGroup.Son{1}.Position  = [ 0.0 0.0 0.15 1] ;

            Handle.ButtonGroup.Son{2} =uicontrol(Handle.ButtonGroup.Father,...
                'style', 'radiobutton', 'string','Joint Torque','fontsize',14) ;
            Handle.ButtonGroup.Son{2}.Units = 'normalized' ;
            Handle.ButtonGroup.Son{2}.Position = [ 0.15 0.0 0.15 1] ;

            Handle.ButtonGroup.Son{3} =uicontrol(Handle.ButtonGroup.Father,...
                'style', 'radiobutton', 'string','Velocity','fontsize',14) ;
            Handle.ButtonGroup.Son{3}.Units = 'normalized' ;
            Handle.ButtonGroup.Son{3}.Position = [ 0.25 0.0 0.15 1] ;
            for i = 1:3
                Handle.ButtonGroup.Son{i}.Callback = @(HObject, Eventdata) ...
                    ROBGUI.ButtonGroupMethod(HObject, Eventdata, Handle) ;

            end

        end
        %     %% call back function
        function HObject = ButtonGroupMethod(HObject, Eventdata, Handle)

            if Handle.ButtonGroup.Son{1}.Value
                Handle.Screen.Panel{1}.Visible = 'on' ;
                Handle.Screen.Panel{2}.Visible = 'off' ;
                Handle.Screen.Panel{3}.Visible = 'off' ;
                %         ROBGUI.PlotMethod(HObject, Eventdata, HObject) ;
            elseif Handle.ButtonGroup.Son{2}.Value
                Handle.Screen.Panel{1}.Visible = 'off' ;
                Handle.Screen.Panel{2}.Visible = 'on' ;
                Handle.Screen.Panel{3}.Visible = 'off' ;
            elseif Handle.ButtonGroup.Son{3}.Value
                Handle.Screen.Panel{1}.Visible = 'off' ;
                Handle.Screen.Panel{2}.Visible = 'off' ;
                Handle.Screen.Panel{3}.Visible = 'on' ;
            end
            %

        end


        %% Create Screen
        function Handle = CreateScreen(Handle)
            Handle.Screen.Panel{1} = uipanel(Handle.Window,'BackgroundColor',[1 1 1]* 0.96) ;
            Handle.Screen.Panel{1}.Position = [0 0 0.8 0.95] ;
            Handle.Screen.Panel{2} = uipanel(Handle.Window,'BackgroundColor',[1 1 1]* 0.96) ;
            Handle.Screen.Panel{2}.Position = [0 0 0.8 0.95] ;
            Handle.Screen.Panel{3} = uipanel(Handle.Window,'BackgroundColor',[1 1 1]* 0.96) ;
            Handle.Screen.Panel{3}.Position = [0 0 0.8 0.95] ;
            Subfig1Position = [
                0.03 0.68  0.45 0.26;
                0.03 0.35  0.45 0.26 ;
                0.03 0.03  0.45 0.26 ; 
                 0.5 0.03  0.55 0.85 ;
                ] ;
 Subfig2Position = [
                0.03 0.53 0.45 0.40;
                0.03 0.03  0.45 0.40 ;
                0.55 0.45  0.45 0.45 ; 
                 0.5 0.03  0.55 0.95 ;
                ] ;


               
            for i = 1: 4
                Handle.Screen.Subfig{1,i} =  axes(Handle.Screen.Panel{1},'units',...
                    'normalized','position',Subfig1Position(i,:)) ;
            end
            %       axis(Handle.Screen.Subfig{1,3}, 'off')
            for i = 1:3
                Handle.Screen.Subfig{2,i} = axes(Handle.Screen.Panel{2},'units',...
                    'normalized','position',Subfig2Position(i,:)) ;
            end
            %       Handle.Screen.Panel{1}.Visible = 'on ' ;
            Handle.Screen.EDIT = uicontrol(Handle.Screen.Panel{3},'style','text','units','normalized'...
                , 'position',Subfig1Position(1,:),'fontsize',16) ;
            Handle.Screen.EDIT.BackgroundColor = [1 1 1] ;
            Handle.Screen.EDIT.HorizontalAlignment = 'Left' ;
            Handle.Screen.EDIT.String = 'First line \n second line' ;

            Handle.Screen.Subfig{3,3} =  axes(Handle.Screen.Panel{3},'units',...
                'normalized','position',Subfig1Position(3,:)) ;
        end

        %% Create Slider and plot button all to bind the plot method
        function Handle = CreateSlider(Handle)
            Handle.Slider = uicontrol(Handle.Window,'style','slider','units','normalized',...
                'position',[0.81 0.01 0.03 0.98]) ;
            Handle.Slider.Callback =  @(HObject, Eventdata) ...
                ROBGUI.PlotMethod(HObject, Eventdata, Handle);
        end


        function Handle = CreateViewButton(Handle)
            Handle.ViewButton{1}= uicontrol(Handle.Window,'Style','pushbutton','String','ViewX-: 130','units','normalized',...
                'Position',[0.85 0.19 0.07 0.04]  ,'FontSize',14)  ;
            Handle.ViewButton{2}= uicontrol(Handle.Window,'Style','pushbutton','String','ViewY-: 30','units','normalized',...
                'Position',[0.85 0.15 0.07 0.04]  ,'FontSize',14)  ;
            Handle.ViewButton{3}= uicontrol(Handle.Window,'Style','pushbutton','String','ViewX+: 130','units','normalized',...
                'Position',[0.92 0.19 0.07 0.04]  ,'FontSize',14)  ;
            Handle.ViewButton{4}= uicontrol(Handle.Window,'Style','pushbutton','String','ViewY+: 30','units','normalized',...
                'Position',[0.92 0.15 0.07 0.04]  ,'FontSize',14)  ;

            Handle.ViewButton{1}.Callback = @(HObject, Eventdata) ...
                ROBGUI.ViewButtonMethodXminus(HObject, Eventdata, Handle);
            Handle.ViewButton{2}.Callback = @(HObject, Eventdata) ...
                ROBGUI.ViewButtonMethodYminus(HObject, Eventdata, Handle);


            Handle.ViewButton{3}.Callback = @(HObject, Eventdata) ...
                ROBGUI.ViewButtonMethodXplus(HObject, Eventdata, Handle);
            Handle.ViewButton{4}.Callback = @(HObject, Eventdata) ...
                ROBGUI.ViewButtonMethodYplus(HObject, Eventdata, Handle);

        end

        %% View method
        function HObject = ViewButtonMethodXminus(HObject, Eventdata, Handle)

            viewx = Handle.ViewButton{1}.String ;
            colon = find(viewx==':') ;
            anglex = viewx(colon+1:end) ;
            anglex = str2num(anglex) - 1 ;
            Handle.ViewButton{1}.String = ['ViewX-: ', num2str(anglex)] ;
            Handle.ViewButton{3}.String = ['ViewX+: ', num2str(anglex)] ;
        end
        function HObject = ViewButtonMethodXplus(HObject, Eventdata, Handle)

            viewx = Handle.ViewButton{1}.String ;
            colon = find(viewx==':') ;
            anglex = viewx(colon+1:end) ;
            anglex = str2num(anglex) + 1 ;
            Handle.ViewButton{1}.String = ['ViewX-: ', num2str(anglex)] ;
            Handle.ViewButton{3}.String = ['ViewX+: ', num2str(anglex)] ;
        end

        function HObject = ViewButtonMethodYminus(HObject, Eventdata, Handle)
            viewx = Handle.ViewButton{2}.String ;
            colon = find(viewx==':') ;
            anglex = viewx(colon+1:end) ;
            anglex = str2num(anglex) - 1 ;
            Handle.ViewButton{2}.String = ['ViewY-: ', num2str(anglex)] ;
            Handle.ViewButton{4}.String = ['ViewY+: ', num2str(anglex)] ;
        end

        function HObject = ViewButtonMethodYplus(HObject, Eventdata, Handle)
            viewx = Handle.ViewButton{2}.String ;
            colon = find(viewx==':') ;
            anglex = viewx(colon+1:end) ;
            anglex = str2num(anglex) + 1 ;
            Handle.ViewButton{2}.String = ['ViewY-: ', num2str(anglex)] ;
            Handle.ViewButton{4}.String = ['ViewY+: ', num2str(anglex)] ;
        end


        function Handle = CreatePlotButton(Handle)
            Handle.Button.PLOT = uicontrol(Handle.Window,'string','PLOT','units','normalized',...
                'position',[0.85 0.1 0.15 0.05], 'fontsize', 15 ) ;
            Handle.Button.PLOT.Callback =  @(HObject, Eventdata)...
                ROBGUI.PlotMethod(HObject, Eventdata, Handle);
        end
        % plot method
        function HObject = PlotMethod(HObject, Eventdata, Handle)
            ImJ = @ ROBGUI.ImportJointData ;
            % disp('Hello this is plot method')
            Vfile = cell(1,2) ; Part  = cell(1,2) ;
            for i = 1: 2
                Vfile{i} = Handle.FileListBox{i}.Value ;
            end

            for i = 1:2
                Part{i} =  Handle.FileListBox{i}.String{Vfile{i}} ;

            end

            Popvalue =  Handle.Popmenu.Value ;
            Path = Handle.Popmenu.String{Popvalue} ;
            pathjp = [Path, '/',Part{1}] ;
            pathjt = [Path, '/',Part{2}] ;
            JointP = ImJ(pathjp, [2 inf]) ;
            JointT = ImJ(pathjt, [2 inf]) ;
            rad = pi/180 ;
        

              Len = min( size(JointP,1),size(JointT,1) ) ;
            
            JointP  = JointP(1:Len,:) ; JointT  = JointT(1:Len,:) ; 
            Difff = ROBGUI.CDF_Int(JointP(:,1)) ;
            NumP = size( JointT, 1) ;
            dt = 0.1 ;
            Time  = linspace(1,NumP,NumP) * dt ;
            JointS = Difff * JointP /dt ;
            JointA = Difff * JointS / dt ;
            JointS(end-2:end,:) = repmat(JointS(end-3,:), [3 1]) ;
            JointA(end-3:end,:) = repmat(JointA(end-4,:), [4 1] );


            Sv = Handle.Slider.Value ;

            % load(ROBGUI.GETSaveFileName) ;
            Index = max(1,round(size(JointT,1) * Sv)) ;
            % predifine the view angle point
                StrangleX = Handle.ViewButton{1}.String ;
                idx = find(StrangleX==':') ; AngleX = str2double(StrangleX(idx+1:end));
                StrangleY = Handle.ViewButton{2}.String ;
                idy = find(StrangleY==':') ; AngleY = str2double(StrangleY(idy+1:end));
            %% second figure
            if Handle.ButtonGroup.Son{1}.Value
                Handle.Screen.Panel{1}.Visible= 'on' ;
                Handle.Screen.Panel{2}.Visible= 'off' ;
                Handle.Screen.Panel{3}.Visible= 'off' ;
                %%%%%%%%%%%%%%%  first subfigure     %%%%%%%%%%%%%%%

                des1 = Handle.Screen.Subfig{1,1} ;
                Col = 'grbmcy' ;
                hold(des1,'off');
                for inline = 1:6
                    plot(des1,Time, JointP(:,inline),'color',Col(inline),'linewidth', 1.5 ) ;
                    hold(des1,'on')
                end
                plot( des1,Time,Time * 0, '--','linewidth',1.5,'color',[1 1 1]*0.7)
                ymin1 = min(min(JointP(:,1:6))) ; ymax1 = max(max(JointP(:,1:6))) ;
                ymin1 = ymin1-0.1*abs(ymin1) ;    ymax1 = ymax1 + 0.1*abs(ymax1) ;
                plot( des1,Time(Index)*[1 1],[ymin1 ymax1] , '--k')

                axis(des1, [0 Time(end) ymin1 ymax1]) % adjust the area of axis
                L = legend(des1,'Jp1', 'Jp2', 'Jp3', 'Jp4','Jp5','Jp6') ;
                L.Position = [0.49, 0.82, 0.06, 0.12] ;
                title(des1, 'Joint Configuration','fontsize', 14)
                grid(des1,'on'); grid(des1,'minor');
                %%%%%%%%%%%%%%%  second subfigure     %%%%%%%%%%%%%%%
                des2 =  Handle.Screen.Subfig{1,2} ;
                hold( des2,'off' )
                for inline = 1 : 6
                    plot( des2,Time, JointS(:,inline),'color',...
                        Col(inline),'linewidth', 1.5) ; hold( des2,'on' )
                end
                plot( des2,Time,Time * 0,'--','linewidth',1.5,'color',[1 1 1]*0.7)
                ymin2 = min(min(JointS(:,1:6))) ; ymax2 = max(max(JointS(:,1:6))) ;
                ymin2 = ymin2-0.1*abs(ymin2) ; ymax2 = ymax2 + 0.1*abs(ymax2) ;
                plot( des2,Time(Index)*[1 1],[ymin2 ymax2] , '--k')
                axis(des2, [0 Time(end) ymin2 ymax2])
                L2 = legend(des2,'Js1', 'Js2', 'Js3', 'Js4','Js5','Js6') ;
                L2.Position = [0.485, 0.49, 0.06, 0.12] ;
                grid(des2,'on'); grid(des2,'minor'); 
                title(des2, 'Joint Speed','FontSize',14) ;

                %%%%%%%%%%%%%%%  third subfigure     %%%%%%%%%%%%%%%
                des3 =  Handle.Screen.Subfig{1,3} ;
                hold( des3,'off' )
                for inline = 1 : 6
                    plot( des3,Time, JointA(:,inline),'color',...
                        Col(inline),'linewidth', 1.5) ; hold( des3,'on' )
                end
                ymin3 = min(min(JointA(:,1:6))) ; ymax3 = max(max(JointA(:,1:6))) ;
                ymin3 = ymin3-0.1*abs(ymin3) ; ymax3 = ymax3 + 0.1*abs(ymax3) ;
                plot( des3,Time(Index)*[1 1],[ymin3 ymax3] , '--k')
                axis(des3, [0 Time(end) ymin3 ymax3])
                title(des3, 'Joint Acceleration','FontSize',14) ;
                L3 = legend(des3,'Ja1', 'Ja2', 'Ja3', 'Ja4','Ja5','Ja6') ;
                L3.Position = [0.485, 0.17, 0.06, 0.12] ;
                 grid(des3,'on'); grid(des3,'minor'); 



                

                
                    
                %%%%%%%%%%%%%%%  fourth subfigure     %%%%%%%%%%%%%%%
                des4 = Handle.Screen.Subfig{1,4} ;  % assign the destination to plot
                Joint_Theta= JointP(Index,:) ;
                % %       ROBGUI.DrawRob_now(Joint_Theta, des3) ;
                ROBGUI.DrawRob_now(des4, Joint_Theta ) ;
                view(des4,AngleX, AngleY)
                Points_all = ROBGUI.TraceCalculation(JointP) ;

                
                plot3(des4, Points_all(:,1),  Points_all(:,2), Points_all(:,3),'--b') ;
                DivNum = 80 ;
                IndexArrow = round(linspace(1, NumP, DivNum+1) ) ;

                Point = Points_all(IndexArrow,:) ;
                Vector = Point* 0 ;
                for i = 1: DivNum

                    Vector(i,:) = Point(i+1,:) - Point(i,:) ;

                end
                quiver3(des4, Point(:,1),  Point(:,2), Point(:,3), ...
                    Vector(:,1), Vector(:,2), Vector(:,3),'color','r','linewidth', 1 ,'linestyle','--')
                cord = Points_all(Index,:);
                Content = {'Visualized Current Robot Pose and Orientation',...
                    ['Current Index: ', num2str(Index)],...
                    ['Coordinate: ',sprintf('%4.2f, %4.2f, %4.2f', cord)]} ;
                title(des4,Content,'fontsize',14)
                drawnow
              

            elseif Handle.ButtonGroup.Son{2}.Value
                Handle.Screen.Panel{1}.Visible= 'off' ;
                Handle.Screen.Panel{2}.Visible= 'on' ;
                Handle.Screen.Panel{3}.Visible= 'off' ;
                des1 = Handle.Screen.Subfig{2,1}  ;
             
                hold(des1, 'off') ;
                % picture 2
                Col = 'grbmcy' ;
                axnum = 3 ;
                for line = 1:3
                plot(des1, Time,JointT(:, line) ,Col(line), 'linewidth', 1.5 )  ;hold(des1, 'on')
                end
                plot( des1,Time,Time * 0, '--','linewidth',1.5,'color',[1 1 1]*0.7)
                ymin1 = min(min(JointT(:,1:3))) ; ymax1 = max(max(JointT(:,1:6))) ;
                ymin1 = ymin1-0.1*abs(ymin1) ;    ymax1 = ymax1 + 0.1*abs(ymax1) ;
                plot( des1,Time(Index)*[1 1],[ymin1 ymax1] , '--k')
                grid(des1,'on'); grid(des1,'minor'); 

                axis(des1, [0 Time(end) ymin1 ymax1]) % adjust the area of axis


                axis(des1,'normal')
                title(des1, ['The Joint Torque for axis 1 - 3 '] ,...
                    'fontsize', 14);
                L1 = legend(des1, 'JT1', 'JT2', 'JT3') ;
                L1.Position = [0.485, 0.84, 0.06,    0.09] ;
                

                des2 = Handle.Screen.Subfig{2,2}  ; % plot the volocity
                hold(des2, 'off')
                for line = 4 : 6
                    plot(des2, Time,JointT(:, line) ,Col(line), 'linewidth', 1.5 )  ;hold(des2, 'on')
                end
                plot( des2,Time,Time * 0, '--','linewidth',1.5,'color',[1 1 1]*0.7)
                ymin2 = min(min(JointT(:,4:6))) ; ymax2 = max(max(JointT(:,4:6))) ;
                ymin2 = ymin2-0.1*abs(ymin2) ;    ymax2 = ymax2 + 0.1*abs(ymax2) ;
                plot( des2,Time(Index)*[1 1],[ymin2 ymax2] , '--k')
                axis(des2, [0 Time(end) ymin2 ymax2]) % adjust the area of axis
                axis(des2,'normal') ;  grid(des2,'on'); grid(des2,'minor'); 
                L2 = legend(des2,'JT 4', 'JT 5', 'JT 6') ;
                L2.Position = [0.485, 0.36 ,   0.06 ,   0.07] ;
                              title(des2, ['The Joint Torque for axis 4 - 6 '] ,...
                    'fontsize', 14);
                % des2.Color = [1 1 1 ] * 0.9;

                %%%%%%%%%%%%% third sub figure
                des3 = Handle.Screen.Subfig{2,3} ;
                Joint_Theta= JointP(Index,:) ;
                ROBGUI.DrawRob_now(des3, Joint_Theta ) ;
                view(des3,AngleX, AngleY) 
                Points_all = ROBGUI.TraceCalculation(JointP) ;
                plot3(des3, Points_all(:,1),  Points_all(:,2), Points_all(:,3),'--b') ;
                Content = {'Visualized Current Robot Pose and Orientation',...
                    ['Current Index: ', num2str(Index)]} ;
                title(des3,Content,'fontsize',14)
                 drawnow ;
                %         title(des3,'Visualized Current Robot Pose and Orientation','fontsize',14)
            elseif Handle.ButtonGroup.Son{3}.Value
                Handle.Screen.EDIT.String = cell(1,10) ;
                % calculate the velocity
                [Vlink, Vworld, Alink, Aworld, ...
                    Plink, dPlink, ddPlink]  = NewtonEulerRec.ForwardRecursive(JointP, dt) ;
                Handle.Screen.EDIT.String{1} = ['Index :', num2str(Index)] ;

                axisnum = 5 ;
                Handle.Screen.EDIT.String{2} = ['Velocity referece to Frame ', num2str(axisnum)] ;
                Handle.Screen.EDIT.String{3} =...
                    sprintf('%.3f, %.3f, %.3f, %.3f, %.3f, %.3f ', Vlink{1,axisnum}(Index,:) ) ;
                Handle.Screen.EDIT.String{4} = ['Velocity referece to world for axis ', num2str(axisnum)] ;
                Handle.Screen.EDIT.String{5} =...
                    sprintf('%.3f, %.3f, %.3f, %.3f, %.3f, %.3f ', Vworld{1,axisnum}(Index,:) ) ;
                Handle.Screen.EDIT.String{6} = ['Acceleration referece to Frame ', num2str(axisnum)] ;
                Handle.Screen.EDIT.String{7} = ...
                    sprintf('%.3f, %.3f, %.3f, %.3f, %.3f, %.3f ', Alink{1,axisnum}(Index,:) ) ;
                Handle.Screen.EDIT.String{8} = ['Acceleration referece to world for axis ', num2str(axisnum)] ;
                Handle.Screen.EDIT.String{9} = ...
                    sprintf('%.3f, %.3f, %.3f, %.3f, %.3f, %.3f ', Aworld{1,axisnum}(Index,:) ) ;

                Handle.Screen.EDIT.String{10} = ['Net force for link ', num2str(axisnum) ] ;


                %%%%%%%%%%%%%%%  third subfigure     %%%%%%%%%%%%%%%
                des3 = Handle.Screen.Subfig{3,3} ;  % assign the destination to plot
                Joint_Theta= JointP(Index,:) ;
                % %       ROBGUI.DrawRob_now(Joint_Theta, des3) ;
                ROBGUI.DrawRob_now(des3, Joint_Theta ) ;
                Points_all = ROBGUI.TraceCalculation(JointP) ;
                plot3(des3, Points_all(:,1),  Points_all(:,2), Points_all(:,3),'--b') ;
                Content = {'Visualized Current Robot Pose and Orientation',...
                    ['Current Index: ', num2str(Index)]} ;
                title(des3,Content,'fontsize',14)
                %         Handle.Screen.EDIT.String =  {'firstline', 'second line', 'Line 3',...
                %           }

            end

        end


    end
end

