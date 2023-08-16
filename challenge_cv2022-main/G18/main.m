classdef main < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                      matlab.ui.Figure
        GridLayout                    matlab.ui.container.GridLayout
        TextArea                      matlab.ui.control.TextArea
        Label                         matlab.ui.control.Label
        SelectInnerrectangleButton    matlab.ui.control.Button
        ClearForegroundObjectsButton  matlab.ui.control.Button
        DoneButton                    matlab.ui.control.Button
        RestartButton                 matlab.ui.control.Button
        SelectVanishingpointButton    matlab.ui.control.Button
        SelectForegroundObjectButton  matlab.ui.control.Button
        SelectImageButton             matlab.ui.control.Button
        UIImageAxes                   matlab.ui.control.UIAxes
    end

    
    properties (Access = public)
        innerRectangle = [];
        point = [];
        img = [];
        lt=[];
        lb=[];
        rt=[];
        rb=[];
        ysz=0;
        xsz=0;
        cd=0;
        FgStruct=struct([]);

        innerRectangleUiObj = [];
        pointUiObj = [];
        foregroundRectangleUiObj = [];
        i=1;
    end
    
    methods (Access = private)
        
        function coords = rectangle_to_coords(app,rect)
            x1 = floor(rect.Position(1));
            y1 = floor(rect.Position(2));
            x2 = ceil(x1 + rect.Position(3)) - 1;
            y2 = ceil(y1 + rect.Position(4)) - 1;
            coords = [x1 y1 x2 y2];
        end

        function app=plotlines(app)
            [x1,y1]=utils.findCorner(app.point(1),app.point(2),app.innerRectangle(1),app.innerRectangle(2),0,0);
            [x2,y2]=utils.findCorner(app.point(1),app.point(2),app.innerRectangle(3),app.innerRectangle(2),app.xsz,0);
            [x3,y3]=utils.findCorner(app.point(1),app.point(2),app.innerRectangle(3),app.innerRectangle(4),app.xsz,app.ysz);
            [x4,y4]=utils.findCorner(app.point(1),app.point(2),app.innerRectangle(1),app.innerRectangle(4),0,app.ysz);
            app.lt=plot(app.UIImageAxes,[app.point(1) x1], [app.point(2) y1],'LineWidth',2,'Color','r');
            app.rt=plot(app.UIImageAxes,[app.point(1) x2], [app.point(2) y2],'LineWidth',2,'Color','r');
            app.rb=plot(app.UIImageAxes,[app.point(1) x3], [app.point(2) y3],'LineWidth',2,'Color','r');
            app.lb=plot(app.UIImageAxes,[app.point(1) x4], [app.point(2) y4],'LineWidth',2,'Color','r');
        end

        function app=deleteeverything(app)
            delete(app.innerRectangleUiObj);
            delete(app.pointUiObj);
            for j=1:app.i-1
                delete(app.FgStruct(j).roi);
            end
            if ~isempty(app.lt)
                delete(app.lt); delete(app.rt); delete(app.rb); delete(app.lb);
            end
        end
        
        function app=restoredefults(app)
           app.innerRectangle = [];
           app.point = [];
           app.lt=[];
           app.lb=[];
           app.rt=[];
           app.rb=[];
           app.FgStruct=struct([]);
           app.innerRectangleUiObj = [];
           app.pointUiObj = [];
           app.foregroundRectangleUiObj = [];
           app.i=1;
        end

        function app=noimage(app)
            app.img=[];
            app.ysz=0;
            app.xsz=0;
            app.cd=0;
        end

        function plotir(app)
            app.innerRectangle = rectangle_to_coords(app, app.innerRectangleUiObj);
            app.point = app.pointUiObj.Position;
            if ~isempty(app.lb)
                delete(app.lb); delete(app.lt); delete(app.rb); delete(app.rt);
            end
            hold(app.UIImageAxes, "on");
            app=plotlines(app);
            hold(app.UIImageAxes,"off");
        end

        function plotvp(app)
            app.innerRectangle = rectangle_to_coords(app, app.innerRectangleUiObj);
            %app.innerRectangle(app.innerRectangle == 0) = 1;
            app.point = app.pointUiObj.Position;
            if ~isempty(app.lb)
                delete(app.lb); delete(app.lt); delete(app.rb); delete(app.rt);
            end
            hold(app.UIImageAxes, "on");
            app=plotlines(app);
            hold(app.UIImageAxes,"off");
        end
        
    end
    
    methods (Access = public)

    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            %clear all;
            close all;
            clc;

            startPicture = imread('gui\StartingImage.jpeg');
            imshow(startPicture,'Parent',app.UIImageAxes);
            axis(app.UIImageAxes, 'off');
            
            % Adjusting axes to image size
            app.UIImageAxes.XLim = [0 size(startPicture, 2)];
            app.UIImageAxes.YLim = [0 size(startPicture, 1)];


        end

        % Button pushed function: SelectImageButton
        function selectImageCallback(app, event)
            if ~isempty(app.img)
                app=deleteeverything(app);
                app=restoredefults(app);
                app=noimage(app);
            end

            % Cleaning necessary UI elements
            app.TextArea.Visible = "on";
            app.TextArea.Value = "";

            % Loading image
            [file,path] = uigetfile({'*.*';'*.jpeg';'*.jpg';'*.png'},...
                          'File Selector', 'pictures/');
            abspath = strcat(path, '/', file);
            fprintf("Loading image from path: %s\n", abspath);
            try
                app.img = imread(abspath);
                [app.ysz,app.xsz,app.cd]=size(app.img);
            catch
                app.TextArea.Value = "Error while reading image. Did you select a valid image?";
                return;
            end

            figure(app.UIFigure);


            % Debug message
            fprintf("Image size: %d\n", size(app.img));
            app.TextArea.Value = "Image loaded";

            % Showing in axes
            imshow(app.img, 'Parent', app.UIImageAxes);
            axis(app.UIImageAxes, 'off');
            
            % Adjusting axes to image size
            app.UIImageAxes.XLim = [0 size(app.img, 2)];
            app.UIImageAxes.YLim = [0 size(app.img, 1)];
            


            % Setting UI elements visibility
            app.UIImageAxes.Visible = "on";
            app.SelectForegroundObjectButton.Visible = "on";
            app.SelectInnerrectangleButton.Visible = "on";
            app.SelectVanishingpointButton.Visible = "on";
            app.RestartButton.Visible = "on";
            app.DoneButton.Enable = "off";
            app.DoneButton.Visible = "on";
            app.ClearForegroundObjectsButton.Visible = "on";
            app.SelectImageButton.Visible = "on";

            % Disable interactivity (stop it from moving around)
            disableDefaultInteractivity(app.UIImageAxes);
        end

        % Button pushed function: SelectForegroundObjectButton
        function SelectForegroundObjectButtonPushed(app, event)
            app.foregroundRectangleUiObj = drawrectangle(app.UIImageAxes,'Color', 'y');
            app.FgStruct(app.i).roi = app.foregroundRectangleUiObj;
            app.i=app.i+1;
        end

        % Button pushed function: SelectInnerrectangleButton
        function selectInnerRectangleCallback(app, event)
            delete(app.innerRectangleUiObj);
            app.innerRectangleUiObj = drawrectangle(app.UIImageAxes);  
            addlistener(app.innerRectangleUiObj,"MovingROI",@movingir);
            function movingir(~,~)
                if ~isempty(app.pointUiObj)
                    plotir(app);
                end
            end
            if ~isempty(app.pointUiObj) && ~isempty(app.innerRectangleUiObj)
                plotir(app);
                app.DoneButton.Enable = "on";
            end
        end

        % Button pushed function: SelectVanishingpointButton
        function SelectVanishingPointButtonPushed(app, event)
            delete(app.pointUiObj);
            app.pointUiObj = drawpoint(app.UIImageAxes, 'Color', [0 1 0] , 'MarkerSize' , 10);
            addlistener(app.pointUiObj,"MovingROI",@movingvp);
            function movingvp(~,~)
                if ~isempty(app.innerRectangleUiObj)
                    plotvp(app);
                end
            end
            if ~isempty(app.pointUiObj) && ~isempty(app.innerRectangleUiObj)
                plotvp(app);
                app.DoneButton.Enable = "on";
            end
        end

        % Button pushed function: RestartButton
        function RestartButtonPushed(app, event)
            app=deleteeverything(app);
            app=restoredefults(app);
            startupFcn(app);
            app.ClearForegroundObjectsButton.Visible = 'off';
            app.DoneButton.Visible = 'off';
            app.RestartButton.Visible = 'off';
            app.SelectVanishingpointButton.Visible = 'off';
            app.SelectInnerrectangleButton.Visible = 'off';
            app.SelectForegroundObjectButton.Visible = 'off';
            app.TextArea.Value = 'Welcome! Please select an image.';
        end

        % Button pushed function: DoneButton
        function doneButtonPushed(app, event)

            % Cleaning possible error message
            app.TextArea.Value = "";

            % Collecting UI saved data
            app.innerRectangle = rectangle_to_coords(app, app.innerRectangleUiObj);
            %app.innerRectangle(app.innerRectangle == 0) = 1;
            try
                app.point = app.pointUiObj.Position;
            catch
                app.TextArea.Value = "Error while exporting. Please select a vanishing point and inner rectangle!";
                return;
            end

            utils.plotSurfaces(app.point, app.innerRectangle, app.img, app.FgStruct);
            
        end

        % Button pushed function: ClearForegroundObjectsButton
        function ClearForegroundObjectsButtonPushed(app, event)
            for j=1:app.i-1
                delete(app.FgStruct(j).roi);
                app.i=1;
            end
            app.FgStruct=struct([]);
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [230 100 962 719];
            app.UIFigure.Name = 'MATLAB App';

            % Create GridLayout
            app.GridLayout = uigridlayout(app.UIFigure);
            app.GridLayout.ColumnWidth = {'1.07x', 89, 63, 100, 52, 58, 42, 47, 100, 70, 82, '1x'};
            app.GridLayout.RowHeight = {23, '15.8x', '1x', 23, 25};
            app.GridLayout.ColumnSpacing = 20;
            app.GridLayout.RowSpacing = 9;

            % Create UIImageAxes
            app.UIImageAxes = uiaxes(app.GridLayout);
            app.UIImageAxes.XTick = [];
            app.UIImageAxes.YTick = [];
            app.UIImageAxes.Layout.Row = 2;
            app.UIImageAxes.Layout.Column = [1 12];
            app.UIImageAxes.Visible = 'off';

            % Create SelectImageButton
            app.SelectImageButton = uibutton(app.GridLayout, 'push');
            app.SelectImageButton.ButtonPushedFcn = createCallbackFcn(app, @selectImageCallback, true);
            app.SelectImageButton.Tag = 'SelectImageButton';
            app.SelectImageButton.BackgroundColor = [0.6549 0.5412 1];
            app.SelectImageButton.FontSize = 14;
            app.SelectImageButton.Layout.Row = 5;
            app.SelectImageButton.Layout.Column = [6 7];
            app.SelectImageButton.Text = 'Select image';

            % Create SelectForegroundObjectButton
            app.SelectForegroundObjectButton = uibutton(app.GridLayout, 'push');
            app.SelectForegroundObjectButton.ButtonPushedFcn = createCallbackFcn(app, @SelectForegroundObjectButtonPushed, true);
            app.SelectForegroundObjectButton.Visible = 'off';
            app.SelectForegroundObjectButton.Layout.Row = 4;
            app.SelectForegroundObjectButton.Layout.Column = [6 8];
            app.SelectForegroundObjectButton.Text = 'Select Foreground Object';

            % Create SelectVanishingpointButton
            app.SelectVanishingpointButton = uibutton(app.GridLayout, 'push');
            app.SelectVanishingpointButton.ButtonPushedFcn = createCallbackFcn(app, @SelectVanishingPointButtonPushed, true);
            app.SelectVanishingpointButton.Visible = 'off';
            app.SelectVanishingpointButton.Layout.Row = 4;
            app.SelectVanishingpointButton.Layout.Column = [4 5];
            app.SelectVanishingpointButton.Text = 'Select Vanishing point';

            % Create RestartButton
            app.RestartButton = uibutton(app.GridLayout, 'push');
            app.RestartButton.ButtonPushedFcn = createCallbackFcn(app, @RestartButtonPushed, true);
            app.RestartButton.BackgroundColor = [1 0.251 0.251];
            app.RestartButton.Visible = 'off';
            app.RestartButton.Layout.Row = 5;
            app.RestartButton.Layout.Column = 9;
            app.RestartButton.Text = 'Restart';

            % Create DoneButton
            app.DoneButton = uibutton(app.GridLayout, 'push');
            app.DoneButton.ButtonPushedFcn = createCallbackFcn(app, @doneButtonPushed, true);
            app.DoneButton.BackgroundColor = [0.3922 0.8314 0.0745];
            app.DoneButton.Enable = 'off';
            app.DoneButton.Visible = 'off';
            app.DoneButton.Layout.Row = 5;
            app.DoneButton.Layout.Column = 4;
            app.DoneButton.Text = 'Done';

            % Create ClearForegroundObjectsButton
            app.ClearForegroundObjectsButton = uibutton(app.GridLayout, 'push');
            app.ClearForegroundObjectsButton.ButtonPushedFcn = createCallbackFcn(app, @ClearForegroundObjectsButtonPushed, true);
            app.ClearForegroundObjectsButton.Visible = 'off';
            app.ClearForegroundObjectsButton.Layout.Row = 4;
            app.ClearForegroundObjectsButton.Layout.Column = [9 10];
            app.ClearForegroundObjectsButton.Text = 'Clear Foreground Objects';

            % Create SelectInnerrectangleButton
            app.SelectInnerrectangleButton = uibutton(app.GridLayout, 'push');
            app.SelectInnerrectangleButton.ButtonPushedFcn = createCallbackFcn(app, @selectInnerRectangleCallback, true);
            app.SelectInnerrectangleButton.Visible = 'off';
            app.SelectInnerrectangleButton.Layout.Row = 4;
            app.SelectInnerrectangleButton.Layout.Column = [2 3];
            app.SelectInnerrectangleButton.Text = 'Select background';

            % Create Label
            app.Label = uilabel(app.GridLayout);
            app.Label.BackgroundColor = [0.9412 0.9412 0.9412];
            app.Label.HorizontalAlignment = 'right';
            app.Label.FontColor = [1 0 0];
            app.Label.Visible = 'off';
            app.Label.Layout.Row = 1;
            app.Label.Layout.Column = 2;
            app.Label.Text = '';

            % Create TextArea
            app.TextArea = uitextarea(app.GridLayout);
            app.TextArea.Editable = 'off';
            app.TextArea.HorizontalAlignment = 'center';
            app.TextArea.WordWrap = 'off';
            app.TextArea.FontSize = 14;
            app.TextArea.FontWeight = 'bold';
            app.TextArea.FontColor = [1 0 0];
            app.TextArea.BackgroundColor = [0.9412 0.9412 0.9412];
            app.TextArea.Layout.Row = 1;
            app.TextArea.Layout.Column = [3 10];
            app.TextArea.Value = {'Welcome! Please select an image.'};

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = main

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end