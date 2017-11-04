function cropFinder
    % this is a newer version of viewSplitter, in which user defined ROIs
    % and matlab did the cropping... now it simply displays the selected
    % ROIs so they can be entered into ffmpeg bat file... Still has a lot
    % of code for doing the actual analysis that should be removed...
    
    % load video
    [file, path] = uigetfile('C:\Users\sawtell\Desktop\github\obstacleRig\bonsai\vid acquisition - webCam\*.*', 'Select video file to analyze...');
    vid = VideoReader([path file]);

    % prepare figure
    frameInd = 1;
    frame = vid.read(1);
    figure('units', 'normalized', 'position', [0 0 1 1]);
    subplot(1,1,1);
    preview = imshow(frame);
    isRunning = true; 
    horzSliderSteps = [1/vid.Width 5/vid.Width];
    vertSliderSteps = [1/vid.Height 5/vid.Height];
    
    % load previous settings
    if exist('cropFinderPrevSettings.mat', 'file')
        load('cropFinderPrevSettings.mat', 'linePosits');
    else
        linePosits = [1 1 1 1 1 1];
    end
    
    % prepare lines
    v1 = line([linePosits(1) linePosits(1)], [1 size(frame,1)]);
    v2 = line([linePosits(2) linePosits(2)], [1 size(frame,1)]);
    h1 = line([1 size(frame,2)], [linePosits(3) linePosits(3)]);
    h2 = line([1 size(frame,2)], [linePosits(4) linePosits(4)]);
    h3 = line([1 size(frame,2)], [linePosits(5) linePosits(5)]);
    h4 = line([1 size(frame,2)], [linePosits(6) linePosits(6)]);
    [sliderv1, sliderv2, sliderh1, sliderh2, sliderh3, sliderh4] = prepareSliders();
    
    % preview analysis
    while isRunning
        % preview frame
        frame = vid.read(frameInd);
        set(preview, 'CData', frame);

        % update frame counter
        if frameInd==vid.NumberOfFrames; frameInd = 1; else; frameInd=frameInd+1; end
        pause(.1);
    end
    
    % ---------
    % FUNCTIONS
    % ---------
    
    % prepare buttons and sliders
    function [sliderv1, sliderv2, sliderh1, sliderh2, sliderh3, sliderh4] = prepareSliders(~,~)
        % BUTTONS
        
        % close and save settings
        uicontrol('Style', 'pushbutton',...
            'Position', [0 120 100 20],...
            'String', 'close/save settings',...
            'Callback', @closeAndSave);
        
        % process video
        uicontrol('Style', 'pushbutton',...
            'Position', [0 140 100 20],...
            'String', 'process video',...
            'Callback', @processVid);
        
        % POSITION SLIDERS
        sliderv1 = uicontrol('Style', 'slider',...
            'Position', [0 0 200 20],...
            'Min', 1, 'Max', size(frame,2),...
            'Value', v1.XData(1),...
            'Callback', @v1Update,...
            'SliderStep', vertSliderSteps);
        uicontrol('Style', 'text', 'String', 'v1', 'Position', [200 0 40 20])
        
        sliderv2 = uicontrol('Style', 'slider',...
            'Position', [0 20 200 20],...
            'Min', 1, 'Max', size(frame,2),...
            'Value', v2.XData(1),...
            'Callback', @v2Update,...
            'SliderStep', vertSliderSteps);
        uicontrol('Style', 'text', 'String', 'v2', 'Position', [200 20 40 20])
        
        sliderh1 = uicontrol('Style', 'slider',...
            'Position', [0 40 200 20],...
            'Min', 1, 'Max', size(frame,1),...
            'Value', h1.YData(1),...
            'Callback', @h1Update,...
            'SliderStep', horzSliderSteps);
        uicontrol('Style', 'text', 'String', 'h1', 'Position', [200 40 40 20])
        
        sliderh2 = uicontrol('Style', 'slider',...
            'Position', [0 60 200 20],...
            'Min', 1, 'Max', size(frame,1),...
            'Value', h2.YData(1),...
            'Callback', @h2Update,...
            'SliderStep', horzSliderSteps);
        uicontrol('Style', 'text', 'String', 'h2', 'Position', [200 60 40 20])
        
        sliderh3 = uicontrol('Style', 'slider',...
            'Position', [0 80 200 20],...
            'Min', 1, 'Max', size(frame,1),...
            'Value', h3.YData(1),...
            'Callback', @h3Update,...
            'SliderStep', horzSliderSteps);
        uicontrol('Style', 'text', 'String', 'h3', 'Position', [200 80 40 20])
        
        sliderh4 = uicontrol('Style', 'slider',...
            'Position', [0 100 200 20],...
            'Min', 1, 'Max', size(frame,1),...
            'Value', h4.YData(1),...
            'Callback', @h4Update,...
            'SliderStep', horzSliderSteps);
        uicontrol('Style', 'text', 'String', 'h4', 'Position', [200 100 40 20])
    end

    % close and save settings
    function closeAndSave(~,~)
        % diplay positions that can be used to crop in ffmpeg
        ys = max(sort(linePosits(3:6)), 1);
        xs = max(sort(linePosits(1:2)), 1);
        disp('crop settings for ffmpeg:')
        fprintf('top-> %d:%d:%d:%d\n', diff(xs), ys(2)-ys(1), xs(1), ys(1))
        fprintf('bot-> %d:%d:%d:%d\n', diff(xs), ys(4)-ys(3), xs(1), ys(3))
        
        isRunning = false;
        save('cropFinderPrevSettings.mat', 'linePosits')
        close all;
        
    end

    % slider position updates
    function v1Update(~,~); val = round(get(sliderv1, 'value')); v1.XData = [val val]; linePosits(1)=val; end
    function v2Update(~,~); val = round(get(sliderv2, 'value')); v2.XData = [val val]; linePosits(2)=val; end
    function h1Update(~,~); val = round(get(sliderh1, 'value')); h1.YData = [val val]; linePosits(3)=val; end
    function h2Update(~,~); val = round(get(sliderh2, 'value')); h2.YData = [val val]; linePosits(4)=val; end
    function h3Update(~,~); val = round(get(sliderh3, 'value')); h3.YData = [val val]; linePosits(5)=val; end
    function h4Update(~,~); val = round(get(sliderh4, 'value')); h4.YData = [val val]; linePosits(6)=val; end
    
    % process video
    function processVid(~,~)
        isRunning = false;
        close all;
        
        % get bounding positions
        horizontals =  sort(linePosits(3:6));
        xLim = sort(linePosits(1:2));
        yLimTop = horizontals(1:2);
        yLimBot = horizontals(3:4);
        
        % write video
        w = waitbar(0, 'processing video...');
        topWrite = VideoWriter(['split video\' file(1:end-4) 'top.mp4'], 'MPEG-4');
        set(topWrite, 'FrameRate', vid.framerate); open(topWrite);
        botWrite = VideoWriter(['split video\' file(1:end-4) 'bot.mp4'], 'MPEG-4');
        set(botWrite, 'FrameRate', vid.framerate); open(botWrite);
                
        % split frames
        for i=1:vid.NumberOfFrames
            frame = read(vid, i);
            frameTop = frame(yLimTop(1):yLimTop(2), xLim(1):xLim(2),:);
            frameBot = frame(yLimBot(1):yLimBot(2), xLim(1):xLim(2),:);
            writeVideo(topWrite, frameTop);
            writeVideo(botWrite, frameBot);
            waitbar(i/vid.numberofframes);
        end
        
        % wrap things up
        close(topWrite); close(botWrite); close(w);
    end 
end


