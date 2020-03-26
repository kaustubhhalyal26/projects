%% Copyright 2013-2015 The MathWorks, Inc.                                 
function varargout = IntruderDetectionApp(varargin)
% MOTIONDETECTIONAPP MATLAB code for MotionDetectionApp.fig
%      MOTIONDETECTIONAPP, by itself, creates a new MOTIONDETECTIONAPP or raises the existing
%      singleton*.
%
%      H = MOTIONDETECTIONAPP returns the handle to a new MOTIONDETECTIONAPP or the handle to
%      the existing singleton*.
%
%      MOTIONDETECTIONAPP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MOTIONDETECTIONAPP.M with the given input arguments.
%
%      MOTIONDETECTIONAPP('Property','Value',...) creates a new MOTIONDETECTIONAPP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MotionDetectionApp_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MotionDetectionApp_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MotionDetectionApp

% Last Modified by GUIDE v2.5 05-Mar-2014 20:36:10

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @IntruderDetectionApp_OpeningFcn, ...
                   'gui_OutputFcn',  @IntruderDetectionApp_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before MotionDetectionApp is made visible.
function IntruderDetectionApp_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to IntruderDetectionApp (see VARARGIN)

% Choose default command line output for IntruderDetectionApp
handles.output = hObject;

% Reset IMAQ
imaqreset;

% Set up image acquisition
% handles.hCamera = videoinput('winvideo', 1, 'RGB24_640x480');
handles.hCamera = webcam;
% handles.hCamera.ReturnedColorspace = 'rgb';
% handles.hCamera.FramesPerTrigger =  1;

% Initialization of the camera
% triggerconfig(handles.hCamera, 'manual');
% start(handles.hCamera);

% Remove figure tickmarks
set(handles.axes1,'xtick',[])
set(handles.axes1,'ytick',[])
set(handles.axes2,'xtick',[])
set(handles.axes2,'ytick',[])
set(handles.axes3,'xtick',[])
set(handles.axes3,'ytick',[])

% Disable Start and Stop until reference image is captured
set(handles.startbutton,'enable','off');
set(handles.stopbutton,'enable','off');

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes IntruderDetectionApp wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = IntruderDetectionApp_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function threshslider_Callback(hObject, eventdata, handles)
% hObject    handle to threshslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
position = get(hObject,'Value');
disp(['Slider moved to position: ' num2str(100*position) '%']);

% --- Executes during object creation, after setting all properties.
function threshslider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to threshslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
set(hObject,'Value',0.5)

% --- Executes on slider movement.
function sizeslider_Callback(hObject, eventdata, handles)
% hObject    handle to sizeslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function sizeslider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sizeslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
set(hObject,'Value',0.5)

% --- Executes on button press in referencebutton.
function referencebutton_Callback(hObject, eventdata, handles)
% hObject    handle to referencebutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get reference image
handles.ref_vid_img = snapshot(handles.hCamera);

% Display image
imshow(handles.ref_vid_img, 'Parent', handles.axes1)

% Enable Start and Stop buttons
set(handles.startbutton,'enable','on');
set(handles.stopbutton,'enable','on');

% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1


% --- Executes on button press in startbutton.
function startbutton_Callback(hObject, eventdata, handles)
% hObject    handle to startbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Disable Start button
set(handles.startbutton,'string','Running','enable','off');

% Loop while not stopped
buttonStr  = get(handles.startbutton,'String');
while strcmp(buttonStr,'Running')
    
    % Acquire an image from the webcam
    vid_img = snapshot(handles.hCamera);
    
    % Different threshold and target object size limit
    s = get(handles.sizeslider,'Value');
    t = get(handles.threshslider,'Value');
    objSize = 640*s;
    objThresh = 256*t;
    
    % Call the live segmentation function
    [highlighted_img, imgThresh, alert] = Segmentation_Fn_UI(vid_img, ...
                    handles.ref_vid_img, objThresh, objSize);
    
    % Update displays
    imshow(imgThresh, 'Parent', handles.axes2);
    imshow(highlighted_img, 'Parent', handles.axes3);
    drawnow;
    
    % Update alert sign
    if alert,
        set(handles.alertbutton,'string','ALERT');
        set(handles.alertbutton,'BackGroundColor',[0.95 0 0]);
    else
        set(handles.alertbutton,'String','All Clear');
        set(handles.alertbutton,'BackGroundColor',[0.5 0.5 0.5]);
    end;
    
    % get startbutton String to determine running state 
    % (startbutton string gets reset by stopbutton callback)
    buttonStr  = get(handles.startbutton,'String');
end;

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in alertbutton.
function alertbutton_Callback(hObject, eventdata, handles)
% hObject    handle to alertbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in stopbutton.
function stopbutton_Callback(hObject, eventdata, handles)
% hObject    handle to stopbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Enable Start button
set(handles.startbutton,'string','Start','enable','on');

% Update handles structure
guidata(hObject, handles);


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure

% Stop loop in startbutton callback
% Disable startbutton and wait a moment for startbutton loop to catch it
set(handles.startbutton,'string','Start','enable','on');
guidata(hObject, handles);
pause(0.5)


% Close figure
delete(hObject);
