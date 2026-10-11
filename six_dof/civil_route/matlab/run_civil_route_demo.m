function summary=run_civil_route_demo(executablePath,outputDir,scene,duration,dt,stride,departure,enroute,approach)
% MATLAB frontend for the C reference implementation; no separate dynamics.
% Build civil_route_demo first. Defaults use launch/direct/slew controllers.
assert(nargin>=1,'Supply the compiled civil_route_demo executable path');
if nargin<2||isempty(outputDir),outputDir=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results_matlab');end
if nargin<3,scene='crossing';end
if nargin<4,duration=120;end
if nargin<5,dt=0.005;end
if nargin<6,stride=10;end
if nargin<7,departure='launch';end
if nargin<8,enroute='direct';end
if nargin<9,approach='slew';end
executablePath=char(executablePath);outputDir=char(outputDir);scene=char(scene);
names={char(departure),char(enroute),char(approach)};
assert(ismember(scene,{'high','crossing','monitor'}),'Unknown scene');
assert(all(ismember(names,{'launch','direct','slew'})),'Unknown controller');
assert(isscalar(duration)&&isfinite(duration)&&duration>0&&duration<=3600,'Invalid duration');
assert(isscalar(dt)&&isfinite(dt)&&dt>0&&ceil(duration/dt)<=2000000,'Invalid dt');
assert(isscalar(stride)&&isfinite(stride)&&stride>=1&&stride==floor(stride),'Invalid stride');
assert(exist(executablePath,'file')==2,'Compile the C executable first');
% Reject shell expansion characters in the two user-supplied paths.
for value={executablePath,outputDir}
    forbidden={'"','''','`','$','%','!',sprintf('\n'),sprintf('\r')};
    assert(~any(contains(value{1},forbidden)),'Unsupported shell characters in path');
end
if ~exist(outputDir,'dir'),mkdir(outputDir);end
csvPath=fullfile(outputDir,[scene,'.csv']);
command=sprintf('"%s" %s "%s" %.17g %.17g %.0f %s %s %s', ...
    executablePath,scene,csvPath,duration,dt,stride,names{1},names{2},names{3});
[status,message]=system(command);assert(status==0,'C simulation failed: %s',message);
summary=jsondecode(fileread([csvPath,'.json']));plot_saved_civil_route(outputDir,scene);disp(message);
end
