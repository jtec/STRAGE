% Adds the necessary files and folders to the matlab search path.
pathOfThisScript_STRAGE = mfilename('fullpath');
[pathstr, name, ext] = fileparts(pathOfThisScript_STRAGE);
% First add everything in the current folder.
addpath(genpath(pathstr));
% Remove .git folder since it does not contain anything relevant
% to matlab
garagefolderPath = strcat(pathstr, filesep, '.git');
rmpath(genpath(garagefolderPath));

disp([mfilename '>> Added STRAGE tool set to matlab path.'])
