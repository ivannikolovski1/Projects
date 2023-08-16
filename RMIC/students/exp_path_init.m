clearvars
close all
clc

cd(fileparts(matlab.desktop.editor.getActiveFilename)); % use this to CD

this_path = fileparts( mfilename('fullpath') );

addpath(fullfile(this_path, 'controller'));
addpath(fullfile(this_path, 'main'));
addpath(fullfile(this_path, 'model'));
addpath(fullfile(this_path, 'plotting'));
addpath(fullfile(this_path, 'util'));
