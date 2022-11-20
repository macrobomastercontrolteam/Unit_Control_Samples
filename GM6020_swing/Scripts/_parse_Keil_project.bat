@echo off

call Scripts\_errorlevel_handler.bat
SET KEIL_PROJECT_PATH=MDK-ARM\GM6020_demo.uvprojx
python ./Scripts/_parse_Keil_project.py "%KEIL_PROJECT_PATH%" .vscode/c_cpp_properties.json ./Scripts/_set_Keil_project_paths.bat
call Scripts\_set_Keil_project_paths.bat
%ifErr% echo( &(%errExit%)
