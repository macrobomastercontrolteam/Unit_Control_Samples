@echo off

setlocal enabledelayedexpansion

call Scripts\_errorlevel_handler.bat

IF "%~1"=="" (
  SET PROJECT_TARGET=Release
) ELSE (
  SET PROJECT_TARGET=%~1
)

call Scripts\_parse_Keil_project.bat
%ifErr% echo( &(%errExit%)
call Scripts\_check_validity.bat
%ifErr% echo( &(%errExit%)
call Scripts\_create_folders.bat
%ifErr% echo( &(%errExit%)
call Scripts\_create_folders_for_target.bat
%ifErr% echo( &(%errExit%)

call Scripts\build.bat
%ifErr% echo( &(%errExit%)
call Scripts\convert.bat
%ifErr% echo( &(%errExit%)

echo(
echo %PROJECT_TARGET% building is done.
