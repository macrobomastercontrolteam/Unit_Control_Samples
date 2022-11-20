@echo off

setlocal enabledelayedexpansion

call Scripts\_errorlevel_handler.bat
call Scripts\_parse_Keil_project.bat
%ifErr% echo( &(%errExit%)
call Scripts\_check_validity.bat
%ifErr% echo( &(%errExit%)

echo(
if exist "Scripts\_increment_version.bat" (
  call Scripts\_increment_version.bat
)

echo(
echo The project for ALL configurations is cleaning. Please wait...
call Scripts\_delete_output_folders.bat
call Scripts\_create_folders.bat
break>"%TMP_FOLDER%\program_size_summary.log"

call Scripts\clean.bat
%ifErr% echo( &(%errExit%)
echo(
echo Done.
