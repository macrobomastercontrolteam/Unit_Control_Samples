@echo off

call Scripts\_errorlevel_handler.bat

where UV4.exe
%ifErr% echo UV4.exe was not found. &(%errExit%)

echo(
echo PROJECT_FOLDER: "%PROJECT_FOLDER%"
echo KEIL_PROJECT_NAME_UVPROJX: "%KEIL_PROJECT_NAME_UVPROJX%"
echo BUILD_LOG: "%BUILD_LOG%"

call Scripts\_check_validity.bat
%ifErr% echo( &(%errExit%)

for %%I in ("%PROJECT_FOLDER%\%KEIL_PROJECT_NAME_UVPROJX%") do (
  echo UV4.exe -c -j0 %%I -o "%BUILD_LOG%"
  echo(
  UV4.exe -c -j0 %%I -o "%BUILD_LOG%"
  %ifErr% IF EXIST "%PROJECT_FOLDER%\%BUILD_LOG%" type "%PROJECT_FOLDER%\%BUILD_LOG%" &(%errExit%)
  IF EXIST "%PROJECT_FOLDER%\%KEIL_PROJECT_OUTPUT_DIR%%BUILD_LOG%" type "%PROJECT_FOLDER%\%KEIL_PROJECT_OUTPUT_DIR%%BUILD_LOG%"
)
