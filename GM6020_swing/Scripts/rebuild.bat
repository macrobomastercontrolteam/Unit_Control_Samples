@echo off

call Scripts\_errorlevel_handler.bat

echo(
echo The project for configuration "%PROJECT_TARGET%" is rebuilding. Please wait...
echo PROJECT_FOLDER: "%PROJECT_FOLDER%"
echo KEIL_PROJECT_NAME_UVPROJX: "%KEIL_PROJECT_NAME_UVPROJX%"
echo BUILD_LOG: "%BUILD_LOG%"
echo PROJECT_TARGET: "%PROJECT_TARGET%"
echo KEIL_PROJECT_OUTPUT_DIR: "%KEIL_PROJECT_OUTPUT_DIR%"

call Scripts\_check_validity.bat
%ifErr% echo( &(%errExit%)

for %%I in ("%PROJECT_FOLDER%\%KEIL_PROJECT_NAME_UVPROJX%") do (
  echo UV4.exe -r -j8 %%I -t "%PROJECT_TARGET%" -o "%KEIL_PROJECT_OUTPUT_DIR%%BUILD_LOG%"
  echo(
  UV4.exe -r -j8 %%I -t "%PROJECT_TARGET%" -o "%KEIL_PROJECT_OUTPUT_DIR%%BUILD_LOG%"
  %ifKeilErr% IF EXIST "%PROJECT_FOLDER%\%KEIL_PROJECT_OUTPUT_DIR%%BUILD_LOG%" type "%PROJECT_FOLDER%\%KEIL_PROJECT_OUTPUT_DIR%%BUILD_LOG%" &(%errKeilExit%)
  IF EXIST "%PROJECT_FOLDER%\%KEIL_PROJECT_OUTPUT_DIR%%BUILD_LOG%" type "%PROJECT_FOLDER%\%KEIL_PROJECT_OUTPUT_DIR%%BUILD_LOG%"
)
