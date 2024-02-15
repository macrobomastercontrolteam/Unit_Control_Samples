@echo off

call Scripts\_errorlevel_handler.bat

where fromelf.exe
%ifErr% echo fromelf.exe was not found. &(%errExit%)

echo(
echo The project for configuration "%PROJECT_TARGET%" is converting. Please wait...
echo PROJECT_FOLDER: "%PROJECT_FOLDER%"
echo PRODUCTION_FOLDER: "%PRODUCTION_FOLDER%"
echo TMP_FOLDER: "%TMP_FOLDER%"
echo PROJECT_TARGET: "%PROJECT_TARGET%"

call Scripts\_check_validity.bat
%ifErr% echo( &(%errExit%)

for %%I in ("%PROJECT_FOLDER%\%KEIL_PROJECT_OUTPUT_DIR%*.axf") do (
  echo fromelf.exe "%%I" --bin --output "%TMP_FOLDER%\%PROJECT_TARGET%\%%~nI.bin"
  fromelf.exe "%%I" --bin --output "%TMP_FOLDER%\%PROJECT_TARGET%\%%~nI.bin"
  %ifErr% echo( &(%errExit%)

  echo(
  call Scripts\_summary.bat
)
