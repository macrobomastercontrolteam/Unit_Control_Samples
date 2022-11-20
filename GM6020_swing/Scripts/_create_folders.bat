@echo off

call Scripts\_errorlevel_handler.bat

if not exist "%PRODUCTION_FOLDER%" (
  echo mkdir "%PRODUCTION_FOLDER%"
  mkdir "%PRODUCTION_FOLDER%"
  %ifErr% echo( &(%errExit%)
)
if not exist "%TMP_FOLDER%" (
  echo mkdir "%TMP_FOLDER%"
  mkdir "%TMP_FOLDER%"
  %ifErr% echo( &(%errExit%)
)
