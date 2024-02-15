@echo off

call Scripts\_errorlevel_handler.bat

if /i "!PROJECT_TARGET:~0,5!"=="[REL]" (
  if not exist "%PRODUCTION_FOLDER%\%PROJECT_TARGET%" (
    echo mkdir "%PRODUCTION_FOLDER%\%PROJECT_TARGET%"
    mkdir "%PRODUCTION_FOLDER%\%PROJECT_TARGET%"
    %ifErr% echo( &(%errExit%)
  )
)
if not exist "%TMP_FOLDER%\%PROJECT_TARGET%" (
  echo mkdir "%TMP_FOLDER%\%PROJECT_TARGET%"
  mkdir "%TMP_FOLDER%\%PROJECT_TARGET%"
  %ifErr% echo( &(%errExit%)
)
