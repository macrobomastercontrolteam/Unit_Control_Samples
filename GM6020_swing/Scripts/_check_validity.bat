@echo off

call Scripts\_errorlevel_handler.bat

IF "%KEIL_PROJECT_PATH%" == "" (
  echo KEIL_PROJECT_PATH: "%KEIL_PROJECT_PATH%"
  echo KEIL_PROJECT_PATH cannot be empty. &(%errExit%)
)
IF "%PROJECT_FOLDER%" == "" (
  echo PROJECT_FOLDER: "%PROJECT_FOLDER%"
  echo PROJECT_FOLDER cannot be empty. &(%errExit%)
)
IF "%KEIL_PROJECT_NAME_UVPROJX%" == "" (
  echo KEIL_PROJECT_NAME_UVPROJX: "%KEIL_PROJECT_NAME_UVPROJX%"
  echo KEIL_PROJECT_NAME_UVPROJX cannot be empty. &(%errExit%)
)
IF "%PROJECT_TARGET_LIST[0]%" == "" (
  echo PROJECT_TARGET_LIST[0]: "%PROJECT_TARGET_LIST[0]%"
  echo PROJECT_TARGET_LIST[0] cannot be empty. &(%errExit%)
)
IF "%KEIL_PROJECT_OUTPUT_DIR_LIST[0]%" == "" (
  echo KEIL_PROJECT_OUTPUT_DIR_LIST[0]: "%KEIL_PROJECT_OUTPUT_DIR_LIST[0]%"
  echo KEIL_PROJECT_OUTPUT_DIR_LIST[0] cannot be empty. &(%errExit%)
)
IF "%PROJECT_TARGET_LIST_LENGTH%" == "" (
  echo PROJECT_TARGET_LIST_LENGTH: "%PROJECT_TARGET_LIST_LENGTH%"
  echo PROJECT_TARGET_LIST_LENGTH cannot be empty. &(%errExit%)
)