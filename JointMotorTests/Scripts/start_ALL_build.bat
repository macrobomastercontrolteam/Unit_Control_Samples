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
echo The project for ALL configurations is building. Please wait...
call Scripts\_create_folders.bat
%ifErr% echo( &(%errExit%)
break>"%TMP_FOLDER%\program_size_summary.log"

for /l %%I in (0,1,%PROJECT_TARGET_LIST_LENGTH%) do (
  echo(
  echo === !PROJECT_TARGET_LIST[%%I]! ===
  echo call Scripts\start_build.bat "!PROJECT_TARGET_LIST[%%I]!"
  call Scripts\start_build.bat "!PROJECT_TARGET_LIST[%%I]!"
  %ifErr% echo( &(%errExit%)
)

echo(
echo ALL Done.
call Scripts\_beeps_at_the_end.bat
