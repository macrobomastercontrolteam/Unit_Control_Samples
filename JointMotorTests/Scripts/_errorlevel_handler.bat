@echo off

set "ifErr=set foundErr=1&(if errorlevel 0 if not errorlevel 1 set foundErr=)&if defined foundErr"
set "errExit=echo Last operation failed - errorlevel was non-zero. Aborting.&(exit 1)"

set "ifKeilErr=set foundErr=1&(if errorlevel 0 (set foundErr=))&if errorlevel 1 (set foundErr=)&(if errorlevel 2 (set foundErr=1))&if defined foundErr"
set "errKeilExit=echo Last Keil operation failed - errorlevel was grater that 1. Aborting.&(exit 1)"