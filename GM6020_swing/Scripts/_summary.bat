@echo off

echo( >> "%TMP_FOLDER%\program_size_summary.log"
set /A optim_level=%KEIL_PROJECT_OPTIM_LEVEL%-1
echo Build summary for %PROJECT_TARGET% (%date% %time:~0,5%)
echo Build summary for %PROJECT_TARGET% (%date% %time:~0,5%) >> "%TMP_FOLDER%\program_size_summary.log"
echo Keil's compiler optimization: "Level %optim_level% (-O%optim_level%)"
echo Keil's compiler optimization: "Level %optim_level% (-O%optim_level%)" >> "%TMP_FOLDER%\program_size_summary.log"

SET PATH_TO_MAP=""
IF NOT "%KEIL_PROJECT_LISTING_DIR%"=="" (
  SET PATH_TO_MAP="%PROJECT_FOLDER%\%KEIL_PROJECT_LISTING_DIR%*.map"
) ELSE IF NOT "%KEIL_PROJECT_OUTPUT_DIR%"=="" (
  SET PATH_TO_MAP="%PROJECT_FOLDER%\%KEIL_PROJECT_OUTPUT_DIR%*.map"
)

IF NOT %PATH_TO_MAP%=="" (
  findstr /C:"Grand Totals" %PATH_TO_MAP%
  findstr /C:"Grand Totals" %PATH_TO_MAP% >> "%TMP_FOLDER%\program_size_summary.log"
  findstr /C:"ROM Totals" %PATH_TO_MAP%
  findstr /C:"ROM Totals" %PATH_TO_MAP% >> "%TMP_FOLDER%\program_size_summary.log"
  findstr /C:"Total ROM Size" %PATH_TO_MAP%
  findstr /C:"Total ROM Size" %PATH_TO_MAP% >> "%TMP_FOLDER%\program_size_summary.log"
  findstr /C:"Load Region LR_IROM1" %PATH_TO_MAP% >> "%TMP_FOLDER%\program_size_summary.log"

  python ".\Scripts\_program_size_summary_cf.py" "%TMP_FOLDER%\program_size_summary.log"

  echo(
  findstr /C:"Build time" "%TMP_FOLDER%\program_size_summary_cf.log"
  findstr /C:"%PROJECT_TARGET%" "%TMP_FOLDER%\program_size_summary_cf.log"
) ELSE (
  echo(
  echo Couldn't generate the summary. Path to .map file is invalid in Keil's project.
  echo Couldn't generate the summary. Path to .map file is invalid in Keil's project. >> "%TMP_FOLDER%\program_size_summary.log"
  echo Click on "Select Folder for Listings..." button in Listing tab in Keil's project options.
  echo Click on "Select Folder for Listings..." button in Listing tab in Keil's project options. >> "%TMP_FOLDER%\program_size_summary.log"
)