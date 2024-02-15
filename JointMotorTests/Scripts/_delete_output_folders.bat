@echo off

for /l %%I in (0,1,%PROJECT_TARGET_LIST_LENGTH%) do (
  echo(
  echo === !PROJECT_TARGET_LIST[%%I]! ===
  echo rmdir /s /q "%PROJECT_FOLDER%\!KEIL_PROJECT_OUTPUT_DIR_LIST[%%I]!"
  rmdir /s /q "%PROJECT_FOLDER%\!KEIL_PROJECT_OUTPUT_DIR_LIST[%%I]!"
  echo rmdir /s /q "%PROJECT_FOLDER%\!KEIL_PROJECT_LISTING_DIR_LIST[%%I]!"
  rmdir /s /q "%PROJECT_FOLDER%\!KEIL_PROJECT_LISTING_DIR_LIST[%%I]!"
)
