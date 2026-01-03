@echo off

:: delete plugins
del /f /q /s "bin\plugins\veritas_*.dll" >nul 2>&1

:: Delete exes and configs in bin 
del /f /q bin\*.exe >nul 2>&1
del /f /q bin\*.ini >nul 2>&1

:: remove build folder (if it exists)
if exist build rmdir /q /s build











