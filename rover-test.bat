@echo off
rem Rover One Reusable Physical Test Framework CLI Wrapper (Batch)
set PYTHONPATH=%~dp0;%PYTHONPATH%
python -m tools.rover_tests.cli %*
