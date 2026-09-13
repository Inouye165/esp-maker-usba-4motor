# Rover One Reusable Physical Test Framework CLI Wrapper (PowerShell)
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition
$env:PYTHONPATH = "$ScriptDir;$env:PYTHONPATH"
python -m tools.rover_tests.cli @args
