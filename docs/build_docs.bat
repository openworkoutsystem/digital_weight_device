@echo off
echo ========================================
echo Open Workout System Documentation Build
echo ========================================

echo Installing/updating requirements...
pip install -r requirements.txt

echo.
echo Building documentation...
python -m sphinx.cmd.build -b html -W . build

echo.
if %ERRORLEVEL% EQU 0 (
    echo ========================================
    echo Build successful!
    echo Documentation available at: build/index.html
    echo ========================================
    echo.
    echo Opening documentation in browser...
    start build/index.html
) else (
    echo ========================================
    echo Build failed! Check the errors above.
    echo ========================================
)

echo.
pause