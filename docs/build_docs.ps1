Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Open Workout System Documentation Build" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

Write-Host "`nInstalling/updating requirements..." -ForegroundColor Yellow
pip install -r requirements.txt

Write-Host "`nBuilding documentation..." -ForegroundColor Yellow
python -m sphinx.cmd.build -b html -W . build

if ($LASTEXITCODE -eq 0) {
    Write-Host "`n========================================" -ForegroundColor Green
    Write-Host "Build successful!" -ForegroundColor Green
    Write-Host "Documentation available at: build/index.html" -ForegroundColor Green
    Write-Host "========================================" -ForegroundColor Green

    Write-Host "`nOpening documentation in browser..." -ForegroundColor Yellow
    Start-Process "build/index.html"
} else {
    Write-Host "`n========================================" -ForegroundColor Red
    Write-Host "Build failed! Check the errors above." -ForegroundColor Red
    Write-Host "========================================" -ForegroundColor Red
}

Read-Host "`nPress Enter to continue"