# Step 6: play a 3-second 440Hz tone through the camera's speaker
# IMPORTANT: first set Windows output device to the camera's USB speaker:
#   Settings > System > Sound > Output device -> "Speaker (USB Audio ...)"
param([int]$Seconds = 3)

. "$PSScriptRoot\_common.ps1"
Assert-Ffmpeg
$outDir = New-OutputDir
$file = Join-Path $outDir "speaker_test.wav"

Write-Host ("Generating {0}s 440Hz tone ..." -f $Seconds) -ForegroundColor Cyan
& ffmpeg -hide_banner -f lavfi -i "sine=frequency=440:duration=$Seconds" -y $file 2>&1 | Out-Null

Write-Host "Playing through the DEFAULT output device." -ForegroundColor Green
Write-Host "  Make sure Windows output = the camera's USB speaker, volume up:"
Write-Host "  Settings > System > Sound > Output device -> 'Speaker (USB Audio ...)'"
(New-Object Media.SoundPlayer -ArgumentList $file).PlaySync()

Write-Host "=== Judgement ===" -ForegroundColor Yellow
Write-Host "  PASS = you heard 3 beeps from the camera module speaker"
Write-Host "  If played from laptop speakers instead, the USB speaker is not set as default"
