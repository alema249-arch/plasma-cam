# PlasmaCam デスクトップショートカット作成スクリプト
# 職場・自宅どちらのPCでもこのファイルを右クリック→「PowerShellで実行」するだけでOK

$appDir   = "$env:USERPROFILE\OneDrive\projects\plasma-cam"
$iconPath = "$appDir\plasma_cam.ico"

# pythonw.exe の場所を自動検索
$pythonw = (Get-Command pythonw.exe -ErrorAction SilentlyContinue).Source
if (-not $pythonw) {
    $py = (Get-Command python.exe -ErrorAction SilentlyContinue).Source
    if ($py) { $pythonw = $py -replace 'python\.exe', 'pythonw.exe' }
}
if (-not (Test-Path $pythonw)) {
    $pythonw = (Get-Command python.exe -ErrorAction SilentlyContinue).Source
}

$lnkPath = "$env:USERPROFILE\Desktop\PlasmaCam.lnk"
$WshShell = New-Object -ComObject WScript.Shell
$sc = $WshShell.CreateShortcut($lnkPath)
$sc.TargetPath       = $pythonw
$sc.Arguments        = "`"$appDir\main.py`""
$sc.WorkingDirectory = $appDir
$sc.Description      = "Plasma CAM - CNC制御アプリ"
if (Test-Path $iconPath) { $sc.IconLocation = $iconPath }
$sc.Save()

Write-Host "ショートカットを作成しました: $lnkPath" -ForegroundColor Green
Start-Sleep 2
