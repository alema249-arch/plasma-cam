# PlasmaCam デスクトップショートカット作成スクリプト
# 職場・自宅どちらのPCでもこのファイルを右クリック→「PowerShellで実行」するだけでOK

$appDir   = $PSScriptRoot
$exePath  = "$appDir\dist\PlasmaCam.exe"
$iconPath = "$appDir\plasma_cam.ico"

$lnkPath = "$env:USERPROFILE\Desktop\PlasmaCam.lnk"
$WshShell = New-Object -ComObject WScript.Shell
$sc = $WshShell.CreateShortcut($lnkPath)

if (Test-Path $exePath) {
    # ビルド済みEXEがあればそれを直接起動
    $sc.TargetPath = $exePath
} else {
    # EXEがなければ python main.py にフォールバック
    $pythonw = (Get-Command pythonw.exe -ErrorAction SilentlyContinue).Source
    if (-not $pythonw) {
        $py = (Get-Command python.exe -ErrorAction SilentlyContinue).Source
        if ($py) { $pythonw = $py -replace 'python\.exe', 'pythonw.exe' }
    }
    if (-not (Test-Path $pythonw)) {
        $pythonw = (Get-Command python.exe -ErrorAction SilentlyContinue).Source
    }
    $sc.TargetPath = $pythonw
    $sc.Arguments  = "`"$appDir\main.py`""
}

$sc.WorkingDirectory = $appDir
$sc.Description      = "Plasma CAM - CNC制御アプリ"
if (Test-Path $iconPath) { $sc.IconLocation = $iconPath }
$sc.Save()

Write-Host "ショートカットを作成しました: $lnkPath" -ForegroundColor Green
Start-Sleep 2
