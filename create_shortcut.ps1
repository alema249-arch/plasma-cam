# PlasmaCam デスクトップショートカット作成スクリプト
# 職場・自宅どちらのPCでもこのファイルを右クリック→「PowerShellで実行」するだけでOK

$appDir   = $PSScriptRoot
# onedir形式(dist\PlasmaCam\PlasmaCam.exe)を優先。
# 古いonefile形式(dist\PlasmaCam.exe)が残っていればそちらにフォールバック。
$exePathDir  = "$appDir\dist\PlasmaCam\PlasmaCam.exe"
$exePathFile = "$appDir\dist\PlasmaCam.exe"
$iconPath = "$appDir\plasma_cam.ico"

$lnkPath = "$([Environment]::GetFolderPath('Desktop'))\PlasmaCam.lnk"
$WshShell = New-Object -ComObject WScript.Shell
$sc = $WshShell.CreateShortcut($lnkPath)

if (Test-Path $exePathDir) {
    $sc.TargetPath = $exePathDir
} elseif (Test-Path $exePathFile) {
    $sc.TargetPath = $exePathFile
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
