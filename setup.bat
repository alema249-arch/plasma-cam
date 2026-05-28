@echo off
echo =========================================
echo  Plasma CAM セットアップ
echo =========================================
echo.

REM Python がインストールされているか確認
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Python が見つかりません。
    echo https://www.python.org/downloads/ からインストールしてください。
    echo インストール時に "Add Python to PATH" にチェックを入れてください。
    pause
    exit /b 1
)

echo Python が見つかりました。ライブラリをインストール中...
echo.
python -m pip install ezdxf matplotlib pyserial shapely
echo.
echo =========================================
echo  インストール完了！
echo  start_app.bat でアプリを起動できます。
echo =========================================
pause
