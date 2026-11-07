@echo off
setlocal
chcp 65001 >nul

rem Keep this script in the installer folder next to the FitMe folder.
for %%I in ("%~dp0..\FitMe") do set "SOURCE=%%~fI"
set "DESTINATION=%APPDATA%\Autodesk\Autodesk Fusion 360\API\AddIns\FitMe"

if not exist "%SOURCE%\FitMe.manifest" (
    echo ERROR: FitMe folder was not found next to the installer folder.
    echo エラー: installer フォルダーの隣に FitMe フォルダーが見つかりません。
    echo Expected / 想定パス: "%SOURCE%"
    pause
    exit /b 1
)

echo FitMe will be copied to:
echo FitMe のコピー先:
echo "%DESTINATION%"
echo.
choice /C YN /M "Copy FitMe to the Fusion add-ins folder? / Fusion のアドインフォルダーに FitMe をコピーしますか？"
if errorlevel 2 (
    echo Installation cancelled.
    echo インストールを中止しました。
    pause
    exit /b 0
)

robocopy "%SOURCE%" "%DESTINATION%" /E /R:2 /W:1
if errorlevel 8 (
    echo.
    echo ERROR: FitMe installation failed. See the copy errors above.
    echo エラー: FitMe のインストールに失敗しました。上のコピーエラーを確認してください。
    pause
    exit /b 1
)

echo.
echo FitMe installation completed. Restart Fusion if it is running.
echo FitMe のインストールが完了しました。Fusion が起動中の場合は再起動してください。
explorer.exe "%DESTINATION%"
pause
exit /b 0
