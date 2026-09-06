$gradleExe = "D:\SLAM Bot\android\gradle-bin\gradle-8.4\bin\gradle.bat"

$env:JAVA_HOME = "C:\Program Files\Microsoft\jdk-21.0.11.10-hotspot"
$env:ANDROID_HOME = "C:\Users\saten\AppData\Local\Android\Sdk"
$env:PATH = "$env:JAVA_HOME\bin;$env:PATH"

Write-Host "Running Gradle assembleDebug with JAVA_HOME=$env:JAVA_HOME..."
Set-Location "D:\SLAM Bot\android"
& $gradleExe assembleDebug
