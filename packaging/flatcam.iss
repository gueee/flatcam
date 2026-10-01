; Inno Setup 6 script for the FlatCAM EVO Windows installer.
;
; It packages the PyInstaller one folder build from dist\FlatCAM.
; Build the application first, then compile this script:
;     pyinstaller packaging\flatcam.spec --noconfirm
;     ISCC packaging\flatcam.iss
; packaging\build_win.ps1 does both.

#define MyAppName "FlatCAM EVO"
#define MyAppExeName "FlatCAM.exe"
#define MyAppPublisher "FlatCAM EVO"
#define MyAppURL "http://flatcam.org"

; The version can be passed in with: ISCC /DMyAppVersion=1.0 packaging\flatcam.iss
#ifndef MyAppVersion
  #define MyAppVersion "Unstable"
#endif

#define SourceDir "..\dist\FlatCAM"

[Setup]
AppId={{8B0B0D9E-2C2F-4D3B-9E5A-1F6D4C7A3B21}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}
DefaultDirName={autopf}\FlatCAM EVO
DefaultGroupName={#MyAppName}
DisableProgramGroupPage=yes
OutputDir=..\dist
OutputBaseFilename=FlatCAM_EVO_Setup
SetupIconFile=..\assets\resources\flatcam_icon256.ico
UninstallDisplayIcon={app}\{#MyAppExeName}
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
; the build is 64 bit only
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Files]
Source: "{#SourceDir}\{#MyAppExeName}"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourceDir}\*"; DestDir: "{app}"; Excludes: "{#MyAppExeName}"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{group}\{cm:UninstallProgram,{#MyAppName}}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#StringChange(MyAppName, '&', '&&')}}"; Flags: nowait postinstall skipifsilent

[UninstallDelete]
; the settings that the application writes next to itself when it runs in portable mode
Type: filesandordirs; Name: "{app}\config"
