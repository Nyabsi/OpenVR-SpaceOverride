;--------------------------------
; Include Modern UI

!include "MUI2.nsh"

;--------------------------------
; General Configuration

!define APP_VERSION "2.4.0"
!define APP_VERSION_META "2.4.0.0"
!define APP_NAME "OpenVR-SpaceOverride"

!define INSTALL_DIR "$PROGRAMFILES64\${APP_NAME}"
!define LICENSE_FILE "../bin/LICENSE.txt"
!define FILES_DIR "../bin/"
!define DRIVER_DIR "../driver"

Name "${APP_NAME}"
OutFile "${APP_NAME}_Installer.exe"
InstallDir "${INSTALL_DIR}"
InstallDirRegKey HKLM "Software\${APP_NAME}\Main" ""
RequestExecutionLevel admin
ShowInstDetails show

VIProductVersion "${APP_VERSION_META}"
VIAddVersionKey /LANG=1033 "ProductName" "${APP_NAME}"
VIAddVersionKey /LANG=1033 "FileDescription" "${APP_NAME} Installer"
VIAddVersionKey /LANG=1033 "LegalCopyright" "Copyright (c) 2026 Nyabsi"
VIAddVersionKey /LANG=1033 "FileVersion" "${APP_VERSION_META}"
VIAddVersionKey /LANG=1033 "ProductVersion" "${APP_VERSION}"

;--------------------------------
; Variables

Var alreadyInstalled

;--------------------------------
; Interface Settings

!define MUI_ABORTWARNING

;--------------------------------
; Pages

!insertmacro MUI_PAGE_LICENSE "${LICENSE_FILE}"
!define MUI_PAGE_CUSTOMFUNCTION_PRE dirPre
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES

!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES

;--------------------------------
; Language

!insertmacro MUI_LANGUAGE "English"

;--------------------------------
; Functions

Function dirPre
    StrCmp $alreadyInstalled "true" 0 +2
        Abort
FunctionEnd

Function .onInit
    StrCpy $alreadyInstalled "false"

    ReadRegStr $R0 HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}" "UninstallString"
    StrCmp $R0 "" done

    MessageBox MB_YESNOCANCEL|MB_ICONQUESTION \
        "${APP_NAME} is already installed.$\n$\nClick YES to Reinstall$\nClick NO to Remove$\nClick CANCEL to abort installation" \
        IDYES repair \
        IDNO remove
    Abort

    repair:
        StrCpy $alreadyInstalled "true"
        Goto done

    remove:
        ExecWait '"$INSTDIR\Uninstall.exe" /S _?=$INSTDIR'
        MessageBox MB_OK "${APP_NAME} has been uninstalled."
        Delete "$INSTDIR\Uninstall.exe"
        RMDir "$INSTDIR"
        Quit

    done:
FunctionEnd


;--------------------------------
; Installer Section

Section "Install" SecInstall

    StrCmp $alreadyInstalled "true" 0 noupgrade
        DetailPrint "Cleaning previous installation..."
        ExecWait '"$INSTDIR\Uninstall.exe" /S _?=$INSTDIR'
        Delete "$INSTDIR\Uninstall.exe"
    noupgrade:

    SetOutPath "$INSTDIR"

    File "${FILES_DIR}\LICENSE.txt"
	File "${FILES_DIR}\LICENSE"
	File "${FILES_DIR}\LICENSES"
	File "${FILES_DIR}\manifest.vrmanifest"
    File "${FILES_DIR}\OpenVR-SpaceOverride.exe"
    File "${FILES_DIR}\openvr_api.dll"
    File "${FILES_DIR}\icon.png"

    SetOutPath "$INSTDIR\driver"
    File /r "${DRIVER_DIR}\*"

    WriteRegStr HKLM "Software\${APP_NAME}\Main" "" $INSTDIR
    WriteUninstaller "$INSTDIR\Uninstall.exe"

    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}" "DisplayName" "${APP_NAME}"
    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}" "UninstallString" "$\"$INSTDIR\Uninstall.exe$\""

    CreateShortCut "$SMPROGRAMS\${APP_NAME}.lnk" "$INSTDIR\OpenVR-SpaceOverride.exe"

    Var /GLOBAL vrRuntimePath
	nsExec::ExecToStack '"$INSTDIR\OpenVR-SpaceOverride.exe" -openvrpath'
	Pop $0
	Pop $vrRuntimePath
	DetailPrint "VR runtime path: $vrRuntimePath"

    ExecWait '"$vrRuntimePath\bin\win64\vrpathreg.exe" adddriver "$INSTDIR\driver"'

	SetOutPath "$INSTDIR"
	nsExec::ExecToLog '"$INSTDIR\OpenVR-SpaceOverride.exe" -installmanifest'
	nsExec::ExecToLog '"$INSTDIR\OpenVR-SpaceOverride.exe" -activatemultipledrivers'

SectionEnd

;--------------------------------
; Uninstaller Section

Section "Uninstall"

	SetOutPath "$INSTDIR"
	nsExec::ExecToLog '"$INSTDIR\OpenVR-SpaceOverride.exe" -removemanifest'

    Delete "$INSTDIR\LICENSE.txt"
	Delete "$INSTDIR\LICENSE"
	Delete "$INSTDIR\LICENSES"
	Delete "$INSTDIR\manifest.vrmanifest"
    Delete "$INSTDIR\OpenVR-SpaceOverride.exe"
    Delete "$INSTDIR\openvr_api.dll"
    Delete "$INSTDIR\icon.png"
    RMDir /r "$INSTDIR\driver"

    DeleteRegKey HKLM "Software\${APP_NAME}"
    DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}"
    Delete "$SMPROGRAMS\${APP_NAME}.lnk"

    RMDir "$INSTDIR"

    Var /GLOBAL vrRuntimePath2
	nsExec::ExecToStack '"$INSTDIR\OpenVR-SpaceOverride.exe" -openvrpath'
	Pop $0
	Pop $vrRuntimePath2
	DetailPrint "VR runtime path: $vrRuntimePath"

    ExecWait '"$vrRuntimePath2\bin\win64\vrpathreg.exe" removedriver "$INSTDIR\driver"'

SectionEnd
