Infinite Recharge
=================

Team 2393 software for 2020.
For overall FRC documentation see https://docs.wpilib.org/en/latest

Uninstall Previous Setup
------------------------
If you have development tools from a previous year installed, we suggest to start over by first uninstalling them.

Open Windows search "Add or remove Programs" and remove

 * GRIP
 * CTRE Phoenix Framework
 * HERO Drivers
 * National Instruments Software (takes some time and then you need to restart the computer)

Delete the following folders:

* C:\Users\Public\Public Documents\FRC
* C:\Users\Public\frc2019
* C:\Users\{Your Name}\\.gradle
* C:\Users\{Your Name}\\.tooling
* C:\Users\{Your Name}\\.wpilib


Install Development Tools
-------------------------

 * Java development environment from
   https://github.com/wpilibsuite/allwpilib/releases:
   Download the latest `WPILibInstaller_Windows64-2020.*.zip`.
   'Extract all', then run.
   * For Visual Studio Code, either 'Download' or select an existing `OfflineVsCodeFiles-1.41.1.zip`.
   * De-select `C++ Compiler`, all the rest should be checked.
   * `Execute Install`

 * Git for Windows from https://git-scm.com/downloads (you can skip this on a Mac)

For CAN bus access, install the CTRE Phoenix framework from
https://github.com/CrossTheRoadElec/Phoenix-Releases/releases/

Computers used as drive stations also need the NI game tools from
https://www.ni.com/en-il/support/downloads/drivers/download.frc-game-tools.html


Get Robot Code
--------------

Open VS Studio.

Suggestions for File, Preferences:
 * Color Theme 'Light'
 * Disable the 'minimap'
 * Disable 'render indent guides'
 * Tabs: Use 2 spaces, no auto-detection of indentation

Invoke menu View, Command Palette, and type "Git: Clone".
Enter this URL: https://github.com/Team2393/FRC2020.
Create & browse to a folder `git` in your home directory.
