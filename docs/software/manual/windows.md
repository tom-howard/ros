---
title: Installing ROS on Windows (using WSL)
---

# Installing ROS on Windows (using WSL)

**Applicable to**: Those who have a Windows 10 or 11 personal machine.

## Prerequisites

1. Your computer must be running Windows 10 **Build 19044 or higher**, or Windows 11.
2. [Update the GPU drivers for your machine](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps#install-support-for-linux-gui-apps).
3. Install or update WSL:
    1. If you don't already have WSL installed on your machine then follow [these instructions to install it](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps#fresh-install---no-prior-wsl-installation).
    2. If you *do* already have WSL installed on your machine, then follow [these instructions to update it](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps#existing-wsl-install).
4. [Install the Windows Terminal](https://learn.microsoft.com/en-us/windows/terminal/install).
5. [Install Visual Studio Code](https://code.visualstudio.com/) and [the WSL extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl).

## Install and configure an Ubuntu 20.04 WSL distribution

1. Launch PowerShell and enter the following command to download and install an Ubuntu 20.04 distribution for WSL:
  
    ```powershell
    wsl --install Ubuntu-20.04 --web-download
    ```

    The installation might take a few minutes, but once complete the distribution should automatically be launched.

1. From within the new Ubuntu distribution, update and install a few important tools:
    
    1.  Update existing packages:

        ```bash
        sudo apt update && sudo apt upgrade
        ```
    
    1. Install the latest version of Git:

        ```bash
        sudo add-apt-repository ppa:git-core/ppa
        ```
        ```bash
        sudo apt update && sudo apt install git
        ```
    
    1. Make sure a few additional packages are also installed:

        ```bash
        sudo apt install eog dos2unix curl wget
        ```

## Installing ROS

Having installed and configured your Ubuntu 20.04 instance, continue on to the next page and follow [the steps for Installing ROS (and dependencies)](./linux.md).
