+++  
title = "Getting Started with WSL-ROS on a University Computer"  
menuTitle = "Getting Started"  
weight = 1  
+++

{{% textalign left %}}
[<i class="fas fa-solid fa-arrow-left"></i> Previous Page: "The WSl-ROS Simulation Environment"](/wsl-ros/)
{{% /textalign %}}

## How it Works

WSL-ROS is installed on all the University of Sheffield Managed Desktop Computers located in **Computer Rooms 1, 2, 3 & 4 in the Diamond**. It's also possible to access it on *some* remote-access machines (see ["Accessing the Environment Remotely"](#rdp) for more details).

Each time you launch WSL-ROS on an applicable University Machine the WSL-ROS environment will be installed from a custom OS image that we have created, which contains the Ubuntu 20.04 operating system, ROS and all the additional ROS packages that you will need for this lab course. The environment that you install will only remain on the machine that you install it on for a limited time, and won't be preserved across different computers that you log into. It's therefore really important that you [follow some backup and restore procedures as detailed below](#backup-restore).

### Launching WSL-ROS for the First Time {#first-run}

Click the Windows Start Menu button: ![](/images/wsl/win_start_button.png)
    
Then, start typing `"wsl-ros"` and click on the application shortcut that should then appear in the list:

![](/images/wsl/win_menu.png)

You will be presented with the following screen:

![](/images/wsl/first_install.png?width=700)

WSL-ROS is now being installed from the custom OS image, which may take a couple of minutes to complete.  Once it's done, the *Windows Terminal* should automatically launch:

![](/images/wsl/welcome.png)

This is an *Ubuntu Terminal Instance*, giving us access to the Ubuntu 20.04 operating system that we have just installed. This is the **WSL-ROS Environment**!

{{% textalign center %}}
***You're all set up and ready to go!***
{{% /textalign %}}

## Configuring Visual Studio Code {#vscode}

*Visual Studio Code* (or *'VS Code'*, for short) should be installed on all the University of Sheffield Managed Desktop Computers in the Diamond Computer Rooms. This is a great *Integrated Development Environment (IDE)* that we'll use extensively alongside WSL-ROS. We first need to make sure it's set up correctly though, so follow the steps below **now** to install the *"Remote - WSL"* VS Code extension, in preparation for later.

{{% nicenote tip %}}
You should only ever need to do this bit once: the configurations should be saved to your user profile, and should be carried over to any other University Desktop Computer that you log into!
{{% /nicenote %}}

1. Click the Windows Start Menu button: ![](/images/wsl/win_start_button.png)

1. Type `"vscode"` and the Visual Studio Code application shortcut should then appear in the list:

    ![](/images/vscode/win_menu.png)

    Click on it to launch the application.

1. In the left-hand toolbar click the "Extensions" icon (or use the `Ctrl+Shift+X` keyboard shortcut):

    ![](/images/vscode/extensions_icon.png?width=600)

1. In the search bar (where is says "Search Extensions in Marketplace") type `"remote"`, find the "Remote - WSL" extension in the list and then click the blue "Install" button next to it:

    ![](/images/vscode/install_remote_wsl.png?width=600)

    Once installed, you should see a page similar to the one below:

    ![](/images/vscode/remote_wsl_installed.png?width=600)

    On this page, it should state that `"This extension is enabled globally"` (as shown in red), and you should also see a green icon with two arrows in the bottom left-hand corner of the application window.

1. You can close down VS Code now, we'll launch it again when we need it.

## A Quick Introduction to the Linux Terminal {#linux-term}

As explained earlier, you'll be working extensively with the Linux Terminal throughout this lab course. An *idle* WSL-ROS terminal instance will look like this:

![](/images/wsl/linux_terminal.png)

Here, the presence of the `$` symbol indicates that the terminal is ready to accept a command. Text before the `$` symbol has two parts separated by the `:` symbol:
* Text to the **left** of the `:` tells us the name of the Linux user ("student" in this case) followed by the WSL-ROS version that you are working with.
* Text to the **right** of the `:` tells us where in the Linux Filesystem we are currently located (`~` means *"The Home Directory"*, which is an alias for the path: `/home/student/`).

If you don't see the `$` symbol at all, then this means that a process is currently running. To stop any running process enter `Ctrl+C` simultaneously on your keyboard.

## The WSL-ROS Environment (Part 2): Other Things to be Aware of... {#wsl-ros-part-2}

### Returning or Re-Launching the Environment {#wsl-relaunch}

As discussed above, [the WSL-ROS Environment that you created earlier](#first-run) will only be preserved for a limited time on the machine that you installed it on!

{{% nicenote warning %}}
Any work that you do within WSL-ROS **will not be preserved** between sessions or across different University machines automatically!
{{% /nicenote %}}

At the beginning of each practical session (or any other time you want to work in WSL-ROS) you'll need to re-install the environment from the OS image. You **must** therefore make sure that you [back up your work to your University U: Drive](##backup-restore) every time you finish working in the environment, so that you can then restore it the next time you return (we'll remind you about this at the start and end of every practical session, just in case you forget). 

The WSL-ROS Environment *will*, however, be preserved for a limited time if you happen to log in to the *same* University machine within a few hours. If this is the case, then on launching WSL-ROS you will be presented with the following message:

![](/images/wsl/resume.png?width=700)

Enter `Y` to continue where you left things previously, or `N` to start from a fresh installation.

{{% nicenote warning %}}
If you select `N` then any work that you have created in the existing environment will be deleted! Always make sure you back up your work using the procedure outlined below!
{{% /nicenote %}}

### Backing-Up (and Restoring) your Data {#backup-restore}

Once again, every time you do any work in the WSL-ROS Environment it's *really important* that you run a backup script before you close it down and log out of the University Machine that you are working on.  To do so is very easy, simply run the command below from any WSL-ROS Terminal Instance:

```bash
wsl_ros backup
```

This will create an archive of your [Linux Home Directory](#home_dir) and save it to your University U: Drive. Whenever you launch a *fresh* WSL-ROS Environment again on another day, or on a different machine, simply run the following command to restore your work to it:

```bash
wsl_ros restore
```

{{% nicenote info Remember %}}
Your work *will not be preserved* within the WSL-ROS Environment between sessions *unless* you happen to log back in to the *same* computer again within a few hours!
{{% /nicenote %}}

To make things a little easier, on launching WSL-ROS, the system will check to see if a backup file already exists from a previous session. If it does, then the system will ask you if you want to restore it straight away:

![](/images/wsl/restore_prompt.png)

Enter `Y` to restore your data from this backup file, or `N` to leave the backup file alone and work from fresh (none of your previous work will be restored). 

### Accessing WSL-ROS Remotely {#rdp}

WSL-ROS is also available remotely on some University machines via the [University Remote Desktop Service](https://students.sheffield.ac.uk/it-services/computer-facilities/university-computers).

{{% nicenote note %}}
You'll need to use your university account credentials to access this service and will also need to [have multifactor authentication (MFA) set up on your university account](https://sites.google.com/sheffield.ac.uk/mfa/setting-up-mfa).
{{% /nicenote %}}

1. The environment has been installed on all the machines in [Virtual Classroom 1](https://www.sheffield.ac.uk/findapc/rdp/room/49/pcs) (University of Sheffield sign-in required).
1.  From here, you should see a long list of PCs. Any that have a blue "Connect" button next to them are available to use.
1.  Click on the "Connect" button next to an available machine.  This will download a `.rdp` file to your computer.
1.  Once downloaded, double-click on the `.rdp` file and log in to the PC using your university account credentials. (*Look out for an MFA notification on your MFA-linked device.*)
1. Once you're logged into the remote machine follow the steps in the [Launching WSL Section](#first-run) to launch the environment. 
1. When you're finished, close the RDP connection and then close the Remote Desktop app on your machine.

{{% nicenote tip %}}
Only the computers in **Virtual Classroom 1** have the WSL-ROS environment installed, so make sure you only use this room if you want to work on this remotely.
{{% /nicenote %}}

#### Remote Access Support {#its}

{{% nicenote note %}}
We (the COM2009 Teaching Team) are **not** able to deal with issues relating to the University Remote Desktop Service!
{{% /nicenote %}}

If you are facing any issues then you should have a look at [this IT Services support page](https://shef.topdesk.net/tas/public/ssp/content/detail/knowledgeitem?unid=582bb43de02043e4b8bf0ed1b7edcf18&origin=searchResults&decorate=false&_gl=1*es9ipd*_ga*MTIxNzAwMDExNi4xNjMzNjgyMzgx*_ga_TK1DPBM232*MTY2NTQxMDE2NS40ODYuMS4xNjY1NDEwNTA1LjQuMC4w). You can also [contact the IT Services Helpdesk](https://www.sheffield.ac.uk/it-services/support/help) for help and support (Monday-Friday between 8am-5pm).

If you're having issues related to WSL-ROS however, then please let us know.

### Windows Terminal Settings {#wt-settings}

We use [the Windows Terminal](https://devblogs.microsoft.com/commandline/introducing-windows-terminal/) alongside WSL-ROS, to interact with our Ubuntu and ROS environment, and we've created a custom settings file to configure this appropriately. If you've used the Windows Terminal before on a University Machine, or if you have used the WSL-ROS environment previously and have modified the settings file that we have provided, then you'll be presented with the following prompt:

![](/images/wsl/wt_overwrite.png?width=700)

Enter `Y` to use the settings file that we have provided and overwrite whatever was there previously (recommended). Alternatively, enter `N` to preserve your own settings, but note that your experience will then differ to that presented throughout this Wiki, and some instructions may no longer work in the same way.

{{% textalign right %}}
[Next Page: "Launching VS Code in WSL-ROS" <i class="fas fa-solid fa-arrow-right"></i>](/wsl-ros/vscode)
{{% /textalign %}}