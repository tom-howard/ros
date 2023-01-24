# Backing-Up (and Restoring) your Data

Once again, every time you do any work in the WSL-ROS Environment it's *really important* that you run a backup script before you close it down and log out of the University Machine that you are working on.  To do so is very easy, simply run the command below from any WSL-ROS Terminal Instance:

```bash
wsl_ros backup
```

This will create an archive of your [Linux Home Directory](../linux-term) and save it to your University U: Drive. Whenever you launch a *fresh* WSL-ROS Environment again on another day, or on a different machine, simply run the following command to restore your work to it:

```bash
wsl_ros restore
```

!!! info "Remember"
    Your work **will not be preserved** within the WSL-ROS Environment between sessions *unless* you happen to log back in to the *same* computer again within a few hours!

To make things a little easier, on launching WSL-ROS, the system will check to see if a backup file already exists from a previous session. If it does, then the system will ask you if you want to restore it straight away:

<figure markdown>
  ![](../images/wsl/restore_prompt.png)
</figure>

Enter `Y` to restore your data from this backup file, or `N` to leave the backup file alone and work from fresh (none of your previous work will be restored). 