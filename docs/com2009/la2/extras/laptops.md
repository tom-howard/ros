---  
title: Working with your ROS Packages on the laptops
---  

You'll need to transfer your ROS package(s) to a robot laptop whenever you want to work on a real robot in the real robot arena during the labs. 

Having your ROS package on GitHub makes it much easier to transfer between simulation (WSL-ROS) and the real robots. Using SSH keys, you can clone your team's ROS package to the robot laptops, make commits and push these back up to GitHub without needing to provide your GitHub username and a personal access token every time. This makes life a lot easier! The following steps describe the process you should follow (adapted from [GitHub Docs](https://docs.github.com/en/authentication/connecting-to-github-with-ssh)).

!!! warning "WiFi"
    You'll need to connect the laptop to "eduroam" when doing this!

## Check if you already have an SSH Key on the Laptop

*These instructions are adapted from [this GitHub Docs page](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/checking-for-existing-ssh-keys?platform=linux).*

If you're generating an SSH key for the first time, then you can skip this step and go straight to the next section: "[Generating an SSH key (on the Laptop)](#ssh-keygen)". If you've already generated an SSH Key on the laptop previously then check it's still there before you go any further by following these steps...

1. Open a terminal on the laptop.

    ??? tip "Pro-Tip"
        You can use the ++ctrl+alt+t++ keyboard combination to open a terminal!

1. Use the `ls` command as follows, to see if your SSH key already exists on the laptop:

    ```bash
    ls -al ~/.ssh
    ```

    This will provide you with a list of all the SSH keys on the laptop. Your team's key should have the same name as your ROS package ([if you followed the steps correctly when you created the key previously](#ssh-key-name)), and so you should see your key in the list, i.e.: `com2009_team{}.pub`.

1. If your key is there, then you're good to go... you may need to [Clone your ROS package onto the Laptop again](#ssh-clone) if you deleted it the last time you were working on the laptop.

1. If you can't see your key in the list, then you'll need to follow all the steps on this page, starting with [Generating an SSH key (on the Laptop)](#ssh-keygen).

## Generating an SSH key (on the Laptop) {#ssh-keygen}

1. From a terminal instance on the laptop navigate to the `~/.ssh` folder:

    ```bash
    cd ~/.ssh
    ```

1. Create a new SSH key on the laptop, using your GitHub email address:

    ```bash
    ssh-keygen -t ed25519 -C "your.email@sheffield.ac.uk"
    ```

    Replacing `your.email@sheffield.ac.uk` with **your GitHub email address**.

    <a name="ssh-key-name"></a>

1. You'll then be asked to **"Enter a file in which to save the key"**. This needs to be unique, so enter the name of your ROS package. For the purposes of this example, let's assume yours is called `com2009_team999`.

1. You'll then be asked to **enter a passphrase**. This is how you make your SSH key secure, so that no other teams using the same laptop can access and make changes to your team's package/GitHub repo. You'll be asked to enter this whenever you try to commit/push new changes to your ROS package on GitHub. Decide on a passphrase and share this with your team **ONLY**. 

1. Next, start the laptop's ssh-agent:

    ```bash
    eval "$(ssh-agent -s)"
    ```

1. Add your SSH private key to the laptop's ssh-agent. You'll need to enter the name of the SSH key file that you created in the earlier step (e.g.: `com2009_team999`)

    ```bash
    ssh-add ~/.ssh/com2009_team999
    ```

    Replacing `com2009_team999` with the name of your own SSH key file, of course!

1. Then, you'll need to add the SSH key to your account on GitHub...

## Adding an SSH key to your GitHub account

*These instructions are replicated from [this GitHub Docs page](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account?platform=linux)*.

1. On the laptop, copy the SSH public key that you created in the previous steps to your clipboard.
    
    Do this from a terminal on the laptop, using `cat`:

    ```bash
    cat ~/.ssh/com2009_team999.pub
    ```

    replacing `com2009_team999` with the name of your SSH key file.

    The content of the file will then be displayed in the terminal... copy it from here.

    !!! note
        To copy text from inside a terminal window use ++ctrl+shift+c++

    ??? tip 
        You could also open the file in VS Code and copy it from there:

        ```bash
        code ~/.ssh/com2009_team999.pub
        ```

1. Go to your GitHub account in a web browser. In the upper-right corner of any page, click your profile photo, then click **Settings**.

1. In the "Access" section of the sidebar, click **SSH and GPG keys**.

1. Click **New SSH key**.

1. Enter a descriptive name for the key in the "Title" field, e.g. `com2009_dia-laptop1`.

1. Select `Authentication Key` as the "Key Type."

1. Paste the text from your SSH Public Key file into the "Key" field.

1. Finally, click the "Add SSH Key" button.

## Cloning your ROS package onto the Laptop {#ssh-clone}

With your SSH keys all set up, you can now clone your ROS package onto the laptop. 

There's a Catkin Workspace on each of the robot laptops and (much the same as in the WSL-ROS environment) your package **must** reside within this workspace!

1. From a terminal on the laptop, navigate to the Catkin Workspace `src` directory:

    ```bash
    cd ~/catkin_ws/src
    ```

1. Go to your ROS package on GitHub. Click the Code button and then select the SSH option to reveal the SSH address of your repo. Copy this. 

1. Head back to the terminal instance on the laptop to then clone your package into the `catkin_ws/src/` directory using `git`:

    ```bash
    git clone {REMOTE_SSH_ADDRESS}
    ```

    Where `{REMOTE_SSH_ADDRESS}` is the SSH address that you have just copied from GitHub.

1. Run Catkin Build to make sure that any resources within your package that need to be compiled (custom ROS messages, etc.) are compiled onto the laptop so that they can be used locally:
	
    ```bash
    catkin build com2009_team999
    ```
	
	...again, replacing `com2009_team999` with *your* team's package name.
	
1. Then, re-source your environment:
	
    ```bash
    source ~/.bashrc
    ```

You should then be able to commit and push any updates that you make to your ROS package while working on the laptop, back to your remote repository using the secret passphrase that you defined earlier!

## Deleting your ROS package after a lab session

Remember that the laptops use an account that everyone in the class has access to. You might therefore want to delete your package from the laptop at the end of each lab session. It's very easy to clone it back onto the laptop again by following [the steps above](#ssh-clone) at the start of each lab session. Deleting your package (by following the instructions below) **won't** delete your SSH key from the laptop though, so you won't need to do all that again, and your SSH key will still be protected with the secret passphrase that you set up when generating the SSH Key to begin with (assuming that you are working on the same laptop, of course!) 

Delete your package by simply running the following command from any terminal on the laptop:

!!! warning
    Make sure you've pushed any changes to GitHub before deleting your package!

```bash
rm -rf ~/catkin_ws/src/com2009_team{}
```

... replacing `{}` with your own team's number!

