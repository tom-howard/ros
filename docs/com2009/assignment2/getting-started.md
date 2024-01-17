---  
title: Getting Started in Week 1
---  

Before you get started on Assignment #2 (as detailed in the pages that follow), you should work through the following tasks in your teams during the first lab in Week 1. 

## Set Up Your Team's ROS Package

As discussed on [the Assignment #2 Overview](./README.md#assessment), everything that your team submit for this lab assignment must be contained within a single ROS package. Inside this you will develop all the necessary nodes to make a TurtleBot3 Waffle (real or simulated) complete each of the assignment tasks. Each task will be assessed by the Teaching Team via *launch files* that you must also provide within your package. You'll therefore need to create one launch file per task (we'll talk more about this later).

The first step however is to create your team's ROS Package.

Only one member of your team needs to actually do this bit, and it's best to do this from within your own ROS installation (or WSL-ROS), rather than on the robotics laptop that you will use to work with the real robots in the lab. Select a team member who has access to their own ROS installation in the lab now (i.e. via a personal laptop), or access WSL-ROS using one of the "WSL-ROS laptops" that are also available in the lab.

Follow the steps below to create your team's ROS package, initialise it as a Git repo, and push it to GitHub to allow for sharing and collaboration amongst your team[^github-pro]. You should then be able to transfer your package to one of the robotics laptops easily in the lab when required (we'll provide further details on how to do this later).

[^github-pro]: As a University of Sheffield student, you can apply for the [GitHub Student Developer Pack](https://education.github.com/pack), which gives you access to a range of developer tools including *GitHub Pro*. GitHub Pro allows you to have unlimited collaborators on your repositories, which might help you to collaborate on your ROS package with your team.

### Configure Git {#git}

!!! warning
    Only do this bit on your own personal ROS installations (or in WSL-ROS), **NOT** on one of the robotics laptops!

If you haven't done so already, you'll need to make sure Git is configured properly in your local ROS installation before you do anything else.

1. From a terminal instance located in your home directory (`cd ~`) run the following commands to update your personal details in the global Git config file on your machine:

    ``` { .bash .no-copy }
    git config --global user.name "{your name}"
    ```
    ...replacing `{your name}` with your actual name! E.g.: `#!bash git config --global user.name "John Smith"`
    
    ``` { .bash .no-copy }
    git config --global user.email "{your email address}"
    ```
    ...replacing `{your email address}` with your actual email address!

2. If you're working in WSL-ROS on a University machine, don't forget to run `wsl_ros backup` to save these changes to your external WSL-ROS backup file, so that they will always be restored whenever you run `wsl_ros restore` in a fresh WSL-ROS instance on another machine. 

    !!! note
        **All team members will actually need to do this bit before interacting with Git!**

        Regardless of which team member is setting up your team's ROS package to begin with, you'll **all** need to interact with Git for this assignment, and you should therefore *each* set up your own individual Git configurations (via the steps above) before working individually on your team's ROS package.

### Create Your Team's Assignment #2 ROS package {#create-pkg}

!!! info "Remember"
    Only one team member needs to do this next bit! But you'll then need to share it with the rest of your team members via an online code repository (e.g. GitHub), by making the rest of your team members collaborators.

1. In your local ROS installation, navigate to the `catkin_ws/src` directory:

    ```bash
    cd ~/catkin_ws/src/
    ```
  
1. Use the `catkin_create_pkg` tool to create a new ROS package (as covered in Assignment #1):

    ```bash
    catkin_create_pkg com2009_team{} rospy
    ```

    ...replacing the `{}` with *your* team number!

    !!! warning
        It's really important that you follow the naming conventions that we specify when defining your package (and creating your launch files). If you don't then you could receive **no marks**!

1. Run `catkin build` and then re-source your environment:

    ```bash
    catkin build com2009_team{}
    ```
    ```bash
    source ~/.bashrc
    ```

    ??? tip "WSL-ROS Tip"
        Use the `src` alias for this command!
  
1. Then navigate into the package directory that should have just been created:

    ```bash
    cd com2009_team{}/
    ```

1. This should already contain a `src` folder for you to populate with all your Python ROS nodes. Create a `launch` folder in here too, which you will use to store all your launch files:

    ```bash
    mkdir launch
    ```

1. Create a placeholder file in the `src` and `launch` directories, just to make sure that these folders both get pushed to GitHub when we get to that part in the following section:

    ```bash
    touch src/placeholder && touch launch/placeholder
    ```

    (you can delete these later on, once you start creating your own Nodes and launch files.)

1. Then follow the steps in the next section to import this project to GitHub (other online code repositories should work similarly)...

### Push Your Package to GitHub {#github}

!!! info "Remember"
    Only one member of your team needs to do this bit too!

These instructions are taken from [this GitHub Docs page](https://docs.github.com/en/get-started/importing-your-projects-to-github/importing-source-code-to-github/adding-locally-hosted-code-to-github#adding-a-local-repository-to-github-using-git). Instructions may vary if you are using other online code repositories (such as GitLab for instance), so check with your target provider.

1. [Create a new (**private**) repository on GitHub.com](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-new-repository), but **DON'T** initialise the new repository with a README, license, or gitignore (you can do this later, it'll cause issues if you do it at this stage).

    <figure markdown>
      ![](https://docs.github.com/assets/cb-11427/images/help/repository/repo-create.png)
      <figcaption>From [docs.github.com](https://docs.github.com/en/get-started/importing-your-projects-to-github/importing-source-code-to-github/adding-locally-hosted-code-to-github#adding-a-local-repository-to-github-using-git)</figcaption>
    </figure>

    Call this repository `com2009_team{}` to match the ROS package that you've just created.

1. Head back to your local ROS installation, where you created your ROS package in the previous section. Make sure your terminal is located in the root of your new package directory: 

    ```bash
    roscd com2009_team{}
    ```

1. Initialise the package as a Git repo:
    
    ```bash
    git init -b main
    ```

1. Stage all the initial files in your package (including the placeholders) for an initial commit:

    ```bash
    git add .
    ```

    !!! warning
        Don't forget the `.` at the end there!

1. Then commit them:

    ```bash
    git commit -m "First commit"
    ```

1. Head back to GitHub on the web. At the top of your repository on GitHub.com's Quick Setup page, click the :material-clipboard-outline: button to copy the remote repository URL (HTTPS).

    <figure markdown>
      ![](https://docs.github.com/assets/cb-25662/images/help/repository/copy-remote-repository-url-quick-setup.png)
      <figcaption>From [docs.github.com](https://docs.github.com/en/get-started/importing-your-projects-to-github/importing-source-code-to-github/adding-locally-hosted-code-to-github#adding-a-local-repository-to-github-using-git)</figcaption>
    </figure>

1. Then, go back to your local terminal and add the URL to the remote repository:

    ```bash
    git remote add origin {REMOTE_URL}
    ```
    
    Change `{REMOTE_URL}` to the URL that you copied in the previous step, and **get rid of the curly brackets** (`{}`)!

    Then verify the new remote URL: 
    ```bash
    git remote -v
    ```

1. Finally, push the changes from your package (your "local" repo), to your "remote" repository on GitHub:

    ```bash
    git push origin main
    ```

    You'll then be asked to enter your GitHub username, followed by a password. **This password is not your GitHub account password**!  

    !!! warning
        **Your GitHub account password won't work here!** You'll need to [generate a personal access token (classic)](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token#creating-a-personal-access-token-classic) and use this instead!

1. Back on GitHub, add your team members to the repo as collaborators. All team members should then be able to pull the remote repo into their own Catkin Workspaces (`cd ~/catkin_ws/src/` & `git clone {REMOTE_URL}`), make contributions and push these back to the remote repo as required (using their own GitHub account credentials and personal access tokens).

You'll need to copy your ROS package onto the Robot Laptops when working on the Real-Robot based tasks, which we'll cover in more detail later. 

## Getting to Know the Real Robots

All the Assignment #2 Tasks will be assessed using real robots, and you'll therefore have access to the robots for every lab session so that you can work on these tasks as you wish. All the details on how the robots work, how to get them up and running and start programming them can be found in the "Waffles" section of this course site. You should proceed now as follows (**in your teams**):

1. Everyone **must** complete a health and safety quiz (on Blackboard) before you (or your team) work with the real robots for the first time. Head to Blackboard and do this now, if you haven't already.
1. Each team has been assigned a specific robot (there's a list on Blackboard). When you're ready, speak to a member of the teaching team who will provide you with the robot that has been assigned to you.
1. Work through each page of [the "Waffles" section of this site](../../../waffles) (**in order**):
   
    1. Read about the hardware.
    1. Learn how to launch ROS and get the robots up and running.
    1. Have a go at the initial exercises on [the Basics of ROS & the Waffles](../../../waffles/exercises), which will help to get you started and understand how ROS and the robots work.
    1. Review the [Shutdown Procedures](../../../waffles/shutdown) and follow the steps here to shut down the robot and power off the robotics laptop at the end of each lab session.
    1. Finally, there are some [Fact-Finding Missions](../../../waffles/fact-finding) which are also really important, but you'll need to have completed certain parts of Assignment #1 before you can complete them all. Either way, have a read through them at this stage and work through the bits that you can (if any).
    
        !!! warning "Important"
            Make sure (as a team) you have completed **ALL** the Fact-Finding Missions **before the end of Week 6**.   