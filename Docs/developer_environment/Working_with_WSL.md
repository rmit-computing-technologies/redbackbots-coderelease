# Working with WSL

This document contains some notes on how to get the project building if you use WSL

## Why do this?

- Keep yourself in Windows if that's where you're most productive
- More performant running a headless VM than an entire VM desktop within Windows

> **⚠ Warning**  Using WSL for this repo is not for the faint hearted. Make sure you're comfortable with your way around Linux, as there won't be much hand holding if you run into any issues.
>
> Also, do make sure you're on **WSL 2** before continuing

## Following the default 'Getting started' guide

For the most part, you can follow the default Getting started guide word for word. You should however run the following beforehand:

```bash
sudo dpkg --add-architecture i386 # This may not be needed anymore with the new build?
```

You may also need to take the following into account depending on your setup:

- The `/etc/hosts` file will be overwritten every time you restart WSL. To fix this, you need to make a config file in `/etc/wsl.conf`. Read the comments in `/etc/hosts` to learn more
- It's recommended you try get 22.04 on WSL as that is the version the build script works with. As of 05/08/2024, it will not run on 20.04 because G++-12 is not available on that version. 
- You may often run into errors such as `could not read ...\r`. This is because Windows uses the line ending character `\r\n` while linux uses `\n`. You'll need to install the utility `dos2unix` and use it to convert the problematic file whenever this happens.
- Please make sure to not sync CRLF files onto the robot. If this is a core script, the script won't run, which could affect core systems such as the daemon script.

## GUI/Graphics

> Note: As of Feb 2023, Windows 10 and 11 have graphics built into WSL. Do `wsl --update` to get the latest version

You will need an X11 server in order to use graphical applications like `offnao` or `vatnao`. You can download it from https://sourceforge.net/projects/vcxsrv/
You can learn about how to setup X11 for WSL via Google. If your X11 server is running correctly, you should be able to launch `offnao` within Windows.

Note that Windows 11 has WSL graphics built in (WSLg).