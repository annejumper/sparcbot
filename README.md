# SPARCBot Estimation & Control

ROS 2 Jazzy. 

## Git Fork, Clone & Build

### 1. Fork the Repository

1. Log in to your GitHub account.
2. Open the [sparcbot repository](https://github.com/siddharthbhurat4/sparcbot).
3. Click **Fork** → **Create fork**.

This creates your own copy of the repository under your GitHub account.

### 2. Configure Git

Run once on your Raspberry Pi:

```bash
git config --global user.name "Your Name"
git config --global user.email "your_email@example.com"
```

Use an email associated with your GitHub account.

### 3. Clone Your Fork
Replace `YOUR_USERNAME` with your GitHub username:
```
git clone https://github.com/YOUR_USERNAME/sparcbot.git
```

Compile the code and set up auto-sourcing (this might take a couple of minutes):
```bash
source /opt/ros/jazzy/setup.bash
cd ~/sparcbot
colcon build
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/sparcbot/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


### 4. Add the Course Repository

Run once:

```bash
git remote add upstream https://github.com/siddharthbhurat4/sparcbot.git
```

Verify:

```bash
git remote -v
```

* `origin` → your fork
* `upstream` → course repository

### 5. Save Your Work
Run this whenever you want to save your code changes:
```bash
git add .
git commit -m "Describe your changes"
git push origin main
```

### 6. Get Course Updates

Run this to get code updates when new code is released for the class:
```bash
git fetch upstream
git merge upstream/main
git push origin main
```

If Git reports a merge conflict, resolve the conflict before continuing.