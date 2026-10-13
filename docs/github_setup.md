# Putting the repo on GitHub

Two ways to do this. **Option A (website only)** is all you need to start. Option B (git) is optional, for later.

## Option A: GitHub website only (no installs)

### 1. Create the repository

1. Sign in at <https://github.com>, or create an account.
2. Top-right **+** → **New repository**.
   - Name: `lostlink-sar`
   - Description: *Search-and-rescue drone that knows when it's about to lose its way home (PX4, Gazebo, ROS2)*
   - **Private** (until Stage 2 works)
   - Leave "Add a README" **unticked**, because we already have one.
3. **Create repository**.

### 2. Upload the files

1. On the empty repo page, click **"uploading an existing file"**.
2. Open the `px4 simulation` folder in File Explorer, select **everything except `_archive`**, and drag it into the browser window. Folders keep their structure in Chrome and Edge.
   - These: `.gitignore`, `LICENSE`, `README.md`, `docs`, `media`, `results`, `ros2_ws`, `sim`
   - **Not** `_archive` (old files and big logs), and **not** any `__pycache__` folder. The `.gitignore` file only works with git, not with browser uploads, so you have to leave these out yourself.
3. Commit message: `Stages 0-1: baseline, damaged-building world, flight analysis, docs` → **Commit changes**.
4. Open the repo's front page and check the README shows its images.

Browser upload limits: 100 files per upload and 25 MB per file. This repo is well within both.

### 3. Attach the flight logs (Release)

The `.ulg` logs are too big for the repo (27 MB and 65 MB), so they go in a Release, which allows up to 2 GB per file:

1. Repo page → right side **Releases → Create a new release**
2. **Choose a tag** → type `stage1` → *Create new tag*
3. Title: *Stage 1: damaged-building baseline*
4. Description: paste the "All runs" table from `docs/stage1_obstacle_world.md`
5. Drag in `_archive/stage1_flight.ulg` (run 3) and `_archive/stage1_run4_flight.ulg` (run 4) → **Publish release**

### Updating after each work session

- **Changed or new files:** **Add file → Upload files**, drag them in (same name = replaced), and commit.
- **Small text fixes:** open the file on GitHub → pencil icon → edit → **Commit changes**.
- Keep a short list of the files you changed in each session (the lab notebook helps), so nothing is missed.

### The trade-off

The website is simplest, but every update is manual drag and drop, and deleting or renaming folders is clumsy. Once Stage 2 adds ROS2 code (many files that change often), switching to Option B saves time.

## Option B: git from Ubuntu (later)

```bash
cd "/mnt/c/Users/karee/Downloads/projects 2026/custom quadcopter hardware/px4 simulation"
bash sim/scripts/init_repo.sh                     # one-time: git init + first commit
git remote add origin https://github.com/<your-username>/lostlink-sar.git
git pull origin main --allow-unrelated-histories  # only if you already uploaded via the website
git push -u origin main                           # asks for username + a Personal Access Token
```

Create the token at GitHub → Settings → Developer settings → Personal access tokens. After that, each session is just `git add . && git commit -m "..." && git push`.

## Making it public later

Repo page → **Settings → General → Danger Zone → Change visibility → Public**.
