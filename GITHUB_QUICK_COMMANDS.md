# Quick Copy-Paste Commands for GitHub Upload

## ⚡ FASTEST METHOD - Copy All Commands at Once

Open Git Bash and paste these commands one section at a time:

---

### 1️⃣ Navigate to Your Project
```bash
cd C:/Users/nahoy/Mujoco_local_structured/code2
```

---

### 2️⃣ Check Git Status
```bash
git status
```

**If you see "fatal: not a git repository"**, run:
```bash
git init
git branch -M main
```

---

### 3️⃣ Add Your GitHub Repository

**IMPORTANT:** Replace `YOUR_USERNAME` and `YOUR_REPO` with your actual GitHub username and repository name!

```bash
# Check if remote exists
git remote -v

# If empty, add remote (REPLACE WITH YOUR URL!)
git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPO.git

# If remote exists but wrong, update it (REPLACE WITH YOUR URL!)
git remote set-url origin https://github.com/YOUR_USERNAME/YOUR_REPO.git

# Verify
git remote -v
```

---

### 4️⃣ Exclude Large Folders (IMPORTANT!)

```bash
# Remove large folders from git tracking (keeps them locally)
git rm -r --cached robocasa 2>/dev/null
git rm -r --cached robosuite 2>/dev/null
git rm -r --cached c_generated_code 2>/dev/null
git rm --cached acados_ocp.json 2>/dev/null
git rm --cached acados_ocp_robot.json 2>/dev/null
git rm --cached robot_mpc.json 2>/dev/null
```

---

### 5️⃣ Stage All Files
```bash
# Add all files (respects .gitignore)
git add .

# Check what will be committed
git status
```

**VERIFY:** Make sure robocasa/ and robosuite/ do NOT appear in the list!

---

### 6️⃣ Create Commit
```bash
git commit -m "Initial commit: MuJoCo dual-robot system

- Add dual-robot navigation and pick-and-place system
- Add Ubuntu 22.04 installation guides and automated installer
- Add ACADOS MPC path planning controllers
- Add RoboCasa integration support
- Add LLM-based natural language control
- Add comprehensive documentation
- Exclude large dependencies (robocasa, robosuite) from git
- Users should install dependencies via install_ubuntu_22.04.sh"
```

---

### 7️⃣ Push to GitHub
```bash
# Push to GitHub
git push -u origin main
```

**If error "Updates were rejected":**
```bash
git pull origin main --allow-unrelated-histories
git push origin main
```

**If error about 'main' not found:**
```bash
git push -u origin master
```

---

### 8️⃣ Verify Upload
```bash
# Check commit history
git log --oneline -5

# Check remote
git remote -v

# Check branch
git branch -a
```

Then visit your GitHub repository in a web browser!

---

## 🌿 WORKING WITH BRANCHES

### Check Current Branch

```bash
# Show current branch
git branch

# Show all branches (local and remote)
git branch -a

# Show current branch name only
git rev-parse --abbrev-ref HEAD
```

---

### Create a New Branch

```bash
# Create new branch and switch to it (recommended)
git checkout -b feature-branch-name

# OR create branch without switching
git branch feature-branch-name

# Verify you're on the new branch
git branch
```

**Common branch naming conventions:**
- Feature: `feature/description` (e.g., `feature/add-lidar-mapping`)
- Bug fix: `fix/description` (e.g., `fix/robot2-control`)
- Experimental: `experiment/description` (e.g., `experiment/new-mpc-controller`)
- Development: `dev` or `develop`

---

### Switch Between Branches

```bash
# Switch to an existing branch
git checkout main
git checkout feature-branch-name

# OR use newer syntax (Git 2.23+)
git switch main
git switch feature-branch-name

# Create and switch in one command
git switch -c new-branch-name
```

---

### Push Branch to GitHub

```bash
# Push new branch to GitHub for the first time
git push -u origin feature-branch-name

# After first push, just use
git push

# Push all local branches to GitHub
git push --all origin
```

---

### Make Changes on a Branch

```bash
# 1. Make sure you're on the right branch
git branch

# 2. Make your code changes...

# 3. Check what changed
git status

# 4. Stage changes
git add .

# 5. Commit changes
git commit -m "Add new feature: description"

# 6. Push to GitHub
git push origin feature-branch-name
```

---

### Merge Branch into Main

```bash
# 1. Switch to main branch
git checkout main

# 2. Pull latest changes from GitHub
git pull origin main

# 3. Merge your feature branch
git merge feature-branch-name

# 4. Push merged changes to GitHub
git push origin main
```

**If there are conflicts:**
```bash
# Git will show conflict files
# 1. Open conflicted files and resolve manually
# 2. Stage resolved files
git add .

# 3. Complete the merge
git commit -m "Merge feature-branch-name into main"

# 4. Push
git push origin main
```

---

### Delete a Branch

```bash
# Delete local branch (after merging)
git branch -d feature-branch-name

# Force delete local branch (even if not merged)
git branch -D feature-branch-name

# Delete remote branch on GitHub
git push origin --delete feature-branch-name
```

---

### View Branch Information

```bash
# List all local branches
git branch

# List all remote branches
git branch -r

# List all branches (local and remote)
git branch -a

# Show last commit on each branch
git branch -v

# Show branches that have been merged into current branch
git branch --merged

# Show branches that haven't been merged
git branch --no-merged
```

---

### Common Branch Workflows

#### Workflow 1: Create Feature Branch from Main

```bash
# 1. Start from main branch
git checkout main
git pull origin main

# 2. Create and switch to feature branch
git checkout -b feature/new-controller

# 3. Make changes and commit
git add .
git commit -m "feat: add new MPC controller implementation"

# 4. Push feature branch to GitHub
git push -u origin feature/new-controller

# 5. Create Pull Request on GitHub (optional)
# Visit: https://github.com/YOUR_USERNAME/YOUR_REPO/pulls

# 6. After review, merge to main
git checkout main
git merge feature/new-controller
git push origin main

# 7. Delete feature branch
git branch -d feature/new-controller
git push origin --delete feature/new-controller
```

---

#### Workflow 2: Work on Existing Branch

```bash
# 1. Fetch all branches from GitHub
git fetch origin

# 2. Switch to existing branch
git checkout feature/existing-branch

# 3. Pull latest changes
git pull origin feature/existing-branch

# 4. Make your changes...
git add .
git commit -m "Update feature implementation"

# 5. Push changes
git push origin feature/existing-branch
```

---

#### Workflow 3: Create Development Branch

```bash
# 1. Create long-lived development branch
git checkout -b develop
git push -u origin develop

# 2. Create feature branches from develop
git checkout develop
git checkout -b feature/new-feature

# 3. After feature is done, merge to develop
git checkout develop
git merge feature/new-feature

# 4. When ready for release, merge develop to main
git checkout main
git merge develop
git push origin main
```

---

### Branch Comparison

```bash
# Show commits in feature-branch not in main
git log main..feature-branch

# Show commits in main not in feature-branch
git log feature-branch..main

# Show difference between branches
git diff main..feature-branch

# Show files that differ between branches
git diff --name-only main..feature-branch
```

---

### Update Branch with Latest Main

```bash
# Option 1: Merge main into your branch
git checkout feature-branch
git merge main

# Option 2: Rebase your branch on top of main (cleaner history)
git checkout feature-branch
git rebase main

# After rebase, you may need to force push
git push -f origin feature-branch
```

**Note:** Use rebase carefully, especially on shared branches!

---

### Quick Branch Commands Reference

```bash
# Create and switch to new branch
git checkout -b branch-name

# Switch to existing branch
git checkout branch-name

# Push new branch to GitHub
git push -u origin branch-name

# Delete local branch
git branch -d branch-name

# Delete remote branch
git push origin --delete branch-name

# List all branches
git branch -a

# Show current branch
git branch --show-current
```

---

## 🔍 Verification Checklist

Go to your GitHub repository and check:
- [ ] All Python files are there (.py files)
- [ ] Documentation is there (README.md, etc.)
- [ ] Installation scripts are there (.sh files)
- [ ] robocasa/ folder is NOT there ✗
- [ ] robosuite/ folder is NOT there ✗
- [ ] Repository size is ~10-50 MB (not 500+ MB)

---

## 🆘 If Something Goes Wrong

### Problem: "permission denied" or "authentication failed"
```bash
# Use HTTPS with token instead
git remote set-url origin https://YOUR_USERNAME:YOUR_TOKEN@github.com/YOUR_USERNAME/YOUR_REPO.git
```

Get token from: https://github.com/settings/tokens

### Problem: Large folders still in git
```bash
# Remove them and recommit
git rm -r --cached robocasa robosuite
git commit -m "Remove large folders"
git push -f origin main
```

### Problem: "src refspec main does not match any"
```bash
# Use master instead
git push -u origin master
```

### Problem: "fatal: refusing to merge unrelated histories"
```bash
# Allow merging unrelated histories
git pull origin main --allow-unrelated-histories
```

### Problem: Branch shows as "ahead" or "behind"
```bash
# If local branch is behind remote
git pull origin branch-name

# If local branch is ahead of remote
git push origin branch-name

# If branches have diverged
git pull origin branch-name
# Resolve any conflicts, then
git push origin branch-name
```

---

## 📝 Complete Command List (All at Once)

**IMPORTANT:** Update YOUR_USERNAME and YOUR_REPO first!

```bash
cd C:/Users/nahoy/Mujoco_local_structured/code2
git status
git init
git branch -M main
git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPO.git
git rm -r --cached robocasa 2>/dev/null
git rm -r --cached robosuite 2>/dev/null
git rm -r --cached c_generated_code 2>/dev/null
git add .
git status
git commit -m "Initial commit: MuJoCo dual-robot system with documentation"
git push -u origin main
```

---

## 🎯 After First Upload - Making Future Changes

### On Main Branch

```bash
# Check what changed
git status

# Stage changes
git add .

# Commit with message
git commit -m "Description of what you changed"

# Push
git push origin main
```

### On Feature Branch

```bash
# Create feature branch
git checkout -b feature/my-new-feature

# Make changes, then commit
git add .
git commit -m "feat: add new feature"

# Push feature branch
git push -u origin feature/my-new-feature

# When done, merge to main
git checkout main
git merge feature/my-new-feature
git push origin main
```

---

## ✅ Success Indicators

You'll know it worked when:
1. `git push` completes without errors
2. You can see your files on GitHub website
3. Repository size is reasonable (~10-50 MB)
4. robocasa/ and robosuite/ are NOT on GitHub
5. README.md is displayed on the repository homepage
6. Branches are visible on GitHub (if you pushed branches)

---

## 🌿 Branch Best Practices

1. **Keep main branch stable** - Only merge tested code to main
2. **Use descriptive branch names** - `feature/add-lidar` not `test123`
3. **Delete merged branches** - Keep repository clean
4. **Pull before push** - Always pull latest changes before pushing
5. **Commit often** - Make small, focused commits
6. **Write clear commit messages** - Explain what and why

---

**Repository URL format:**
- HTTPS: `https://github.com/YOUR_USERNAME/YOUR_REPO.git`
- SSH: `git@github.com:YOUR_USERNAME/YOUR_REPO.git`

**Use HTTPS if you're not sure!**
