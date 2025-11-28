# How to Upload Project Without RoboCasa

This guide explains how to upload your project to GitHub without including the large RoboCasa directory.

## Quick Summary

✅ Your `.gitignore` already excludes `robocasa/`
✅ Users will install RoboCasa separately using your documentation
✅ Follow the steps below to ensure clean upload

---

## Step 1: Verify RoboCasa is Ignored

Check if robocasa is currently tracked by git:

```bash
cd ~/mujoco_pick_place
git ls-files | grep "^robocasa/"
```

**Expected result**: No output (robocasa is not tracked)

If you see files listed, see Step 2 to remove them.

---

## Step 2: Remove RoboCasa from Git Tracking (If Needed)

If robocasa files are currently tracked, remove them:

```bash
cd ~/mujoco_pick_place

# Remove robocasa from git tracking (but keep local files)
git rm -r --cached robocasa/

# Verify .gitignore includes robocasa
grep "robocasa" .gitignore

# Commit the removal
git add .gitignore
git commit -m "Remove robocasa from git tracking - users will install separately"
```

---

## Step 3: Verify What Will Be Uploaded

Check what git will include:

```bash
cd ~/mujoco_pick_place

# See all tracked files
git ls-files

# See status
git status

# Check the size of what will be uploaded
git count-objects -vH
```

**Look for**:
- ✅ No `robocasa/` directory in tracked files
- ✅ Helper scripts ARE included: `fix_robocasa_deps.py`, `install_robocasa_complete.sh`
- ✅ Documentation IS included: `ROBOCASA_INSTALL.md`, `README.md`

---

## Step 4: Create/Update .gitattributes (Optional)

Create a `.gitattributes` file to ensure consistent line endings:

```bash
cd ~/mujoco_pick_place
cat > .gitattributes << 'EOF'
# Auto detect text files and perform LF normalization
* text=auto

# Python
*.py text eol=lf

# Shell scripts
*.sh text eol=lf

# Markdown
*.md text eol=lf

# XML files
*.xml text eol=lf

# JSON files
*.json text eol=lf

# Binary files
*.png binary
*.jpg binary
*.jpeg binary
*.npz binary
*.so binary
*.dll binary
EOF

git add .gitattributes
git commit -m "Add .gitattributes for consistent line endings"
```

---

## Step 5: Add and Commit Recent Changes

Commit the README updates and helper scripts:

```bash
cd ~/mujoco_pick_place

# Add all changes
git add .

# Check what will be committed
git status

# Commit with a descriptive message
git commit -m "Add RoboCasa installation documentation and helper scripts

- Updated README.md with RoboCasa installation instructions
- Added ROBOCASA_INSTALL.md quick reference guide
- Added fix_robocasa_deps.py diagnostic tool
- Added install_robocasa_complete.sh automated installer
- Documented robosuite dependency issue and solutions"
```

---

## Step 6: Push to GitHub

### First Time Push (New Repository)

If you haven't created a GitHub repository yet:

1. **Create repository on GitHub**:
   - Go to https://github.com/new
   - Repository name: `mujoco_pick_place` (or your choice)
   - Description: "Dual-robot MuJoCo simulation with navigation, pick-place, and LLM control"
   - Set to Public or Private
   - **DO NOT** initialize with README (you already have one)
   - Click "Create repository"

2. **Link and push**:
   ```bash
   cd ~/mujoco_pick_place
   
   # Add GitHub as remote (replace YOUR_USERNAME)
   git remote add origin https://github.com/YOUR_USERNAME/mujoco_pick_place.git
   
   # Push to GitHub
   git branch -M main
   git push -u origin main
   ```

### Subsequent Pushes

If repository already exists:

```bash
cd ~/mujoco_pick_place

# Push to GitHub
git push origin main
```

---

## Step 7: Add Instructions to README (Already Done!)

Your README.md already includes:

✅ Clear explanation that robocasa is NOT included
✅ Installation instructions for robocasa
✅ Links to helper scripts
✅ Troubleshooting guide

Users will see:
```markdown
### RoboCasa Kitchen Environments

**Quick Reference**: See ROBOCASA_INSTALL.md for complete installation guide.

**Important**: RoboCasa requires robosuite as a dependency...
```

---

## What Users Will Do

When someone clones your repository:

```bash
# 1. Clone your repository
git clone https://github.com/YOUR_USERNAME/mujoco_pick_place.git
cd mujoco_pick_place

# 2. Install basic dependencies
pip install -r requirements.txt

# 3. Clone RoboCasa separately
git clone https://github.com/robocasa/robocasa.git

# 4. Install RoboCasa (following your documentation)
pip install robosuite --break-system-packages
cd robocasa && pip install -e . --break-system-packages

# 5. Run the simulation
python3 main.py
```

---

## Alternative: Using Git Submodule (NOT Recommended)

You could make robocasa a git submodule, but this is NOT recommended because:

❌ Users would need to learn submodule commands
❌ Submodule updates are tricky
❌ RoboCasa is actively developed (frequent updates)
❌ Your helper scripts already make installation easy

**If you still want to use submodules**:
```bash
# Add as submodule (NOT recommended)
git submodule add https://github.com/robocasa/robocasa.git robocasa
git commit -m "Add robocasa as submodule"
git push

# Users would then need to:
git clone --recurse-submodules https://github.com/YOUR_USERNAME/mujoco_pick_place.git
```

---

## Verification Checklist

Before pushing, verify:

- [ ] `git status` shows no `robocasa/` files
- [ ] `.gitignore` includes `robocasa/`
- [ ] `README.md` has RoboCasa installation instructions
- [ ] `ROBOCASA_INSTALL.md` exists and is tracked
- [ ] `fix_robocasa_deps.py` is tracked
- [ ] `install_robocasa_complete.sh` is tracked
- [ ] All documentation references are correct

---

## File Size Considerations

### Without RoboCasa (Your Setup)
```
Total repository size: ~5-50 MB
- Python code: ~2 MB
- XML models: ~1-3 MB
- Documentation: <1 MB
- Assets: ~2-10 MB
- LIDAR maps: ~1-5 MB (optional, can be .gitignored)
```

### With RoboCasa (If Included)
```
Total repository size: ~500 MB - 1+ GB
- RoboCasa code: ~100 MB
- Kitchen assets: ~300-500 MB
- Dependencies: ~100+ MB
```

**Conclusion**: Excluding RoboCasa makes your repository 10-100x smaller! ✅

---

## Troubleshooting

### "robocasa still showing in git status"

```bash
# Force remove from cache
git rm -rf --cached robocasa/
git add .gitignore
git commit -m "Remove robocasa from tracking"
```

### "Want to remove robocasa from history"

If you previously committed robocasa and want to remove it from history:

```bash
# WARNING: This rewrites history! Only for new repos.
git filter-branch --tree-filter 'rm -rf robocasa' HEAD
git push --force
```

### "Repository too large"

If GitHub complains about size:

```bash
# Find large files
git rev-list --objects --all | \
  git cat-file --batch-check='%(objecttype) %(objectname) %(objectsize) %(rest)' | \
  awk '/^blob/ {print substr($0,6)}' | \
  sort --numeric-sort --key=2 | \
  tail -20

# Remove large files from history
git filter-branch --tree-filter 'rm -f path/to/large/file' HEAD
```

---

## Summary

✅ **DO include**:
- All Python source code
- Documentation (README.md, ROBOCASA_INSTALL.md, etc.)
- Helper scripts (fix_robocasa_deps.py, install_robocasa_complete.sh)
- Robot models (XML files)
- Configuration files
- Requirements.txt

❌ **DO NOT include**:
- robocasa/ directory (users install separately)
- robosuite/ directory (users install separately)  
- Large data files (*.h5, *.pkl)
- Build artifacts (c_generated_code/, *.so)
- Personal API keys

---

## Need Help?

If you encounter issues:

1. Check `.gitignore` is working: `git check-ignore robocasa/`
2. Verify git status: `git status --ignored`
3. Check what will be pushed: `git ls-files`
4. Ask for help with specific error messages

---

**Ready to upload?** Follow Step 5 and Step 6 above! 🚀
