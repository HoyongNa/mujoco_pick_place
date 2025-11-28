#!/bin/bash

# Quick GitHub Upload Script
# This script prepares and uploads your project without robocasa

echo "=============================================="
echo "GitHub Upload Helper"
echo "=============================================="
echo ""

cd ~/mujoco_pick_place

# Step 1: Check if robocasa is tracked
echo "Step 1: Checking if robocasa is tracked by git..."
ROBOCASA_FILES=$(git ls-files | grep "^robocasa/" | wc -l)

if [ "$ROBOCASA_FILES" -gt 0 ]; then
    echo "⚠️  Found $ROBOCASA_FILES robocasa files tracked by git"
    echo "   Removing them from git tracking..."
    git rm -r --cached robocasa/
    echo "✅ Removed robocasa from git tracking"
else
    echo "✅ robocasa is not tracked by git (good!)"
fi

# Step 2: Verify .gitignore
echo ""
echo "Step 2: Verifying .gitignore..."
if grep -q "robocasa/" .gitignore; then
    echo "✅ robocasa/ is in .gitignore"
else
    echo "⚠️  Adding robocasa/ to .gitignore..."
    echo "" >> .gitignore
    echo "# RoboCasa (install separately)" >> .gitignore
    echo "robocasa/" >> .gitignore
    echo "✅ Added robocasa/ to .gitignore"
fi

# Step 3: Check for large files
echo ""
echo "Step 3: Checking for large files..."
git ls-files | xargs ls -lh 2>/dev/null | awk '{if ($5 ~ /M$/ && $5+0 > 10) print $9 " (" $5 ")"}'
echo "   (Files larger than 10MB are shown above)"

# Step 4: Show what will be committed
echo ""
echo "Step 4: Checking repository status..."
git status

# Step 5: Offer to commit
echo ""
echo "=============================================="
read -p "Do you want to commit changes now? (y/n): " COMMIT_CHOICE

if [ "$COMMIT_CHOICE" = "y" ] || [ "$COMMIT_CHOICE" = "Y" ]; then
    git add .
    
    echo ""
    echo "Enter commit message (or press Enter for default):"
    read COMMIT_MSG
    
    if [ -z "$COMMIT_MSG" ]; then
        COMMIT_MSG="Update project with RoboCasa installation documentation"
    fi
    
    git commit -m "$COMMIT_MSG"
    echo "✅ Changes committed!"
else
    echo "⏭️  Skipping commit. You can commit manually later."
fi

# Step 6: Offer to push
echo ""
echo "=============================================="

# Check if remote exists
if git remote | grep -q "origin"; then
    echo "Remote 'origin' found: $(git remote get-url origin)"
    echo ""
    read -p "Do you want to push to GitHub now? (y/n): " PUSH_CHOICE
    
    if [ "$PUSH_CHOICE" = "y" ] || [ "$PUSH_CHOICE" = "Y" ]; then
        echo "Pushing to GitHub..."
        git push origin main
        
        if [ $? -eq 0 ]; then
            echo "✅ Successfully pushed to GitHub!"
        else
            echo "❌ Push failed. Check your credentials and remote URL."
        fi
    else
        echo "⏭️  Skipping push. You can push manually later with:"
        echo "   git push origin main"
    fi
else
    echo "⚠️  No remote 'origin' configured."
    echo ""
    echo "To add GitHub remote:"
    echo "  1. Create repository on GitHub"
    echo "  2. Run: git remote add origin https://github.com/YOUR_USERNAME/mujoco_pick_place.git"
    echo "  3. Run: git push -u origin main"
fi

echo ""
echo "=============================================="
echo "Summary"
echo "=============================================="
echo ""
echo "✅ Repository is ready to upload without robocasa"
echo ""
echo "What users will do after cloning:"
echo "  1. git clone https://github.com/YOUR_USERNAME/mujoco_pick_place.git"
echo "  2. cd mujoco_pick_place"
echo "  3. git clone https://github.com/robocasa/robocasa.git"
echo "  4. pip install robosuite --break-system-packages"
echo "  5. cd robocasa && pip install -e . --break-system-packages"
echo ""
echo "Your documentation explains all of this!"
echo ""
echo "For more details, see: GITHUB_UPLOAD_GUIDE.md"
echo "=============================================="
