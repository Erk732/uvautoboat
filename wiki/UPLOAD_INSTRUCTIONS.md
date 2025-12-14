# GitHub Wiki Upload Instructions

Step-by-step guide to upload these wiki pages to your GitHub repository wiki.

---

## Method 1: Manual Upload (Recommended for First Time)

### Step 1: Enable Wiki on GitHub

1. Go to your repository: `https://github.com/Erk732/uvautoboat`
2. Click **Settings** (top menu)
3. Scroll down to **Features** section
4. Check ✅ **Wikis** to enable
5. Click **Save changes**

### Step 2: Access Wiki

1. Click **Wiki** tab in your repository
2. Click **Create the first page** (if wiki is empty)

### Step 3: Upload Pages

For each `.md` file in the `wiki/` directory:

#### Upload Home Page

1. In Wiki, click **New Page**
2. Title: Leave as "Home" (this is automatic for first page)
3. Copy content from `wiki/Home.md`
4. Paste into editor
5. Click **Save Page**

#### Upload Other Pages

Repeat for each wiki page:

| File Name | Wiki Page Title |
|:----------|:----------------|
| `Installation_Guide.md` | Installation Guide |
| `Quick_Start.md` | Quick Start |
| `System_Overview.md` | System Overview |
| `SASS.md` | Simple Anti-Stuck (deprecated) |
| `3D_LIDAR_Processing.md` | 3D LIDAR Processing |
| `Common_Issues.md` | Common Issues |

**Steps for each:**

1. Click **New Page** button
2. Enter **Page title** (from table above, use hyphens not spaces)
3. Copy content from corresponding `.md` file
4. Paste into editor
5. Click **Save Page**

---

## Method 2: Git Clone (Advanced Users)

### Step 1: Clone Wiki Repository

GitHub wikis are actually Git repositories!

```bash
cd ~/seal_ws/src/uvautoboat
git clone https://github.com/Erk732/uvautoboat.wiki.git wiki-repo
```

### Step 2: Copy Wiki Files

```bash
cp wiki/*.md wiki-repo/
cd wiki-repo
```

### Step 3: Commit and Push

```bash
git add *.md
git commit -m "Add comprehensive wiki documentation"
git push origin master
```

**Note**: Wiki repository uses `master` branch, not `main`.

---

## Method 3: Batch Upload Script

Create a script to automate the upload process:

```bash
#!/bin/bash
# upload-wiki.sh

WIKI_DIR="wiki"
WIKI_REPO="https://github.com/Erk732/uvautoboat.wiki.git"

# Clone wiki repo
git clone $WIKI_REPO wiki-repo
cd wiki-repo

# Copy all markdown files
cp ../$WIKI_DIR/*.md .

# Commit and push
git add *.md
git commit -m "Update wiki documentation - $(date +%Y-%m-%d)"
git push origin master

echo "✅ Wiki uploaded successfully!"
```

**Usage:**

```bash
chmod +x upload-wiki.sh
./upload-wiki.sh
```

---

## Verifying Upload

After uploading, verify the wiki:

1. Go to `https://github.com/Erk732/uvautoboat/wiki`
2. Check **Home** page loads correctly
3. Click links to verify page navigation
4. Check images display properly
5. Verify code blocks have syntax highlighting

---

## Wiki File Structure

Your wiki pages should be organized as:

```text
wiki/
├── Home.md                    # Landing page
├── Installation_Guide.md      # Setup instructions
├── Quick_Start.md             # 5-minute quick start
├── System_Overview.md         # Architecture overview
├── SASS.md                    # Simple Anti-Stuck (deprecated)
├── 3D_LIDAR_Processing.md     # OKO perception
├── Common_Issues.md           # Troubleshooting
└── [Additional pages...]      # Future additions
```

---

## Additional Pages to Create

Based on the wiki home navigation, you may want to create these pages later:

### Getting Started

- [ ] `First-Mission-Tutorial.md`

### Architecture

- [ ] `Vostok1-Architecture.md`
- [ ] `Modular-Architecture.md`
- [ ] `Atlantis-Architecture.md`
- [ ] `ROS2-Topic-Flow.md`

### User Guides

- [ ] `Terminal-Mission-Control.md`
- [ ] `Web-Dashboard-Guide.md`
- [ ] `Keyboard-Teleop.md`
- [ ] `Configuration-and-Tuning.md`
- [ ] `Launch-Files-Reference.md`

### Core Concepts

- [ ] `GPS-Navigation.md`
- [ ] `PID-Control.md`
- [ ] `Differential-Thrust.md`
- [ ] `Kalman-Filtering.md`

### Advanced Features

- [ ] `Astar-Path-Planning.md`
- [ ] `Waypoint-Skip-Strategy.md`
- [ ] `Obstacle-Avoidance-Loop.md`
- [ ] `Hazard-Zone-Management.md`

### Development

- [ ] `Contributing.md`
- [ ] `Code-Review-Standards.md`
- [ ] `Testing-Guide.md`
- [ ] `API-Reference.md`

### Troubleshooting

- [ ] `Debug-Commands.md`
- [ ] `FAQ.md`

### References

- [ ] `ROS2-Resources.md`
- [ ] `VRX-Competition.md`
- [ ] `Related-Projects.md`

---

## Wiki Maintenance

### Updating Pages

To update an existing page:

**Manual Method:**

1. Go to wiki page
2. Click **Edit** button (top right)
3. Make changes
4. Click **Save Page**

**Git Method:**

```bash
cd wiki-repo
# Edit files
git add .
git commit -m "Update: description of changes"
git push origin master
```

### Adding Images

To add images to wiki:

1. **Upload to repository first**:

   ```bash
   # In main repo
   git add images/new-image.png
   git commit -m "Add wiki image"
   git push
   ```

2. **Reference in wiki markdown**:

   ```markdown
   ![Alt text](https://raw.githubusercontent.com/Erk732/uvautoboat/main/images/new-image.png)
   ```

---

## Best Practices

### Link Format

Use relative wiki links:

```markdown
[Installation Guide](Installation_Guide)
```

NOT:

```markdown
[Installation Guide](https://github.com/Erk732/uvautoboat/wiki/Installation_Guide)
```

### Code Blocks

Use language identifiers:

```markdown
```bash
ros2 run plan vostok1
\`\`\`
```

### Tables

Use consistent formatting:

```markdown
| Column 1 | Column 2 |
|:---------|:---------|
| Data 1   | Data 2   |
```

### Headings

Use proper hierarchy:

```markdown
# Page Title (only one H1)
## Main Section (H2)
### Subsection (H3)
```

---

## Troubleshooting Wiki Upload

### Wiki Not Showing

**Cause**: Wiki not enabled in repository settings

**Solution**: Enable in Settings → Features → Wikis

### Images Not Loading

**Cause**: Image path incorrect

**Solution**: Use raw GitHub URLs:

```markdown
https://raw.githubusercontent.com/Erk732/uvautoboat/main/images/file.png
```

### Links Broken

**Cause**: Incorrect wiki link format

**Solution**: Use page title with underscores:

- Correct: `[System Overview](System_Overview)`
- Wrong: `[System Overview](System Overview)`

### Push Rejected (Git Method)

**Cause**: Wiki changed on GitHub

**Solution**: Pull first:

```bash
git pull origin master
git push origin master
```

---

## Next Steps

After uploading the wiki:

1. **Update main README** to link to wiki pages
2. **Create remaining wiki pages** from checklist above
3. **Add screenshots** to make pages more visual
4. **Get feedback** from team members
5. **Keep wiki updated** as code changes

---

## Questions?

If you need help with wiki upload:

- Check GitHub's [Wiki Documentation](https://docs.github.com/en/communities/documenting-your-project-with-wikis)
- Open an issue on the repository
- Contact the AutoBoat development team

---

## Good luck with your wiki! 🚀
