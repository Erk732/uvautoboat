# AutoBoat Wiki Documentation

This directory contains wiki pages for the AutoBoat GitHub Wiki.

---

## Created Wiki Pages

The following pages have been created and are ready for upload:

| Page | Description | Status |
|:-----|:------------|:------:|
| **Home.md** | Wiki landing page with navigation | ✅ Ready |
| **Installation_Guide.md** | Complete setup instructions | ✅ Ready |
| **Quick_Start.md** | 5-minute getting started guide | ✅ Ready |
| **System_Overview.md** | Architecture and design philosophy | ✅ Ready |
| **SASS.md** | Simple anti-stuck recovery system (deprecated) | ✅ Ready |
| **3D_LIDAR_Processing.md** | OKO perception system explained | ✅ Ready |
| **Common_Issues.md** | Troubleshooting guide | ✅ Ready |
| **UPLOAD_INSTRUCTIONS.md** | How to upload these pages | ✅ Ready |

---

## Upload Instructions

See **[UPLOAD_INSTRUCTIONS.md](UPLOAD_INSTRUCTIONS.md)** for detailed steps.

### Quick Upload (Git Method)

```bash
# From uvautoboat directory
git clone https://github.com/Erk732/uvautoboat.wiki.git wiki-repo
cp wiki/*.md wiki-repo/
cd wiki-repo
git add *.md
git commit -m "Add comprehensive wiki documentation"
git push origin master
```

---

## Additional Pages Needed

These pages are referenced in the wiki but not yet created:

### Priority 1 (High Usage)

- [ ] `Terminal-Mission-Control.md` — CLI usage guide
- [ ] `Web-Dashboard-Guide.md` — Dashboard features
- [ ] `Configuration-and-Tuning.md` — Parameter reference
- [ ] `Astar-Path-Planning.md` — A* algorithm details
- [ ] `FAQ.md` — Frequently asked questions

### Priority 2 (Architecture)

- [ ] `Vostok1-Architecture.md` — Integrated system
- [ ] `Modular-Architecture.md` — OKO-SPUTNIK-BURAN
- [ ] `Atlantis-Architecture.md` — Control group approach
- [ ] `ROS2-Topic-Flow.md` — Topic communication diagrams

### Priority 3 (Concepts)

- [ ] `GPS-Navigation.md` — Coordinate systems
- [ ] `PID-Control.md` — Controller fundamentals
- [ ] `Differential-Thrust.md` — Two-thruster control
- [ ] `Kalman-Filtering.md` — State estimation theory

### Priority 4 (Features)

- [ ] `Waypoint-Skip-Strategy.md` — Skip logic
- [ ] `Obstacle-Avoidance-Loop.md` — Continuous control
- [ ] `Hazard-Zone-Management.md` — No-go areas
- [ ] `First-Mission-Tutorial.md` — Step-by-step walkthrough

### Priority 5 (Development)

- [ ] `Contributing.md` — Contribution guidelines
- [ ] `Code-Review-Standards.md` — Best practices
- [ ] `Testing-Guide.md` — Unit tests
- [ ] `API-Reference.md` — ROS 2 API docs

### Priority 6 (Misc)

- [ ] `Keyboard-Teleop.md` — Manual control
- [ ] `Launch-Files-Reference.md` — Launch file guide
- [ ] `Debug-Commands.md` — Advanced diagnostics
- [ ] `ROS2-Resources.md` — External links
- [ ] `VRX-Competition.md` — Competition info
- [ ] `Related-Projects.md` — Similar work

---

## Content Sources

Wiki content was extracted and reorganized from:

- `README.md` (main documentation)
- `LIDAR_OBSTACLE_AVOIDANCE_GUIDE.md`
- `MISSION_CONTROL_GUIDE.md`
- `LAUNCH_YAML_GUIDE.md`
- `CODE_REVIEW.md`
- `web_dashboard/vostok1/README_vostok1_dashboard.md`

---

## Wiki Structure Philosophy

### What Stays in README

- Quick project overview
- Installation quickstart
- Basic usage (2-terminal setup)
- Links to wiki for details
- Project status and key features

### What Goes in Wiki

- Detailed explanations
- Architecture deep-dives
- Comprehensive tutorials
- Troubleshooting guides
- API documentation
- Development guidelines

---

## Maintaining the Wiki

### Adding New Pages

1. Create `.md` file in `wiki/` directory
2. Follow existing page format:
   - Title as H1
   - Navigation links at bottom
   - Code examples with syntax highlighting
   - Tables for structured data
3. Add to `Home.md` navigation
4. Upload using git method

### Updating Existing Pages

1. Edit `.md` file locally
2. Test markdown rendering
3. Upload changes:

   ```bash
   cd wiki-repo
   git add <modified-file>.md
   git commit -m "Update: description"
   git push origin master
   ```

### Adding Images

1. Add image to `images/` in main repo
2. Reference in wiki:

   ```markdown
   ![Alt](https://raw.githubusercontent.com/Erk732/uvautoboat/main/images/file.png)
   ```

---

## Contribution Guidelines

When contributing to the wiki:

1. **Clarity**: Write for users new to ROS 2 and autonomous systems
2. **Examples**: Include code examples and expected output
3. **Links**: Cross-reference related pages
4. **Structure**: Use consistent heading hierarchy
5. **Testing**: Verify commands work before documenting
6. **Images**: Add diagrams for complex concepts
7. **Tables**: Use tables for comparisons and parameters

---

## Questions?

- Check [UPLOAD_INSTRUCTIONS.md](UPLOAD_INSTRUCTIONS.md)
- See [GitHub Wiki Docs](https://docs.github.com/en/communities/documenting-your-project-with-wikis)
- Open an issue on the repository

---

**Last Updated**: December 2025
**Maintained By**: AutoBoat Development Team
