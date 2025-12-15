# Web Dashboard Port Allocation

To prevent port conflicts between the two navigation systems, each dashboard runs on a separate port:

## Port Assignment

| System | Port | URL |
|--------|------|-----|
| **Robust Avoidance** (All-in-one) | **8001** | http://localhost:8001 |
| **Vostok1** (Modular) | **8002** | http://localhost:8002 |

## Other Ports

| Service | Port | URL/Address |
|---------|------|-------------|
| ROS Bridge WebSocket | 9090 | ws://localhost:9090 |
| Web Video Server | 8080 | http://localhost:8080 |
| Gazebo GUI | 11345 | http://localhost:11345 |

## Important Notes

1. **Never run both systems simultaneously** - They share the same robot topics and will conflict
2. **Hard refresh after switching** - Use `Ctrl + Shift + R` to clear browser cache when switching between dashboards
3. **Kill old processes** - The launch scripts automatically kill old HTTP servers on their respective ports
4. **Browser auto-open** - Each launch script opens its correct dashboard port automatically

## Troubleshooting

### Problem: Dashboard shows wrong content (Vostok1 shows Robust Avoidance or vice versa)
**Solution:** Kill all HTTP servers and restart the launch script
```bash
pkill -9 -f "http.server"
# Then run your desired launch script
```

### Problem: Dashboard loads without CSS styling (just HTML framework)
**Solution:**
1. Check that the HTTP server started successfully in the Dashboard tab
2. Verify the CSS file exists in the dashboard folder
3. Hard refresh the browser: `Ctrl + Shift + R`
4. Check browser console (F12) for file loading errors

### Problem: Port already in use error
**Solution:** Another process is using the port
```bash
# Find what's using the port
lsof -i :8001  # or :8002 for vostok1
# Kill the process
kill -9 <PID>
```

## File Structure

```
web_dashboard/
├── robust_avoidance/      # Port 8001
│   ├── index.html
│   ├── app.js
│   └── styles.css
│
├── vostok1/               # Port 8002
│   ├── index.html
│   ├── app.js
│   ├── style_merged.css
│   └── README_vostok1_dashboard.md
│
└── PORT_ALLOCATION.md     # This file
```
