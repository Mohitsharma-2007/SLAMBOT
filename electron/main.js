const { app, BrowserWindow, ipcMain } = require('electron');
const path = require('path');
const fs = require('fs');
const { spawn } = require('child_process');
const http = require('http');
const os = require('os');

// Trap any uncaught exceptions globally so Electron never crashes with dialogs
process.on('uncaughtException', (err) => {
  console.warn('Trapped uncaught exception:', err.message);
});

process.on('unhandledRejection', (reason) => {
  console.warn('Trapped unhandled rejection:', reason);
});

let mainWindow;
let wslProcess = null;
let backendProcess = null;
let backendHealthy = false;
let rosActive = false;
let isSpawningBackend = false;

// Get local IPv4 address (e.g., 10.147.215.189 or 192.168.1.x)
function getLocalIP() {
  try {
    const interfaces = os.networkInterfaces();
    for (const name of Object.keys(interfaces)) {
      for (const iface of interfaces[name]) {
        if (iface.family === 'IPv4' && !iface.internal && !iface.address.startsWith('169.254') && !iface.address.startsWith('172.')) {
          return iface.address;
        }
      }
    }
  } catch (e) {}
  return '127.0.0.1';
}

function findBackendDir() {
  const candidates = [
    path.join(__dirname, '../backend'),
    path.join(__dirname, 'backend'),
    path.join(process.cwd(), 'backend'),
    'd:\\SLAM Bot\\backend'
  ];
  for (const dir of candidates) {
    try {
      if (fs.existsSync(dir) && fs.existsSync(path.join(dir, 'main.py'))) {
        return dir;
      }
    } catch (e) {}
  }
  return path.join(__dirname, '../backend');
}

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1280,
    height: 850,
    minWidth: 960,
    minHeight: 650,
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      nodeIntegration: false,
      contextIsolation: true
    },
    title: "SLAM Bot — All-In-One Command Center",
    backgroundColor: '#080b11',
    icon: path.join(__dirname, 'icon.png')
  });

  mainWindow.loadFile(path.join(__dirname, 'startup.html'));

  mainWindow.webContents.on('did-finish-load', () => {
    sendLog('System', `Local Network Host IP: ${getLocalIP()}:8000`, 'ok');
    checkBackendHealth();
  });

  mainWindow.on('closed', () => {
    mainWindow = null;
  });
}

function sendLog(source, message, level = 'info') {
  if (mainWindow && !mainWindow.isDestroyed()) {
    mainWindow.webContents.send('log-message', { source, message: (message || '').toString().trim(), level });
  }
}

function updateStatus() {
  if (mainWindow && !mainWindow.isDestroyed()) {
    mainWindow.webContents.send('status-update', {
      backend: backendHealthy,
      ros: rosActive,
      ready: backendHealthy,
      localIp: getLocalIP()
    });
  }
}

function checkBackendHealth(onFail) {
  const req = http.get('http://127.0.0.1:8000/healthz', (res) => {
    if (res.statusCode === 200) {
      backendHealthy = true;
      sendLog('FastAPI', 'Backend server active & healthy (Port 8000)', 'ok');
      updateStatus();
    } else {
      if (onFail) onFail();
    }
  });

  req.on('error', () => {
    if (onFail) onFail();
  });

  req.setTimeout(1000, () => {
    req.destroy();
    if (onFail) onFail();
  });
}

// 1. Safe FastAPI Backend Spawner
function startBackend() {
  sendLog('System', 'Probing FastAPI Backend on port 8000...', 'info');

  checkBackendHealth(() => {
    if (backendHealthy || isSpawningBackend) return;
    isSpawningBackend = true;

    sendLog('FastAPI', 'Starting Python Uvicorn backend server...', 'info');
    try {
      const isWin = process.platform === 'win32';
      const cmd = isWin ? 'python' : 'python3';
      const backendDir = findBackendDir();

      backendProcess = spawn(cmd, ['-m', 'uvicorn', 'main:app', '--host', '0.0.0.0', '--port', '8000', '--ws', 'wsproto'], {
        cwd: backendDir,
        env: { ...process.env, PATH: process.env.PATH || '' }
      });

      backendProcess.on('error', (err) => {
        sendLog('FastAPI', `Notice: ${err.message}. Checking external backend...`, 'warn');
        checkBackendHealth();
      });

      if (backendProcess.stdout) {
        backendProcess.stdout.on('data', (data) => {
          const text = data.toString();
          sendLog('FastAPI', text, 'info');
          if (text.includes('Application startup complete') || text.includes('Uvicorn running')) {
            backendHealthy = true;
            isSpawningBackend = false;
            updateStatus();
          }
        });
      }

      if (backendProcess.stderr) {
        backendProcess.stderr.on('data', (data) => {
          sendLog('FastAPI', data.toString(), 'info');
        });
      }

      backendProcess.on('exit', (code) => {
        isSpawningBackend = false;
        // Verify if backend is reachable before setting unhealthy
        checkBackendHealth(() => {
          backendHealthy = false;
          updateStatus();
        });
      });
    } catch (e) {
      isSpawningBackend = false;
      sendLog('FastAPI', `Notice: ${e.message}`, 'warn');
    }

    pollBackendHealth();
  });
}

function pollBackendHealth() {
  let attempts = 0;
  const check = () => {
    if (backendHealthy) return;
    attempts++;
    checkBackendHealth(() => {
      if (attempts < 60) setTimeout(check, 500);
    });
  };
  setTimeout(check, 500);
}

// 2. Safe WSL2 ROS2 Stack Spawner
function startROS2WSL() {
  sendLog('System', 'Checking WSL2 ROS2 stack...', 'wsl');
  
  try {
    wslProcess = spawn('wsl.exe', [
      '-e', 'bash', '-c', 
      'source /opt/ros/humble/setup.bash && cd /mnt/d/SLAM\\ Bot/ros2_ws && source install/setup.bash && ros2 launch slam_bot_bringup bringup.launch.py'
    ], {
      env: { ...process.env, PATH: process.env.PATH || '' }
    });

    wslProcess.on('error', (err) => {
      sendLog('ROS2', `WSL notice: ${err.message}. If running natively, ROS bridge connects on port 8000.`, 'warn');
    });

    if (wslProcess.stdout) {
      wslProcess.stdout.on('data', (data) => {
        const text = data.toString();
        sendLog('ROS2', text, 'wsl');
        if (text.includes('bringup') || text.includes('slam_toolbox') || text.includes('Nav2') || text.includes('standalone bridge started')) {
          rosActive = true;
          updateStatus();
        }
      });
    }

    if (wslProcess.stderr) {
      wslProcess.stderr.on('data', (data) => {
        sendLog('ROS2', data.toString(), 'warn');
      });
    }

    wslProcess.on('exit', (code) => {
      rosActive = false;
      updateStatus();
    });
  } catch (e) {
    sendLog('ROS2', `WSL launch skipped: ${e.message}`, 'warn');
  }
}

function stopAllProcesses() {
  if (backendProcess) {
    try { backendProcess.kill(); } catch (e) {}
    backendProcess = null;
  }
  if (wslProcess) {
    try { wslProcess.kill(); } catch (e) {}
    wslProcess = null;
  }
  backendHealthy = false;
  rosActive = false;
  isSpawningBackend = false;
}

// IPC handlers
ipcMain.on('launch-dashboard', () => {
  if (mainWindow && !mainWindow.isDestroyed()) {
    sendLog('System', 'Navigating to Dashboard UI...', 'ok');
    mainWindow.loadURL('http://127.0.0.1:8000/');
  }
});

ipcMain.on('restart-all', () => {
  stopAllProcesses();
  setTimeout(() => {
    startBackend();
    setTimeout(startROS2WSL, 1500);
  }, 800);
});

app.on('ready', () => {
  createWindow();
  setTimeout(() => {
    startBackend();
    setTimeout(startROS2WSL, 2000);
  }, 500);
});

app.on('window-all-closed', () => {
  stopAllProcesses();
  if (process.platform !== 'darwin') {
    app.quit();
  }
});
