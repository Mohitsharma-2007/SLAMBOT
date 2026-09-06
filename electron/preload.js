const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
  onLog: (callback) => ipcRenderer.on('log-message', (event, value) => callback(value)),
  onStatusUpdate: (callback) => ipcRenderer.on('status-update', (event, value) => callback(value)),
  restartAll: () => ipcRenderer.send('restart-all'),
  launchDashboard: () => ipcRenderer.send('launch-dashboard')
});
