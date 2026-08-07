import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// The dev server proxies API and WebSocket traffic to the FastAPI backend so
// the frontend can be served from :5173 while talking to :8000 without CORS
// headaches. In production the backend serves webapp/dist itself.
const BACKEND = process.env.SLAM_BACKEND ?? 'http://127.0.0.1:8000'
const BACKEND_WS = BACKEND.replace(/^http/, 'ws')

export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    // Listen on all interfaces so you can open the UI from a phone or tablet
    // on the same network as the robot.
    host: true,
    proxy: {
      '/api': { target: BACKEND, changeOrigin: true },
      '/healthz': { target: BACKEND, changeOrigin: true },
      '/ws': { target: BACKEND_WS, ws: true },
    },
  },
  build: {
    outDir: 'dist',
    sourcemap: true,
  },
})
