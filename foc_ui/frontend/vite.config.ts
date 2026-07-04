import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'

// Dev: Vite serves on :5173 and proxies /api and /ws to the FastAPI backend
// on :8000, so the browser talks to a single origin in both modes.
// Prod: `npm run build` outputs to ./dist, which the backend serves at /.
export default defineConfig({
  plugins: [vue()],
  server: {
    host: '127.0.0.1',
    port: 5173,
    proxy: {
      '/api': { target: 'http://127.0.0.1:8000', changeOrigin: true },
      '/ws':  { target: 'ws://127.0.0.1:8000', ws: true, changeOrigin: true },
    },
  },
  build: {
    outDir: 'dist',
    // Disable sourcemaps in prod build to keep the artifact compact; the
    // backend serves these files as static assets.
    sourcemap: false,
  },
})
