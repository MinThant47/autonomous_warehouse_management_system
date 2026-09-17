import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  // Expose the development UI to other devices on the local network.
  server: { host: '0.0.0.0' },
})
