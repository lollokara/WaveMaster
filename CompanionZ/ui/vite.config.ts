import { defineConfig } from 'vite';
import { mockZbro } from './mock/plugin';

export default defineConfig({
  plugins: [mockZbro()],
  build: {
    outDir: 'dist',
    assetsDir: '',            // flat output: LittleFS wants short flat names
    assetsInlineLimit: 0,
    cssCodeSplit: false,
    modulePreload: false,
    target: 'es2020',
    rollupOptions: {
      output: {
        entryFileNames: 'app-[hash:8].js',
        chunkFileNames: 'c-[hash:8].js',
        assetFileNames: '[name]-[hash:8][extname]',
      },
    },
  },
});
