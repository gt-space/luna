import { defineConfig } from "vite";
import solid from "vite-plugin-solid";

export default defineConfig({
  plugins: [solid()],
  clearScreen: false,
  server: {
    host: "127.0.0.1",
    port: 1421,
    strictPort: true,
    watch: { ignored: ["**/src-tauri/target/**"] },
  },
  build: { target: ["es2021", "chrome100", "safari15"] },
});
