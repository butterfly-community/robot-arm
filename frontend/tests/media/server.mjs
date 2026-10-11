import { createServer } from "vite";
import { fileURLToPath } from "node:url";

const server = await createServer({
  configFile: false,
  root: fileURLToPath(new URL("../..", import.meta.url)),
  cacheDir: "../temp/h264-vite",
  resolve: {
    alias: {
      "react-dom": fileURLToPath(
        new URL(
          "../../web/apps/perception/node_modules/react-dom",
          import.meta.url,
        ),
      ),
      react: fileURLToPath(
        new URL(
          "../../web/apps/perception/node_modules/react",
          import.meta.url,
        ),
      ),
    },
  },
  esbuild: { jsx: "automatic" },
  server: {
    host: "192.168.100.10",
    port: 38765,
    strictPort: true,
    proxy: {
      "/ws/camera-video": {
        target: "ws://192.168.100.10:38081",
        ws: true,
        rewrite: (path) => path.replace("/ws/camera-video", "/ws"),
      },
      "/snapshot": { target: "http://192.168.100.10:38081" },
    },
  },
});
await server.listen();
server.printUrls();
