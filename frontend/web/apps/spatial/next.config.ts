import type { NextConfig } from "next";
const config: NextConfig = {
  experimental: { useTypeScriptCli: false },
  basePath: "/spatial",
  trailingSlash: true,
  transpilePackages: [
    "@robot/contracts",
    "@robot/gateway-client",
    "@robot/ui",
    "@robot/visualization",
  ],
};
export default config;
