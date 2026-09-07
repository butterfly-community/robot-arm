import type { NextConfig } from "next";
const config: NextConfig = {
  experimental: { useTypeScriptCli: false },
  basePath: "/arm-execution",
  trailingSlash: true,
  transpilePackages: [
    "@robot/ui",
    "@robot/contracts",
    "@robot/gateway-client",
    "@robot/visualization",
  ],
};
export default config;
