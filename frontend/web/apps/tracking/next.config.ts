import type { NextConfig } from "next";

const config: NextConfig = {
  basePath: "/tracking",
  trailingSlash: true,
  output: "standalone",
  transpilePackages: [
    "@robot/contracts",
    "@robot/gateway-client",
    "@robot/ui",
    "@robot/visualization",
  ],
};

export default config;
