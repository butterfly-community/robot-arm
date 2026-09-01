import type { NextConfig } from "next";

const config: NextConfig = {
  experimental: { useTypeScriptCli: false },
  basePath: "/tracking",
  trailingSlash: true,
  transpilePackages: ["@robot/contracts", "@robot/gateway-client", "@robot/ui"],
};

export default config;
