import type { NextConfig } from "next";
const config: NextConfig = {
  experimental: { useTypeScriptCli: false },
  basePath: "/motion",
  trailingSlash: true,
  output: "standalone",
  transpilePackages: ["@robot/ui", "@robot/contracts", "@robot/gateway-client"],
};
export default config;
