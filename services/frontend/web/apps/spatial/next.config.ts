import type { NextConfig } from "next";
const config: NextConfig = {
  basePath: "/spatial",
  trailingSlash: true,
  output: "standalone",
  transpilePackages: ["@robot/ui", "@robot/contracts", "@robot/gateway-client"],
};
export default config;
