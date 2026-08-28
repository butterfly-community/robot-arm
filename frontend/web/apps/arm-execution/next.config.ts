import type { NextConfig } from "next";
const config: NextConfig = {
  basePath: "/arm-execution",
  trailingSlash: true,
  output: "standalone",
  transpilePackages: ["@robot/ui", "@robot/contracts", "@robot/gateway-client"],
};
export default config;
