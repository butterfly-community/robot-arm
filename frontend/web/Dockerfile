FROM robot-arm-services-frontend-base:2026.09.03-r1
WORKDIR /workspace
COPY . .
RUN pnpm install --frozen-lockfile
RUN pnpm -r --filter './web/apps/*' build
ENV NODE_ENV=production HOSTNAME=0.0.0.0 PORT=3000
CMD ["sh", "-c", "cd web/apps/${WEB_APP} && exec pnpm exec next start"]
