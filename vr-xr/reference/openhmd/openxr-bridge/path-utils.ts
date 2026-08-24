export function resolvePublicFile(
  pathname: string,
  publicDirectory: URL,
): URL | null {
  const relative = pathname === "/" ? "index.html" : pathname.slice(1);
  if (
    !relative || relative.startsWith("/") ||
    !/^[a-zA-Z0-9._/-]+$/.test(relative) ||
    relative.split("/").some((segment) => segment === "..")
  ) {
    return null;
  }

  const resolved = new URL(relative, publicDirectory);
  return resolved.href.startsWith(publicDirectory.href) ? resolved : null;
}
