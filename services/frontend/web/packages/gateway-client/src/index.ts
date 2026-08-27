import type { Json, Namespace, Snapshot } from "@robot/contracts";
import { nanoid } from "nanoid/non-secure";
import { useEffect, useRef, useState } from "react";

export function requestId(): string {
  return nanoid();
}

export async function getSnapshot(namespace: Namespace): Promise<Snapshot> {
  const response = await fetch(`/api/${namespace}/state`, {
    cache: "no-store",
  });
  if (!response.ok) throw new Error(await response.text());
  return response.json() as Promise<Snapshot>;
}

export async function post<T extends Json>(
  path: string,
  value: T,
): Promise<Json> {
  const response = await fetch(path, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify(value),
  });
  return responseValue(response);
}

export async function patch<T extends Json>(
  path: string,
  value: T,
): Promise<Json> {
  const response = await fetch(path, {
    method: "PATCH",
    headers: { "content-type": "application/json" },
    body: JSON.stringify(value),
  });
  return responseValue(response);
}

async function responseValue(response: Response): Promise<Json> {
  if (!response.ok) throw new Error(await response.text());
  const value = (await response.json()) as Json;
  if (
    value &&
    !Array.isArray(value) &&
    typeof value === "object" &&
    typeof value.original_error === "string"
  )
    throw new Error(value.original_error);
  return value;
}

export function subscribe(
  namespace: Namespace,
  update: (snapshot: Snapshot) => void,
  failed: (message: string) => void,
): () => void {
  let stopped = false;
  let socket: WebSocket | undefined;
  let retry: ReturnType<typeof setTimeout> | undefined;
  const connect = () => {
    const scheme = location.protocol === "https:" ? "wss" : "ws";
    socket = new WebSocket(`${scheme}://${location.host}/ws/${namespace}`);
    socket.onmessage = (event) => update(JSON.parse(event.data) as Snapshot);
    socket.onclose = () => {
      if (stopped) return;
      failed("状态连接已断开，正在重新连接");
      retry = setTimeout(connect, 1000);
    };
  };
  connect();
  return () => {
    stopped = true;
    if (retry) clearTimeout(retry);
    socket?.close();
  };
}

export function useGateway(namespace: Namespace) {
  const [snapshot, setSnapshot] = useState<Snapshot>();
  const [transportError, setTransportError] = useState<string>();
  const [operationError, setOperationError] = useState<string>();
  const pending = useRef<Snapshot | undefined>(undefined);
  useEffect(() => {
    const accept = (value: Snapshot) => {
      if (window.getSelection()?.toString()) {
        pending.current = value;
        return;
      }
      setSnapshot(value);
      setTransportError(undefined);
    };
    const applyPending = () => {
      if (window.getSelection()?.toString() || !pending.current) return;
      const value = pending.current;
      pending.current = undefined;
      setSnapshot(value);
      setTransportError(undefined);
    };
    document.addEventListener("selectionchange", applyPending);
    const dispose = subscribe(namespace, accept, setTransportError);
    getSnapshot(namespace)
      .then(accept)
      .catch((reason: unknown) =>
        setTransportError(
          reason instanceof Error ? reason.message : String(reason),
        ),
      );
    return () => {
      document.removeEventListener("selectionchange", applyPending);
      pending.current = undefined;
      dispose();
    };
  }, [namespace]);
  return {
    snapshot,
    error: operationError ?? transportError,
    setError: setOperationError,
  };
}
