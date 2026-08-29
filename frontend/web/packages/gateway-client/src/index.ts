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

export function prepareRelativeControl(): Promise<Json> {
  return post("/api/motion/prepare-relative", {
    schema_version: 2,
    request_id: requestId(),
  });
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
): () => void {
  let stopped = false;
  let socket: WebSocket | undefined;
  let retry: ReturnType<typeof setTimeout> | undefined;
  const stop = () => {
    stopped = true;
    if (retry) clearTimeout(retry);
    const current = socket;
    socket = undefined;
    if (!current) return;
    current.onmessage = null;
    current.onclose = null;
    if (current.readyState === WebSocket.CONNECTING) {
      current.onopen = () => current.close();
    } else if (current.readyState === WebSocket.OPEN) {
      current.close();
    }
  };
  window.addEventListener("pagehide", stop);
  const connect = () => {
    const scheme = location.protocol === "https:" ? "wss" : "ws";
    socket = new WebSocket(`${scheme}://${location.host}/ws/${namespace}`);
    socket.onmessage = (event) => update(JSON.parse(event.data) as Snapshot);
    socket.onclose = () => {
      if (!stopped) retry = setTimeout(connect, 1000);
    };
  };
  connect();
  return () => {
    window.removeEventListener("pagehide", stop);
    stop();
  };
}

export function useGateway(namespace: Namespace) {
  const [snapshot, setSnapshot] = useState<Snapshot>();
  const [transportError, setTransportError] = useState<string>();
  const [operationError, setOperationError] = useState<string>();
  const pending = useRef<Snapshot | undefined>(undefined);
  useEffect(() => {
    let frame: number | undefined;
    const applyPending = () => {
      frame = undefined;
      if (!pending.current) return;
      if (window.getSelection()?.toString()) {
        frame = window.requestAnimationFrame(applyPending);
        return;
      }
      const value = pending.current;
      pending.current = undefined;
      setSnapshot(value);
      setTransportError(undefined);
    };
    const accept = (value: Snapshot) => {
      pending.current = value;
      if (frame === undefined) {
        frame = window.requestAnimationFrame(applyPending);
      }
    };
    const dispose = subscribe(namespace, accept);
    getSnapshot(namespace)
      .then(accept)
      .catch((reason: unknown) =>
        setTransportError(
          reason instanceof Error ? reason.message : String(reason),
        ),
      );
    return () => {
      if (frame !== undefined) window.cancelAnimationFrame(frame);
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
