import {
  schemaVersion,
  type Json,
  type Namespace,
  type Snapshot,
} from "@robot/contracts";
import { nanoid } from "nanoid/non-secure";
import { useEffect, useState } from "react";

export function requestId(): string {
  return nanoid();
}

// A local edit is not live state. Once the server echoes it, release the draft
// so later changes from another page/client remain visible. Failed saves keep it.
export function useDraftValue<T>(saved: T) {
  const [draft, setDraft] = useState<{ value: T }>();
  const acknowledged =
    draft !== undefined &&
    JSON.stringify(draft.value) === JSON.stringify(saved);
  if (acknowledged) setDraft(undefined);
  const value = draft && !acknowledged ? draft.value : saved;
  return [
    value,
    (next: T | undefined) =>
      setDraft(next === undefined ? undefined : { value: next }),
  ] as const;
}

export async function post<T extends Json>(
  path: string,
  value: T,
  options?: Pick<RequestInit, "keepalive">,
): Promise<Json> {
  const response = await fetch(path, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify(value),
    ...options,
  });
  return responseValue(response);
}

export function prepareRelativeControl(): Promise<Json> {
  return post("/api/motion/prepare-relative", {
    schema_version: schemaVersion,
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
  if (!response.ok) {
    const text = await response.text();
    let message = text;
    try {
      const error = JSON.parse(text) as { original_error?: unknown };
      if (typeof error.original_error === "string")
        message = error.original_error;
    } catch {
      // A proxy may return text/HTML instead of the gateway's JSON envelope.
    }
    throw new Error(message || `HTTP ${response.status}`);
  }
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
  connectionChanged?: (state: ConnectionState) => void,
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
    socket.onopen = () => connectionChanged?.("connected");
    socket.onmessage = (event) => {
      update(JSON.parse(event.data) as Snapshot);
      socket?.send("next");
    };
    socket.onclose = () => {
      if (!stopped) {
        connectionChanged?.("disconnected");
        retry = setTimeout(connect, 1000);
      }
    };
  };
  connect();
  return () => {
    window.removeEventListener("pagehide", stop);
    stop();
  };
}

export type ConnectionState = "connecting" | "connected" | "disconnected";

export function useGateway(namespace: Namespace) {
  const [connection, setConnection] = useState<ConnectionState>("connecting");
  const [snapshot, setSnapshot] = useState<Snapshot>();
  const [operationError, setOperationError] = useState<string>();
  useEffect(() => {
    let pending: Snapshot | undefined;
    let frame: number | undefined;
    const applyPending = () => {
      frame = undefined;
      if (!pending) return;
      const value = pending;
      pending = undefined;
      setSnapshot(value);
    };
    const accept = (value: Snapshot) => {
      pending = value;
      if (frame === undefined) {
        frame = window.requestAnimationFrame(applyPending);
      }
    };
    const dispose = subscribe(namespace, accept, setConnection);
    return () => {
      if (frame !== undefined) window.cancelAnimationFrame(frame);
      pending = undefined;
      dispose();
    };
  }, [namespace]);
  return {
    connection,
    snapshot,
    error: operationError,
    setError: setOperationError,
  };
}
