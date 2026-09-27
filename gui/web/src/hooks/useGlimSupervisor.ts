import {useCallback, useEffect, useState} from "react";
import {useWS} from "./useWS.ts";

/**
 * glim_supervisor (3d_mowgli_slam_stack, glim/supervisor/glim_supervisor.py):
 * the GLIM container runs permanently and the supervisor starts GLIM mapping,
 * the offline viewer or a map export on demand — one at a time. Its status is
 * a latched std_msgs/String whose `data` is JSON (see Supervisor.status()).
 */
export type GlimState = "idle" | "mapping" | "saving" | "viewer" | "exporting" | "error";

export interface GlimSession {
    name: string;
    /** "mapping" = recorded by the supervisor; "saved" = e.g. Save Map in the offline viewer. */
    kind: string;
    /** A complete GLIM dump (graph.bin present) — only these can be opened or exported. */
    complete: boolean;
    started_at?: string | null;
    finished_at?: string | null;
    duration_s?: number | null;
    size_bytes: number;
}

export interface GlimActiveMap {
    path: string;
    exists: boolean;
    size_bytes?: number;
    modified_at?: string;
    source_session?: string;
    exported_at?: string;
    backups: string[];
}

export interface GlimSupervisorStatus {
    state: string;
    message: string;
    session: string | null;
    started_at: string | null;
    elapsed_s: number | null;
    sessions_dir: string;
    sessions: GlimSession[];
    active_map: GlimActiveMap;
    log_tail: string[];
}

/** Parse the std_msgs/String payload; tolerant of either field casing. */
export function parseGlimStatus(raw: unknown): GlimSupervisorStatus | null {
    const msg = raw as {data?: unknown; Data?: unknown} | null;
    const text = msg?.data ?? msg?.Data;
    if (typeof text !== "string" || text.length === 0) return null;
    try {
        const parsed = JSON.parse(text) as unknown;
        if (parsed === null || typeof parsed !== "object") return null;
        const st = parsed as Partial<GlimSupervisorStatus>;
        if (typeof st.state !== "string") return null;
        return {
            state: st.state,
            message: typeof st.message === "string" ? st.message : "",
            session: st.session ?? null,
            started_at: st.started_at ?? null,
            elapsed_s: typeof st.elapsed_s === "number" ? st.elapsed_s : null,
            sessions_dir: st.sessions_dir ?? "",
            sessions: Array.isArray(st.sessions) ? st.sessions : [],
            active_map: st.active_map ?? {path: "", exists: false, backups: []},
            log_tail: Array.isArray(st.log_tail) ? st.log_tail : [],
        };
    } catch {
        return null;
    }
}

export function formatBytes(bytes: number | undefined): string {
    if (bytes === undefined || !Number.isFinite(bytes)) return "—";
    if (bytes >= 1024 ** 3) return `${(bytes / 1024 ** 3).toFixed(2)} GB`;
    if (bytes >= 1024 ** 2) return `${(bytes / 1024 ** 2).toFixed(1)} MB`;
    return `${Math.max(1, Math.round(bytes / 1024))} kB`;
}

export function formatDuration(seconds: number | null | undefined): string {
    if (seconds === null || seconds === undefined || !Number.isFinite(seconds)) return "—";
    const s = Math.max(0, Math.round(seconds));
    const h = Math.floor(s / 3600);
    const m = Math.floor((s % 3600) / 60);
    const sec = s % 60;
    return h > 0 ? `${h}:${String(m).padStart(2, "0")}:${String(sec).padStart(2, "0")}` : `${m}:${String(sec).padStart(2, "0")}`;
}

type TriggerResponse = {success?: boolean; message?: string; error?: string};

async function post(path: string, body?: unknown): Promise<TriggerResponse> {
    const res = await fetch(`/api/glim/${path}`, {
        method: "POST",
        headers: body ? {"Content-Type": "application/json"} : undefined,
        body: body ? JSON.stringify(body) : undefined,
    });
    let parsed: TriggerResponse = {};
    try {
        parsed = (await res.json()) as TriggerResponse;
    } catch {
        // non-JSON error body
    }
    if (!res.ok) throw new Error(parsed.error || `HTTP ${res.status}`);
    return parsed;
}

export const useGlimSupervisor = () => {
    const [status, setStatus] = useState<GlimSupervisorStatus | null>(null);
    const [receivedAt, setReceivedAt] = useState<number | null>(null);
    const [busy, setBusy] = useState(false);
    const [actionError, setActionError] = useState<string | null>(null);

    const stream = useWS<string>(
        () => {},
        () => {},
        (e) => {
            const parsed = parseGlimStatus(e);
            if (parsed) {
                setStatus(parsed);
                setReceivedAt(Date.now());
            }
        },
    );

    useEffect(() => {
        stream.start("/api/mowglinext/subscribe/glimSupervisorStatus");
        return () => stream.stop();
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    const run = useCallback(async (path: string, body?: unknown): Promise<boolean> => {
        setBusy(true);
        setActionError(null);
        try {
            const res = await post(path, body);
            if (!res.success) {
                setActionError(res.message || path);
                return false;
            }
            return true;
        } catch (e: unknown) {
            setActionError(e instanceof Error ? e.message : String(e));
            return false;
        } finally {
            setBusy(false);
        }
    }, []);

    return {
        status,
        receivedAt,
        busy,
        actionError,
        startMapping: useCallback(() => run("start-mapping"), [run]),
        stopMapping: useCallback(() => run("stop-mapping"), [run]),
        closeViewer: useCallback(() => run("close-viewer"), [run]),
        openViewer: useCallback((session: string) => run("open-viewer", {session}), [run]),
        exportMap: useCallback((session: string) => run("export-map", {session}), [run]),
    };
};
