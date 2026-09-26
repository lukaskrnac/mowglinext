import {useCallback, useEffect, useRef, useState} from "react";

/**
 * GPS ↔ LiDAR primary localization source (fusion_graph_node
 * `primary_localization_source`, see fusion_graph_node_lidar_primary.cpp).
 *
 * The switch is applied LIVE (ros2 param set through the foxglove bridge,
 * `POST /api/params`) and then persisted to mowgli_robot.yaml
 * (`POST /api/settings/yaml`), which fusion_graph.launch.py injects on the
 * next start. fusion_graph refuses "lidar" without a valid lidar_map → map
 * calibration (`lidar_pose_map_calibrated`); the bridge then echoes the
 * unchanged value, which is how a refusal is detected here — nothing is
 * persisted in that case.
 */
export type LocalizationSource = "gps" | "lidar";

export const FUSION_GRAPH_NODE = "fusion_graph_node";
export const SOURCE_PARAM = "primary_localization_source";
export const CALIBRATED_PARAM = "lidar_pose_map_calibrated";
export const SOURCE_YAML_KEY = "primary_localization_source";

const POLL_MS = 15_000;

type RosParameter = { name: string; value: unknown; type?: string };
type ParamsBody = { parameters?: RosParameter[]; error?: string };

async function readJson<T>(res: Response): Promise<T> {
    try {
        return (await res.json()) as T;
    } catch {
        return {} as T;
    }
}

/** Bridges differ on whether names carry a leading "/"; match on the suffix. */
export function findParam(params: RosParameter[], param: string): RosParameter | undefined {
    const suffix = `${FUSION_GRAPH_NODE}.${param}`;
    return params.find((p) => p.name === suffix || p.name.endsWith(`/${suffix}`));
}

export function parseSource(value: unknown): LocalizationSource | null {
    return value === "gps" || value === "lidar" ? value : null;
}

export function parseBool(value: unknown): boolean | null {
    if (value === true || value === "true") return true;
    if (value === false || value === "false") return false;
    return null;
}

async function fetchParams(names: string[] | null): Promise<RosParameter[]> {
    const query = names ? `?names=${encodeURIComponent(names.join(","))}` : "";
    const res = await fetch(`/api/params${query}`);
    const body = await readJson<ParamsBody>(res);
    if (!res.ok) {
        throw new Error(body.error || `HTTP ${res.status}`);
    }
    return body.parameters ?? [];
}

/** Maps the hook's error codes to a translated description. */
export function localizationSourceErrorText(
    error: string,
    t: (key: string, opts?: Record<string, unknown>) => string,
): string {
    if (error === "rejected-lidar") return t("localizationSource.rejectedLidar");
    if (error === "rejected") return t("localizationSource.rejected");
    if (error.startsWith("persist:")) {
        return t("localizationSource.persistFailed", {error: error.slice("persist:".length)});
    }
    return error;
}

export interface LocalizationSourceState {
    /** Live value on fusion_graph_node; null = unknown / node not reachable. */
    source: LocalizationSource | null;
    /** lidar_pose_map_calibrated on fusion_graph_node; null = unknown. */
    calibrated: boolean | null;
    loading: boolean;
    switching: boolean;
    error: string | null;
    refresh: () => Promise<void>;
    setSource: (next: LocalizationSource) => Promise<boolean>;
}

export const useLocalizationSource = (): LocalizationSourceState => {
    const [source, setSourceState] = useState<LocalizationSource | null>(null);
    const [calibrated, setCalibrated] = useState<boolean | null>(null);
    const [loading, setLoading] = useState(true);
    const [switching, setSwitching] = useState(false);
    const [error, setError] = useState<string | null>(null);
    // Fully-qualified name as the bridge reports it — reused for the set so a
    // "/"-prefixed bridge and an unprefixed one both work.
    const sourceNameRef = useRef(`${FUSION_GRAPH_NODE}.${SOURCE_PARAM}`);
    const mountedRef = useRef(true);
    const fullListingTriedRef = useRef(false);

    const refresh = useCallback(async () => {
        try {
            // Query by the name form the bridge last reported (prefix included).
            const prefix = sourceNameRef.current.slice(0, -SOURCE_PARAM.length);
            let params = await fetchParams([SOURCE_PARAM, CALIBRATED_PARAM].map((p) => prefix + p));
            // A bridge that wants a different naming form answers a by-name
            // query with nothing; fall back ONCE to the (slow) full listing to
            // learn the form, never on every poll.
            if (!findParam(params, SOURCE_PARAM) && !fullListingTriedRef.current) {
                fullListingTriedRef.current = true;
                params = await fetchParams(null);
            }
            if (!mountedRef.current) return;
            const src = findParam(params, SOURCE_PARAM);
            const cal = findParam(params, CALIBRATED_PARAM);
            if (src) sourceNameRef.current = src.name;
            setSourceState(src ? parseSource(src.value) : null);
            setCalibrated(cal ? parseBool(cal.value) : null);
            setError(src ? null : "fusion_graph_node does not report primary_localization_source");
        } catch (e: unknown) {
            if (!mountedRef.current) return;
            setError(e instanceof Error ? e.message : String(e));
        } finally {
            if (mountedRef.current) setLoading(false);
        }
    }, []);

    useEffect(() => {
        mountedRef.current = true;
        void refresh();
        const id = window.setInterval(() => void refresh(), POLL_MS);
        return () => {
            mountedRef.current = false;
            window.clearInterval(id);
        };
    }, [refresh]);

    const setSource = useCallback(async (next: LocalizationSource): Promise<boolean> => {
        setSwitching(true);
        setError(null);
        try {
            const res = await fetch("/api/params", {
                method: "POST",
                headers: {"Content-Type": "application/json"},
                body: JSON.stringify({parameters: [{name: sourceNameRef.current, value: next}]}),
            });
            const body = await readJson<ParamsBody>(res);
            if (!res.ok) {
                throw new Error(body.error || `HTTP ${res.status}`);
            }
            // The bridge echoes the value the node actually holds after the
            // set; a refusal (e.g. "lidar" without calibration) echoes the old
            // value. Confirm with a fresh read when the echo is inconclusive.
            let applied = parseSource(findParam(body.parameters ?? [], SOURCE_PARAM)?.value);
            if (applied === null) {
                const params = await fetchParams([sourceNameRef.current]);
                applied = parseSource(findParam(params, SOURCE_PARAM)?.value);
            }
            if (applied !== next) {
                throw new Error(next === "lidar" ? "rejected-lidar" : "rejected");
            }
            setSourceState(next);

            // Persist for the next start. Sparse config: "gps" equals the
            // schema default and is pruned, which falls back to the same
            // default in fusion_graph.yaml.
            const persist = await fetch("/api/settings/yaml", {
                method: "POST",
                headers: {"Content-Type": "application/json"},
                body: JSON.stringify({[SOURCE_YAML_KEY]: next}),
            });
            if (!persist.ok) {
                const pb = await readJson<{error?: string}>(persist);
                throw new Error(`persist:${pb.error || `HTTP ${persist.status}`}`);
            }
            return true;
        } catch (e: unknown) {
            setError(e instanceof Error ? e.message : String(e));
            return false;
        } finally {
            if (mountedRef.current) setSwitching(false);
            void refresh();
        }
    }, [refresh]);

    return {source, calibrated, loading, switching, error, refresh, setSource};
};
