import React from "react";
import {Alert, App, Button, Card, Collapse, Descriptions, Space, Table, Tag, Typography} from "antd";
import {RadarChartOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {
    formatBytes,
    formatDuration,
    type GlimSession,
    useGlimSupervisor,
} from "../hooks/useGlimSupervisor.ts";

const {Text, Paragraph} = Typography;

const STATE_COLORS: Record<string, string> = {
    idle: "default",
    mapping: "processing",
    saving: "processing",
    viewer: "blue",
    exporting: "processing",
    error: "error",
};

const formatWhen = (iso?: string | null) => {
    if (!iso) return "—";
    const d = new Date(iso);
    return Number.isNaN(d.getTime()) ? iso : d.toLocaleString();
};

/**
 * Diagnostics → Calibration: GLIM mapping via glim_supervisor. Start/stop a
 * mapping run (each run gets its own session directory), open a session in
 * the offline viewer (on the robot's display — reach it with remote desktop),
 * and export a session as the active map lidar_localization loads.
 */
export const GlimMappingCard: React.FC = () => {
    const {t} = useTranslation();
    const {modal} = App.useApp();
    const {status, busy, actionError, startMapping, stopMapping, closeViewer, openViewer, exportMap} =
        useGlimSupervisor();

    const state = status?.state ?? "unknown";
    const idle = state === "idle" || state === "error";
    const activeMap = status?.active_map;

    const confirmStop = () => modal.confirm({
        title: t("glimMapping.stopTitle"),
        content: t("glimMapping.stopBody"),
        okText: t("glimMapping.stop"),
        cancelText: t("glimMapping.cancel"),
        onOk: () => stopMapping(),
    });

    const confirmCloseViewer = () => modal.confirm({
        title: t("glimMapping.closeViewerTitle"),
        content: t("glimMapping.closeViewerBody"),
        okText: t("glimMapping.closeViewer"),
        okButtonProps: {danger: true},
        cancelText: t("glimMapping.cancel"),
        onOk: () => closeViewer(),
    });

    const confirmExport = (session: string) => modal.confirm({
        title: t("glimMapping.exportTitle", {session}),
        content: (
            <Space direction="vertical" size={4}>
                <Text>{t("glimMapping.exportBody")}</Text>
                <Text type="secondary" style={{fontSize: 12}}>{t("glimMapping.exportAfter")}</Text>
            </Space>
        ),
        okText: t("glimMapping.export"),
        cancelText: t("glimMapping.cancel"),
        onOk: () => exportMap(session),
    });

    const columns = [
        {
            title: t("glimMapping.colSession"),
            dataIndex: "name",
            key: "name",
            render: (name: string, s: GlimSession) => (
                <Space direction="vertical" size={0}>
                    <Text code style={{fontSize: 12}}>{name}</Text>
                    <Space size={4} wrap>
                        <Tag color={s.kind === "mapping" ? "default" : "purple"} style={{fontSize: 11}}>
                            {s.kind === "mapping" ? t("glimMapping.kindMapping") : t("glimMapping.kindSaved")}
                        </Tag>
                        {!s.complete && <Tag color="warning" style={{fontSize: 11}}>{t("glimMapping.incomplete")}</Tag>}
                        {activeMap?.source_session === name && (
                            <Tag color="success" style={{fontSize: 11}}>{t("glimMapping.activeTag")}</Tag>
                        )}
                    </Space>
                </Space>
            ),
        },
        {
            title: t("glimMapping.colWhen"),
            key: "when",
            responsive: ["md" as const],
            render: (_: unknown, s: GlimSession) => (
                <Space direction="vertical" size={0}>
                    <Text style={{fontSize: 12}}>{formatWhen(s.started_at ?? s.finished_at)}</Text>
                    {s.duration_s ? (
                        <Text type="secondary" style={{fontSize: 11}}>{formatDuration(s.duration_s)}</Text>
                    ) : null}
                </Space>
            ),
        },
        {
            title: t("glimMapping.colSize"),
            dataIndex: "size_bytes",
            key: "size",
            responsive: ["sm" as const],
            render: (b: number) => <Text style={{fontSize: 12}}>{formatBytes(b)}</Text>,
        },
        {
            title: "",
            key: "actions",
            render: (_: unknown, s: GlimSession) => (
                <Space size={4} wrap>
                    <Button size="small" disabled={!idle || !s.complete || busy} onClick={() => void openViewer(s.name)}>
                        {t("glimMapping.openViewer")}
                    </Button>
                    <Button size="small" type="primary" ghost disabled={!idle || !s.complete || busy}
                            onClick={() => { confirmExport(s.name); }}>
                        {t("glimMapping.useAsMap")}
                    </Button>
                </Space>
            ),
        },
    ];

    let primary: React.ReactNode = null;
    if (state === "mapping") {
        primary = <Button danger type="primary" loading={busy} onClick={() => { confirmStop(); }}>{t("glimMapping.stop")}</Button>;
    } else if (state === "viewer") {
        primary = <Button danger loading={busy} onClick={() => { confirmCloseViewer(); }}>{t("glimMapping.closeViewer")}</Button>;
    } else if (idle) {
        primary = <Button type="primary" loading={busy} onClick={() => void startMapping()}>{t("glimMapping.start")}</Button>;
    }

    return (
        <Card
            size="small"
            title={<Space><RadarChartOutlined/> {t("glimMapping.title")}</Space>}
            extra={status ? (
                <Tag color={STATE_COLORS[state] ?? "default"}>
                    {t(`glimMapping.state.${state}`, {defaultValue: state})}
                </Tag>
            ) : null}
        >
            {!status ? (
                <Alert type="info" showIcon message={t("glimMapping.noSupervisor")}/>
            ) : (
                <Space direction="vertical" size={10} style={{width: "100%"}}>
                    <Space wrap align="center">
                        {primary}
                        {(state === "mapping" || state === "saving" || state === "exporting" || state === "viewer") && (
                            <Text type="secondary" style={{fontSize: 12}}>
                                {status.session ? <Text code>{status.session}</Text> : null}{" "}
                                {formatDuration(status.elapsed_s)}
                            </Text>
                        )}
                    </Space>
                    {status.message && (
                        <Text type={state === "error" ? "danger" : "secondary"} style={{fontSize: 12}}>{status.message}</Text>
                    )}
                    {actionError && <Alert type="error" showIcon message={actionError}/>}
                    {state === "mapping" && (
                        <Alert type="info" showIcon message={t("glimMapping.mappingHint")}/>
                    )}
                    {state === "saving" && (
                        <Alert type="warning" showIcon message={t("glimMapping.savingHint")}/>
                    )}
                    {state === "viewer" && (
                        <Alert
                            type="info"
                            showIcon
                            message={t("glimMapping.viewerHint", {dir: status.sessions_dir})}
                            description={status.save_target ? (
                                <span>
                                    {t("glimMapping.saveTargetPrefix")} <Text code copyable>{status.save_target}</Text>
                                    {" "}{t("glimMapping.saveTargetSuffix")}
                                </span>
                            ) : undefined}
                        />
                    )}

                    <Descriptions size="small" column={1} title={<Text style={{fontSize: 12}}>{t("glimMapping.activeMap")}</Text>}>
                        <Descriptions.Item label={t("glimMapping.file")}>
                            <Text code style={{fontSize: 12}}>{activeMap?.path || "—"}</Text>
                            {!activeMap?.exists && <Tag color="warning" style={{marginLeft: 6}}>{t("glimMapping.missing")}</Tag>}
                        </Descriptions.Item>
                        {activeMap?.exists && (
                            <>
                                <Descriptions.Item label={t("glimMapping.source")}>
                                    {activeMap.source_session ? <Text code>{activeMap.source_session}</Text> : t("glimMapping.sourceUnknown")}
                                </Descriptions.Item>
                                <Descriptions.Item label={t("glimMapping.modified")}>
                                    {formatWhen(activeMap.exported_at ?? activeMap.modified_at)} · {formatBytes(activeMap.size_bytes)}
                                </Descriptions.Item>
                            </>
                        )}
                        <Descriptions.Item label={t("glimMapping.backups")}>
                            {activeMap?.backups.length ? activeMap.backups.join(", ") : "—"}
                        </Descriptions.Item>
                    </Descriptions>

                    <div>
                        <Text strong style={{fontSize: 12}}>{t("glimMapping.sessions")}</Text>
                        <Paragraph type="secondary" style={{fontSize: 11, margin: "2px 0 6px"}}>
                            {t("glimMapping.sessionsHint", {dir: status.sessions_dir})}
                        </Paragraph>
                        <Table<GlimSession>
                            size="small"
                            rowKey="name"
                            pagination={status.sessions.length > 8 ? {pageSize: 8, size: "small"} : false}
                            dataSource={status.sessions}
                            columns={columns}
                            locale={{emptyText: t("glimMapping.noSessions")}}
                        />
                    </div>

                    {status.log_tail.length > 0 && (
                        <Collapse
                            size="small"
                            defaultActiveKey={state === "error" ? ["log"] : []}
                            items={[{
                                key: "log",
                                label: t("glimMapping.log"),
                                children: (
                                    <pre style={{fontSize: 11, margin: 0, whiteSpace: "pre-wrap", maxHeight: 220, overflow: "auto"}}>
                                        {status.log_tail.join("\n")}
                                    </pre>
                                ),
                            }]}
                        />
                    )}
                </Space>
            )}
        </Card>
    );
};
