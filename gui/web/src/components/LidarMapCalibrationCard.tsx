import React from "react";
import {Alert, App, Button, Card, Col, Descriptions, Progress, Row, Space, Tag, Typography} from "antd";
import {AimOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {
    SPREAD_TARGET_MAJOR_M,
    SPREAD_TARGET_MINOR_M,
    useLidarMapCalibration,
} from "../hooks/useLidarMapCalibration.ts";
import type {LidarMapCalibrationFileStatus} from "../hooks/useCalibrationStatus.ts";
import {LocalizationSourceSwitch} from "./LocalizationSourceSwitch.tsx";

const {Text, Paragraph} = Typography;

type Props = {
    /** Saved result from GET /calibration/status (lidar_map_calibration.yaml). */
    saved?: LidarMapCalibrationFileStatus;
    /** Current datum from settings — flags a calibration made for another datum. */
    datumLat?: number;
    datumLon?: number;
    lidarEnabled: boolean;
    /** Re-poll /calibration/status (the file changes when a run succeeds). */
    onFinished?: () => void;
};

const STATE_COLORS: Record<string, string> = {
    idle: "default",
    collecting: "processing",
    succeeded: "success",
    failed: "error",
    canceled: "warning",
};

const pct = (value: number, target: number) =>
    target > 0 ? Math.max(0, Math.min(100, Math.round((value / target) * 100))) : 0;

// fusion_graph.launch.py drops a calibration whose datum differs by > 1e-7°.
const DATUM_TOLERANCE_DEG = 1e-7;

/**
 * Diagnostics → Calibration: LiDAR map (lidar_map → map) calibration driven by
 * calibrate_lidar_map_node. Start/cancel go through std_srvs/Trigger services;
 * progress streams from ~/status. The operator drives the robot meanwhile.
 */
export const LidarMapCalibrationCard: React.FC<Props> = ({saved, datumLat, datumLon, lidarEnabled, onFinished}) => {
    const {t} = useTranslation();
    const {modal} = App.useApp();
    const {status, busy, actionError, start, cancel, collecting} = useLidarMapCalibration();

    const prevStateRef = React.useRef<string | undefined>(undefined);
    React.useEffect(() => {
        const prev = prevStateRef.current;
        prevStateRef.current = status?.state;
        if (prev === "collecting" && status?.state !== "collecting") onFinished?.();
    }, [status?.state, onFinished]);

    const datumMismatch =
        !!saved?.present && !saved.error &&
        datumLat !== undefined && datumLon !== undefined &&
        saved.datum_lat !== undefined && saved.datum_lon !== undefined &&
        (Math.abs(saved.datum_lat - datumLat) > DATUM_TOLERANCE_DEG ||
            Math.abs(saved.datum_lon - datumLon) > DATUM_TOLERANCE_DEG);

    const confirmStart = () => {
        modal.confirm({
            title: t("lidarMapCalibration.confirmTitle"),
            content: (
                <Space direction="vertical" size={4}>
                    <Text>{t("lidarMapCalibration.confirmBody")}</Text>
                    <Text type="secondary" style={{fontSize: 12}}>{t("lidarMapCalibration.confirmRequirements")}</Text>
                </Space>
            ),
            okText: t("lidarMapCalibration.start"),
            cancelText: t("lidarMapCalibration.cancelDialog"),
            onOk: () => start(),
        });
    };

    const state = status?.state ?? "idle";
    const savedTag = saved?.present
        ? datumMismatch
            ? <Tag color="warning">{t("lidarMapCalibration.staleDatum")}</Tag>
            : <Tag color="success">{t("diagnosticsPage.present")}</Tag>
        : <Tag color="warning">{t("diagnosticsPage.missing")}</Tag>;

    const rejected = Object.entries(status?.rejected ?? {}).filter(([, n]) => n > 0);

    return (
        <Card
            title={<Space><AimOutlined/> {t("lidarMapCalibration.title")}</Space>}
            size="small"
            extra={savedTag}
            actions={collecting ? [
                <Button key="cancel" size="small" type="link" danger loading={busy} onClick={() => void cancel()}>
                    {t("lidarMapCalibration.cancel")}
                </Button>,
            ] : [
                <Button key="run" size="small" type="link" loading={busy} disabled={!lidarEnabled} onClick={confirmStart}>
                    {saved?.present ? t("lidarMapCalibration.rerun") : t("lidarMapCalibration.start")}
                </Button>,
            ]}
        >
            <Row gutter={[16, 12]}>
                <Col xs={24} lg={12}>
                    <Paragraph type="secondary" style={{marginTop: 0, fontSize: 12}}>
                        {t("lidarMapCalibration.intro")}
                    </Paragraph>
                    {!lidarEnabled && (
                        <Alert type="info" showIcon style={{marginBottom: 8}} message={t("lidarMapCalibration.lidarDisabled")}/>
                    )}
                    {actionError && (
                        <Alert type="error" showIcon style={{marginBottom: 8}} message={actionError}/>
                    )}
                    {status?.warning && (
                        <Alert type="warning" showIcon style={{marginBottom: 8}} message={status.warning}/>
                    )}
                    {datumMismatch && (
                        <Alert type="warning" showIcon style={{marginBottom: 8}} message={t("lidarMapCalibration.staleDatumDetail")}/>
                    )}

                    {status && (
                        <Space direction="vertical" size={6} style={{width: "100%"}}>
                            <Space wrap>
                                <Tag color={STATE_COLORS[state] ?? "default"}>
                                    {t(`lidarMapCalibration.state.${state}`, {defaultValue: state})}
                                </Tag>
                                <Tag color={status.lidar_healthy ? "success" : "error"}>
                                    {status.lidar_healthy ? t("lidarMapCalibration.lidarHealthy") : t("lidarMapCalibration.lidarUnhealthy")}
                                </Tag>
                                {status.gps_rtk_fixed !== undefined && (
                                    <Tag color={status.gps_rtk_fixed ? "success" : "warning"}>
                                        {status.gps_rtk_fixed ? "RTK Fixed" : t("lidarMapCalibration.noRtkFixed")}
                                        {typeof status.gps_accuracy_m === "number" ? ` · ${(status.gps_accuracy_m * 100).toFixed(1)} cm` : ""}
                                    </Tag>
                                )}
                            </Space>
                            {status.message && <Text style={{fontSize: 12}}>{status.message}</Text>}
                            {(collecting || state === "succeeded" || state === "failed") && (
                                <>
                                    <div>
                                        <Text type="secondary" style={{fontSize: 11}}>
                                            {t("lidarMapCalibration.pairs", {pairs: status.pairs, min: status.min_pairs})}
                                        </Text>
                                        <Progress size="small" percent={pct(status.pairs, status.min_pairs)} showInfo={false}/>
                                    </div>
                                    <div>
                                        <Text type="secondary" style={{fontSize: 11}}>
                                            {t("lidarMapCalibration.spread", {
                                                major: status.spread_major_std_m.toFixed(2),
                                                minor: status.spread_minor_std_m.toFixed(2),
                                                majorTarget: SPREAD_TARGET_MAJOR_M.toFixed(1),
                                                minorTarget: SPREAD_TARGET_MINOR_M.toFixed(1),
                                            })}
                                        </Text>
                                        <Progress
                                            size="small"
                                            showInfo={false}
                                            percent={Math.min(
                                                pct(status.spread_major_std_m, SPREAD_TARGET_MAJOR_M),
                                                pct(status.spread_minor_std_m, SPREAD_TARGET_MINOR_M),
                                            )}
                                        />
                                    </div>
                                </>
                            )}
                            {rejected.length > 0 && (
                                <Text type="secondary" style={{fontSize: 11}}>
                                    {t("lidarMapCalibration.rejected")}{" "}
                                    {rejected.map(([k, n]) => `${k}: ${n}`).join(", ")}
                                </Text>
                            )}
                        </Space>
                    )}
                </Col>
                <Col xs={24} lg={12}>
                    {status?.yaw_deg !== undefined && collecting && (
                        <Descriptions size="small" column={1} title={<Text style={{fontSize: 12}}>{t("lidarMapCalibration.currentFit")}</Text>}>
                            <Descriptions.Item label={t("diagnosticsPage.yaw")}>{status.yaw_deg.toFixed(2)}°</Descriptions.Item>
                            <Descriptions.Item label={t("lidarMapCalibration.offset")}>
                                ({status.x_m?.toFixed(3)}, {status.y_m?.toFixed(3)}) m
                            </Descriptions.Item>
                            <Descriptions.Item label="RMS">{status.rms_cm?.toFixed(1)} cm</Descriptions.Item>
                        </Descriptions>
                    )}
                    {saved?.present && !saved.error ? (
                        <Descriptions size="small" column={1} title={<Text style={{fontSize: 12}}>{t("lidarMapCalibration.saved")}</Text>}>
                            <Descriptions.Item label={t("diagnosticsPage.calibratedAt")}>{saved.calibrated_at ?? "—"}</Descriptions.Item>
                            <Descriptions.Item label={t("diagnosticsPage.yaw")}>{saved.yaw_deg?.toFixed(2)}°</Descriptions.Item>
                            <Descriptions.Item label={t("lidarMapCalibration.offset")}>
                                ({saved.x?.toFixed(3)}, {saved.y?.toFixed(3)}) m
                            </Descriptions.Item>
                            <Descriptions.Item label="RMS">
                                {saved.rms_m !== undefined ? `${(saved.rms_m * 100).toFixed(1)} cm` : "—"}
                            </Descriptions.Item>
                            <Descriptions.Item label={t("lidarMapCalibration.pairsLabel")}>
                                {saved.inliers ?? "—"} / {saved.pairs ?? "—"}
                            </Descriptions.Item>
                        </Descriptions>
                    ) : saved?.error ? (
                        <Alert type="error" showIcon message={saved.error}/>
                    ) : (
                        <Text type="secondary" style={{fontSize: 12}}>{t("lidarMapCalibration.none")}</Text>
                    )}
                    <div style={{marginTop: 12}}>
                        <Text strong style={{fontSize: 12, display: "block", marginBottom: 6}}>
                            {t("localizationSource.title")}
                        </Text>
                        <LocalizationSourceSwitch/>
                    </div>
                </Col>
            </Row>
        </Card>
    );
};
