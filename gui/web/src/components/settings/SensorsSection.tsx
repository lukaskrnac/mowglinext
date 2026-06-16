import React from "react";
import { Card, Col, Form, InputNumber, Row, Switch, Typography } from "antd";
import { AimOutlined, RadarChartOutlined } from "@ant-design/icons";
import { RobotComponentEditor } from "../RobotComponentEditor.tsx";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

export const SensorsSection: React.FC<Props> = ({ values, onChange }) => {
    // fusion_graph is the sole localizer and always runs (the use_fusion_graph
    // launch flag was removed), so the LiDAR toggle drives the scan-factor
    // gates that ARE consumed by fusion_graph.launch.py: use_scan_matching and
    // use_loop_closure. With no LiDAR there are no scans to match, so both are
    // forced off. Operators can still fine-tune them in the Localization tab.
    const handleLidarToggle = (enabled: boolean) => {
        onChange("lidar_enabled", enabled);
        onChange("use_scan_matching", enabled);
        onChange("use_loop_closure", enabled);
    };

    return (
        <div>
            {/* LiDAR toggle */}
            <Card size="small" style={{ marginBottom: 16 }}>
                <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <RadarChartOutlined style={{ marginRight: 6 }} />
                            LiDAR Sensor
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            Enable if your robot has a LiDAR. Also flips
                            {" "}<Text code>use_scan_matching</Text> and{" "}
                            <Text code>use_loop_closure</Text> so the factor graph fuses LiDAR
                            scans. Fine-tune those in the Localization tab.
                        </Paragraph>
                    </div>
                    <Switch
                        checked={values.lidar_enabled ?? false}
                        onChange={handleLidarToggle}
                    />
                </div>
            </Card>

            {/* Sensor placement visual editor */}
            <RobotComponentEditor values={values} onChange={onChange} />

            {/* IMU bias calibration (hardware_bridge_node, auto-triggered on dock) */}
            <Card size="small" style={{ marginTop: 16 }} title={
                <Text strong style={{ fontSize: 14 }}>
                    <AimOutlined style={{ marginRight: 6 }} />
                    IMU bias calibration
                </Text>
            }>
                <Paragraph type="secondary" style={{ margin: "0 0 12px", fontSize: 12 }}>
                    The hardware bridge re-estimates IMU gyro/accel bias automatically while the
                    robot is parked and stationary on the dock. These tune how it samples.
                </Paragraph>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={24} sm={8}>
                            <Form.Item label="Calibration samples" tooltip="Number of stationary IMU samples averaged per bias calibration.">
                                <InputNumber
                                    value={values.imu_cal_samples}
                                    onChange={(v) => onChange("imu_cal_samples", v)}
                                    min={50} max={2000} step={50} precision={0}
                                    style={{ width: "100%" }}
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={24} sm={8}>
                            <Form.Item label="Rest window before cal" tooltip="How long the robot must be detected stationary before a calibration starts.">
                                <InputNumber
                                    value={values.imu_cal_auto_rest_sec}
                                    onChange={(v) => onChange("imu_cal_auto_rest_sec", v)}
                                    min={1} max={120} step={1} precision={0}
                                    style={{ width: "100%" }} addonAfter="s"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={24} sm={8}>
                            <Form.Item label="Periodic recal interval" tooltip="Re-estimate the IMU bias every N seconds while docked. 0 disables periodic recal.">
                                <InputNumber
                                    value={values.imu_cal_periodic_recal_sec}
                                    onChange={(v) => onChange("imu_cal_periodic_recal_sec", v)}
                                    min={0} max={3600} step={30} precision={0}
                                    style={{ width: "100%" }} addonAfter="s"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>
        </div>
    );
};
