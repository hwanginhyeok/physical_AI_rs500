import { PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useState, useCallback } from "react";
import { createRoot } from "react-dom/client";

// ── Types ──────────────────────────────────────────────────────────────────

interface CoverageData {
  distance_m: number;
  area_m2: number;
  speed_mps: number;
}

interface GeofenceData {
  inside: boolean;
  latitude: number;
  longitude: number;
}

interface E2EStatus {
  mode: string;
  safety_state: string;
  fallback_level: number;
  emergency_stop_count: number;
  confidence: number;
  latency_ms: number;
  generation_method: string;
}

// ── Styles ─────────────────────────────────────────────────────────────────

const STYLES = {
  container: {
    padding: "12px",
    fontFamily: "'Segoe UI', system-ui, sans-serif",
    color: "#e0e0e0",
    backgroundColor: "#1e1e2e",
    height: "100%",
    overflow: "auto",
  } as React.CSSProperties,
  header: {
    fontSize: "16px",
    fontWeight: 700,
    marginBottom: "12px",
    color: "#cdd6f4",
    borderBottom: "1px solid #45475a",
    paddingBottom: "6px",
  } as React.CSSProperties,
  section: {
    marginBottom: "12px",
    padding: "8px",
    backgroundColor: "#313244",
    borderRadius: "6px",
  } as React.CSSProperties,
  sectionTitle: {
    fontSize: "11px",
    fontWeight: 600,
    textTransform: "uppercase" as const,
    color: "#a6adc8",
    marginBottom: "6px",
    letterSpacing: "0.5px",
  } as React.CSSProperties,
  metricRow: {
    display: "flex",
    justifyContent: "space-between",
    alignItems: "center",
    padding: "2px 0",
  } as React.CSSProperties,
  metricLabel: {
    fontSize: "12px",
    color: "#bac2de",
  } as React.CSSProperties,
  metricValue: {
    fontSize: "14px",
    fontWeight: 600,
    fontVariantNumeric: "tabular-nums",
  } as React.CSSProperties,
  statusBadge: (color: string) => ({
    display: "inline-block",
    padding: "2px 8px",
    borderRadius: "4px",
    fontSize: "11px",
    fontWeight: 700,
    backgroundColor: color,
    color: "#1e1e2e",
  } as React.CSSProperties),
  progressBar: {
    width: "100%",
    height: "6px",
    backgroundColor: "#45475a",
    borderRadius: "3px",
    marginTop: "4px",
    overflow: "hidden",
  } as React.CSSProperties,
  progressFill: (pct: number, color: string) => ({
    width: `${Math.min(100, pct)}%`,
    height: "100%",
    backgroundColor: color,
    borderRadius: "3px",
    transition: "width 0.3s ease",
  } as React.CSSProperties),
  noData: {
    fontSize: "12px",
    color: "#6c7086",
    fontStyle: "italic",
  } as React.CSSProperties,
};

// ── Panel Component ────────────────────────────────────────────────────────

function AgriMissionPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [coverage, setCoverage] = useState<CoverageData | undefined>();
  const [geofence, setGeofence] = useState<GeofenceData | undefined>();
  const [e2eStatus, setE2eStatus] = useState<E2EStatus | undefined>();
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Target area (configurable via panel settings in future)
  const TARGET_AREA_M2 = 1000;

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);

      const messages = renderState.currentFrame;
      if (messages) {
        for (const msg of messages) {
          if (msg.topic === "/studio_script/coverage") {
            setCoverage(msg.message as unknown as CoverageData);
          } else if (msg.topic === "/studio_script/geofence") {
            setGeofence(msg.message as unknown as GeofenceData);
          } else if (msg.topic === "/hybrid_e2e/status") {
            try {
              const raw = (msg.message as { data: string }).data;
              setE2eStatus(JSON.parse(raw));
            } catch {
              // ignore parse errors
            }
          }
        }
      }
    };

    context.watch("currentFrame");
    context.subscribe([
      { topic: "/studio_script/coverage" },
      { topic: "/studio_script/geofence" },
      { topic: "/hybrid_e2e/status" },
    ]);
  }, [context]);

  useEffect(() => { renderDone?.(); }, [renderDone]);

  const modeColor = useCallback((mode?: string) => {
    switch (mode) {
      case "learned": return "#a6e3a1";
      case "traditional": return "#89b4fa";
      case "fallback": return "#fab387";
      case "emergency": return "#f38ba8";
      default: return "#6c7086";
    }
  }, []);

  const safetyColor = useCallback((state?: string) => {
    switch (state) {
      case "nominal": return "#a6e3a1";
      case "slow_down": return "#f9e2af";
      case "emergency_stop": return "#f38ba8";
      default: return "#6c7086";
    }
  }, []);

  const coveragePct = coverage ? (coverage.area_m2 / TARGET_AREA_M2) * 100 : 0;

  return (
    <div style={STYLES.container}>
      <div style={STYLES.header}>Agriculture Mission</div>

      {/* Coverage Section */}
      <div style={STYLES.section}>
        <div style={STYLES.sectionTitle}>Coverage Progress</div>
        {coverage ? (
          <>
            <div style={STYLES.metricRow}>
              <span style={STYLES.metricLabel}>Distance</span>
              <span style={{ ...STYLES.metricValue, color: "#a6e3a1" }}>
                {coverage.distance_m.toFixed(1)} m
              </span>
            </div>
            <div style={STYLES.metricRow}>
              <span style={STYLES.metricLabel}>Area Covered</span>
              <span style={{ ...STYLES.metricValue, color: "#89b4fa" }}>
                {coverage.area_m2.toFixed(1)} m²
              </span>
            </div>
            <div style={STYLES.metricRow}>
              <span style={STYLES.metricLabel}>Speed</span>
              <span style={{ ...STYLES.metricValue, color: "#cdd6f4" }}>
                {coverage.speed_mps.toFixed(2)} m/s
              </span>
            </div>
            <div style={STYLES.metricRow}>
              <span style={STYLES.metricLabel}>
                Progress ({coveragePct.toFixed(0)}%)
              </span>
            </div>
            <div style={STYLES.progressBar}>
              <div style={STYLES.progressFill(coveragePct, "#89b4fa")} />
            </div>
          </>
        ) : (
          <div style={STYLES.noData}>Waiting for coverage data...</div>
        )}
      </div>

      {/* Geofence Section */}
      <div style={STYLES.section}>
        <div style={STYLES.sectionTitle}>Geofence</div>
        {geofence ? (
          <>
            <div style={STYLES.metricRow}>
              <span style={STYLES.metricLabel}>Status</span>
              <span style={STYLES.statusBadge(geofence.inside ? "#a6e3a1" : "#f38ba8")}>
                {geofence.inside ? "INSIDE" : "OUTSIDE"}
              </span>
            </div>
            <div style={STYLES.metricRow}>
              <span style={STYLES.metricLabel}>Position</span>
              <span style={{ ...STYLES.metricValue, color: "#cdd6f4", fontSize: "11px" }}>
                {geofence.latitude.toFixed(6)}, {geofence.longitude.toFixed(6)}
              </span>
            </div>
          </>
        ) : (
          <div style={STYLES.noData}>Waiting for GPS data...</div>
        )}
      </div>

      {/* E2E Status Section */}
      <div style={STYLES.section}>
        <div style={STYLES.sectionTitle}>Hybrid E2E</div>
        {e2eStatus ? (
          <>
            <div style={STYLES.metricRow}>
              <span style={STYLES.metricLabel}>Mode</span>
              <span style={STYLES.statusBadge(modeColor(e2eStatus.mode))}>
                {e2eStatus.mode.toUpperCase()}
              </span>
            </div>
            <div style={STYLES.metricRow}>
              <span style={STYLES.metricLabel}>Safety</span>
              <span style={STYLES.statusBadge(safetyColor(e2eStatus.safety_state))}>
                {e2eStatus.safety_state.toUpperCase()}
              </span>
            </div>
            <div style={STYLES.metricRow}>
              <span style={STYLES.metricLabel}>Planning</span>
              <span style={{ ...STYLES.metricValue, color: "#cdd6f4" }}>
                {e2eStatus.generation_method}
              </span>
            </div>
            <div style={STYLES.metricRow}>
              <span style={STYLES.metricLabel}>Confidence</span>
              <span style={{ ...STYLES.metricValue, color: "#f9e2af" }}>
                {(e2eStatus.confidence * 100).toFixed(0)}%
              </span>
            </div>
            <div style={STYLES.metricRow}>
              <span style={STYLES.metricLabel}>Latency</span>
              <span style={{ ...STYLES.metricValue, color: "#cdd6f4" }}>
                {e2eStatus.latency_ms.toFixed(1)} ms
              </span>
            </div>
            {e2eStatus.emergency_stop_count > 0 && (
              <div style={STYLES.metricRow}>
                <span style={STYLES.metricLabel}>E-Stops</span>
                <span style={{ ...STYLES.metricValue, color: "#f38ba8" }}>
                  {e2eStatus.emergency_stop_count}
                </span>
              </div>
            )}
          </>
        ) : (
          <div style={STYLES.noData}>Waiting for E2E status...</div>
        )}
      </div>
    </div>
  );
}

// ── Init ───────────────────────────────────────────────────────────────────

export function initAgriMissionPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<AgriMissionPanel context={context} />);
  return () => { root.unmount(); };
}
