/// Report generation: terminal, JSON output for scenario results.
use crate::metrics::SceneResult;

/// Summary of a single scenario run.
#[derive(Debug, serde::Serialize)]
pub struct ScenarioReport {
    pub name: String,
    pub category: String,
    pub passed: bool,
    pub detected: usize,
    pub expected: usize,
    pub detection_rate: f64,
    pub corner_rmse: f64,
    pub max_corner_error: f64,
    pub false_positives: usize,
    pub detection_time_us: u64,
    pub threshold: f64,
    /// Mean rotation error in degrees (None if no pose data).
    pub mean_rotation_error_deg: Option<f64>,
    /// Mean translation error normalized by t_z (None if no pose data).
    pub mean_translation_error_frac: Option<f64>,
}

/// Full report across all scenarios.
#[derive(Debug, serde::Serialize)]
pub struct FullReport {
    pub scenarios: Vec<ScenarioReport>,
    pub total: usize,
    pub passed: usize,
    pub failed: usize,
}

impl FullReport {
    pub fn from_scenarios(scenarios: Vec<ScenarioReport>) -> Self {
        let total = scenarios.len();
        let passed = scenarios.iter().filter(|s| s.passed).count();
        let failed = total - passed;
        Self {
            scenarios,
            total,
            passed,
            failed,
        }
    }

    pub fn all_passed(&self) -> bool {
        self.failed == 0
    }
}

/// Print a terminal table summarizing results.
pub fn print_terminal(report: &FullReport) {
    let has_pose = report
        .scenarios
        .iter()
        .any(|s| s.mean_rotation_error_deg.is_some());

    if has_pose {
        println!(
            "{:<35} {:>5} {:>8} {:>8} {:>8} {:>7} {:>7} {:>6}",
            "Scenario", "Det%", "RMSE", "MaxErr", "FP", "RotErr", "TrnErr", "Status"
        );
        println!("{}", "-".repeat(95));
    } else {
        println!(
            "{:<35} {:>5} {:>8} {:>8} {:>8} {:>6}",
            "Scenario", "Det%", "RMSE", "MaxErr", "FP", "Status"
        );
        println!("{}", "-".repeat(75));
    }

    for s in &report.scenarios {
        let status = if s.passed { "PASS" } else { "FAIL" };
        if has_pose {
            let rot = s
                .mean_rotation_error_deg
                .map_or("--".to_string(), |v| format!("{v:.1}°"));
            let trn = s
                .mean_translation_error_frac
                .map_or("--".to_string(), |v| format!("{v:.4}"));
            println!(
                "{:<35} {:>4.0}% {:>8.2} {:>8.2} {:>8} {:>7} {:>7} {:>6}",
                truncate(&s.name, 35),
                s.detection_rate * 100.0,
                s.corner_rmse,
                s.max_corner_error,
                s.false_positives,
                rot,
                trn,
                status,
            );
        } else {
            println!(
                "{:<35} {:>4.0}% {:>8.2} {:>8.2} {:>8} {:>6}",
                truncate(&s.name, 35),
                s.detection_rate * 100.0,
                s.corner_rmse,
                s.max_corner_error,
                s.false_positives,
                status,
            );
        }
    }

    let sep_width = if has_pose { 95 } else { 75 };
    println!("{}", "-".repeat(sep_width));
    println!(
        "Total: {} | Passed: {} | Failed: {}",
        report.total, report.passed, report.failed
    );
}

/// Render report as JSON.
pub fn to_json(report: &FullReport) -> String {
    serde_json::to_string_pretty(report).unwrap_or_else(|e| format!("{{\"error\": \"{e}\"}}"))
}

/// Build a ScenarioReport from a scenario name, result, and threshold.
pub fn scenario_report(
    name: &str,
    category: &str,
    result: &SceneResult,
    expected_count: usize,
    threshold: f64,
    max_rotation_error_deg: Option<f64>,
) -> ScenarioReport {
    let detected = result
        .matches
        .iter()
        .filter(|m| m.detection.is_some())
        .count();
    let mut passed = result.detection_rate >= 1.0 && result.corner_rmse <= threshold;

    // Check rotation error threshold if set
    if let (Some(max_rot), Some(actual_rot)) =
        (max_rotation_error_deg, result.mean_rotation_error_deg)
    {
        if actual_rot > max_rot {
            passed = false;
        }
    }

    ScenarioReport {
        name: name.to_string(),
        category: category.to_string(),
        passed,
        detected,
        expected: expected_count,
        detection_rate: result.detection_rate,
        corner_rmse: result.corner_rmse,
        max_corner_error: result.max_corner_error,
        false_positives: result.false_positives.len(),
        detection_time_us: result.detection_time_us,
        threshold,
        mean_rotation_error_deg: result.mean_rotation_error_deg,
        mean_translation_error_frac: result.mean_translation_error_frac,
    }
}

fn truncate(s: &str, max_len: usize) -> String {
    if s.len() <= max_len {
        s.to_string()
    } else {
        format!("{}…", &s[..max_len - 1])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_report_counts() {
        let reports = vec![
            ScenarioReport {
                name: "a".into(),
                category: "test".into(),
                passed: true,
                detected: 1,
                expected: 1,
                detection_rate: 1.0,
                corner_rmse: 0.5,
                max_corner_error: 0.7,
                false_positives: 0,
                detection_time_us: 100,
                threshold: 2.0,
                mean_rotation_error_deg: None,
                mean_translation_error_frac: None,
            },
            ScenarioReport {
                name: "b".into(),
                category: "test".into(),
                passed: false,
                detected: 0,
                expected: 1,
                detection_rate: 0.0,
                corner_rmse: 0.0,
                max_corner_error: 0.0,
                false_positives: 0,
                detection_time_us: 200,
                threshold: 2.0,
                mean_rotation_error_deg: None,
                mean_translation_error_frac: None,
            },
        ];
        let full = FullReport::from_scenarios(reports);
        assert_eq!(full.total, 2);
        assert_eq!(full.passed, 1);
        assert_eq!(full.failed, 1);
        assert!(!full.all_passed());
    }

    #[test]
    fn json_output_parses() {
        let full = FullReport::from_scenarios(vec![]);
        let json = to_json(&full);
        let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed["total"], 0);
        assert_eq!(parsed["passed"], 0);
    }

    #[test]
    fn truncate_short_string() {
        assert_eq!(truncate("hello", 10), "hello");
    }

    #[test]
    fn truncate_long_string() {
        let result = truncate("this-is-a-very-long-scenario-name", 20);
        assert_eq!(result.chars().count(), 20);
        assert!(result.ends_with('…'));
    }
}
