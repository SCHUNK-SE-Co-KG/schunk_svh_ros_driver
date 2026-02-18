#!/usr/bin/env python3
"""
CVE-Scanner für das schunk_svh_ros_driver Repository.

Scannt ROS 2 package.xml-Abhängigkeiten, CMakeLists.txt find_package()-
Aufrufe sowie System-Bibliotheken gegen die Google OSV API (https://osv.dev)
und die GitHub Advisory Database auf bekannte Sicherheitslücken.

Ergebnisse werden als Markdown-Report und JSON unter
security/reports/ abgelegt.

Verwendung:
    python security/cve_scanner.py                # Scan + Report
    python security/cve_scanner.py --json-only    # Nur JSON-Ausgabe

Voraussetzungen:
    Python 3.9+, requests (pip install requests)

Autor:  SCHUNK Security Tooling
Lizenz: GPL-3.0
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

# ── Stdout auf UTF-8 umstellen (Windows cp1252 crasht bei Emojis) ──
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

try:
    import requests
except ImportError:
    sys.exit(
        "Fehler: 'requests' ist nicht installiert.\n"
        "  pip install requests"
    )

# ── Konstanten ───────────────────────────────────────────────────────
ROOT_DIR = Path(__file__).resolve().parent.parent
REPORT_DIR = ROOT_DIR / "security" / "reports"
REPO_NAME = "SCHUNK-SE-Co-KG/schunk_svh_ros_driver"

OSV_QUERY_URL = "https://api.osv.dev/v1/query"
OSV_BATCH_URL = "https://api.osv.dev/v1/querybatch"
OSV_VULN_URL = "https://api.osv.dev/v1/vulns"  # + /{id}

CVSS_HIGH_THRESHOLD = 7.0  # Ab diesem Wert wird ein Alert ausgelöst

# Severity-Label → geschätzter CVSS-Basiswert (letzter Fallback)
SEVERITY_LABEL_SCORES: dict[str, float] = {
    "CRITICAL": 9.0,
    "HIGH": 7.5,
    "MODERATE": 5.0,
    "MEDIUM": 5.0,
    "LOW": 2.5,
}

# ── ROS-Paket → OSV-Ecosystem-Mapping ───────────────────────────────
# ROS-Pakete, die als System-Bibliotheken installiert werden und über
# bekannte Ecosystems in OSV abfragbar sind.
#
# Mapping: rosdep-Key → (ecosystem, osv_package_name, known_version_or_empty)
#
# Hinweis: Die meisten ROS 2 Core-Pakete (rclcpp, launch, etc.) haben
# kein eigenes OSV-Ecosystem. Wir scannen sie über "GitHub" und
# "OSS-Fuzz" wo möglich.

# System-Bibliotheken die über apt/system installiert werden
SYSTEM_DEPS: dict[str, dict] = {
    "glfw3": {
        "ecosystem": "Debian",
        "name": "glfw3",
        "description": "GLFW3 OpenGL library",
    },
    "glew": {
        "ecosystem": "Debian",
        "name": "glew",
        "description": "OpenGL Extension Wrangler",
    },
    "mujoco": {
        "ecosystem": "PyPI",
        "name": "mujoco",
        "description": "MuJoCo Physics Engine",
    },
}

# ROS 2 Pakete mit bekannten upstream-Quellen die über GitHub Advisory
# scanbar sind. Format: rosdep_key → GitHub repo
ROS_UPSTREAM_REPOS: dict[str, str] = {
    "rclcpp": "ros2/rclcpp",
    "pluginlib": "ros/pluginlib",
    "hardware_interface": "ros-controls/ros2_control",
    "controller_manager": "ros-controls/ros2_control",
    "joint_state_broadcaster": "ros-controls/ros2_controllers",
    "joint_trajectory_controller": "ros-controls/ros2_controllers",
    "robot_state_publisher": "ros/robot_state_publisher",
    "xacro": "ros/xacro",
    "launch": "ros2/launch",
    "launch_ros": "ros2/launch_ros",
    "rviz2": "ros2/rviz",
}

# Tags für die internen Packages (überspringen bei OSV-Scan)
INTERNAL_PACKAGES = {
    "schunk_svh_driver",
    "schunk_svh_description",
    "schunk_svh_simulation",
    "schunk_svh_tests",
    "schunk_svh_library",
}

# ROS build-tool Pakete (kein Security-Risk, überspringen)
BUILD_TOOL_PACKAGES = {
    "ament_cmake",
    "ament_cmake_pytest",
    "ament_cmake_copyright",
    "ament_lint_auto",
    "ament_lint_common",
    "ros2launch",
    "launch_pytest",
}


# ── Datenstrukturen ─────────────────────────────────────────────────
@dataclass
class Dependency:
    """Eine erkannte Abhängigkeit."""

    name: str
    version: str
    ecosystem: str  # "GitHub", "PyPI", "Debian", "ROS"
    source: str  # Datei, in der sie gefunden wurde
    upstream_repo: str = ""  # GitHub-Repo (falls bekannt)


# ── CVSS Score Extraktion ────────────────────────────────────────────
def _fetch_ghsa_cvss_score(ghsa_id: str) -> Optional[float]:
    """Holt den offiziellen numerischen CVSS-Score für ein GitHub Advisory.

    Strategie (Priorität):
    1. GitHub Advisory Webseite scrapen – zeigt IMMER den aktuellsten Score
       (CVSS 3.x oder 4.0), Pattern: ">X.X<" vor "/ 10"
    2. GitHub REST API (/advisories/{id}) – Fallback, kann veraltete
       CVSS 3.1 Scores liefern wenn Advisory auf CVSS 4.0 aktualisiert wurde
    """
    # 1. Advisory-Webseite scrapen (Primärquelle – immer aktuell)
    page_url = f"https://github.com/advisories/{ghsa_id}"
    try:
        resp = requests.get(page_url, timeout=10, headers={
            "Accept": "text/html",
            "User-Agent": "SCHUNK-CVE-Scanner/1.0",
        })
        if resp.status_code == 200:
            idx = resp.text.find("/ 10")
            if idx > 0:
                window = resp.text[max(0, idx - 3000):idx]
                numbers = re.findall(r">(\d+\.\d+)<", window)
                if numbers:
                    return float(numbers[-1])
    except requests.RequestException:
        pass

    # 2. REST API als Fallback
    url = f"https://api.github.com/advisories/{ghsa_id}"
    try:
        resp = requests.get(url, timeout=10, headers={
            "Accept": "application/vnd.github+json",
            "X-GitHub-Api-Version": "2022-11-28",
        })
        if resp.status_code == 200:
            data = resp.json()
            for field_name in ("cvss_v4", "cvss"):
                cvss_data = data.get(field_name)
                if cvss_data and isinstance(cvss_data, dict):
                    score = cvss_data.get("score")
                    if isinstance(score, (int, float)) and score > 0:
                        return float(score)
    except requests.RequestException:
        pass

    return None


@dataclass
class Vulnerability:
    """Eine gefundene Sicherheitslücke."""

    vuln_id: str
    summary: str
    severity: str
    cvss_score: float
    affected_package: str
    affected_versions: str
    fixed_version: str
    references: list[str] = field(default_factory=list)
    aliases: list[str] = field(default_factory=list)


@dataclass
class ScanResult:
    """Gesamtergebnis eines Scan-Laufs."""

    timestamp: str
    repository: str
    dependencies_scanned: int
    vulnerabilities_found: int
    high_severity_count: int = 0
    dependencies: list[Dependency] = field(default_factory=list)
    vulnerabilities: list[Vulnerability] = field(default_factory=list)


# ── Abhängigkeiten aus package.xml extrahieren ───────────────────────
def extract_package_xml_deps(root_dir: Path) -> list[Dependency]:
    """Extrahiert Abhängigkeiten aus allen package.xml-Dateien."""
    deps: list[Dependency] = []
    seen: set[str] = set()

    dep_tags = [
        "depend", "build_depend", "buildtool_depend",
        "exec_depend", "run_depend", "test_depend",
    ]

    for pkg_xml in root_dir.rglob("package.xml"):
        try:
            tree = ET.parse(pkg_xml)
            root = tree.getroot()
        except ET.ParseError as e:
            print(f"  ⚠  XML-Fehler in {pkg_xml}: {e}")
            continue

        source = str(pkg_xml.relative_to(root_dir))

        for tag in dep_tags:
            for elem in root.findall(tag):
                name = elem.text.strip() if elem.text else ""
                if not name:
                    continue

                # Interne und Build-Tool-Pakete überspringen
                if name in INTERNAL_PACKAGES or name in BUILD_TOOL_PACKAGES:
                    continue

                if name in seen:
                    continue
                seen.add(name)

                # Ecosystem und upstream bestimmen
                if name in SYSTEM_DEPS:
                    info = SYSTEM_DEPS[name]
                    deps.append(Dependency(
                        name=info["name"],
                        version="",
                        ecosystem=info["ecosystem"],
                        source=source,
                    ))
                elif name in ROS_UPSTREAM_REPOS:
                    deps.append(Dependency(
                        name=name,
                        version="",
                        ecosystem="ROS",
                        source=source,
                        upstream_repo=ROS_UPSTREAM_REPOS[name],
                    ))
                else:
                    # Allgemeines ROS-Paket ohne bekanntes Mapping
                    deps.append(Dependency(
                        name=name,
                        version="",
                        ecosystem="ROS",
                        source=source,
                    ))

    return deps


def extract_cmake_deps(root_dir: Path) -> list[Dependency]:
    """Extrahiert find_package()-Aufrufe und pkg_search_module() aus CMakeLists.txt."""
    deps: list[Dependency] = []
    seen: set[str] = set()

    find_pkg_re = re.compile(r"find_package\s*\(\s*(\w+)", re.IGNORECASE)
    pkg_config_re = re.compile(r"pkg_search_module\s*\(\s*\w+\s+(?:REQUIRED\s+)?(\w+)", re.IGNORECASE)

    for cmake_file in root_dir.rglob("CMakeLists.txt"):
        try:
            content = cmake_file.read_text(encoding="utf-8")
        except Exception:
            continue

        source = str(cmake_file.relative_to(root_dir))

        for pattern in [find_pkg_re, pkg_config_re]:
            for m in pattern.finditer(content):
                name = m.group(1)
                name_lower = name.lower()

                # Bereits gesehen oder build-tools → überspringen
                if name_lower in seen:
                    continue
                if name in BUILD_TOOL_PACKAGES or name in INTERNAL_PACKAGES:
                    continue
                if name_lower in {"ament_cmake", "pkgconfig", "ament_lint_auto",
                                  "ament_cmake_pytest"}:
                    continue

                seen.add(name_lower)

                if name_lower in SYSTEM_DEPS:
                    info = SYSTEM_DEPS[name_lower]
                    deps.append(Dependency(
                        name=info["name"],
                        version="",
                        ecosystem=info["ecosystem"],
                        source=source,
                    ))
                elif name in ROS_UPSTREAM_REPOS:
                    deps.append(Dependency(
                        name=name,
                        version="",
                        ecosystem="ROS",
                        source=source,
                        upstream_repo=ROS_UPSTREAM_REPOS[name],
                    ))

    return deps


def deduplicate_deps(deps: list[Dependency]) -> list[Dependency]:
    """Entfernt doppelte Abhängigkeiten (gleicher Name + Ecosystem)."""
    seen: set[str] = set()
    unique: list[Dependency] = []
    for d in deps:
        key = f"{d.ecosystem}:{d.name}"
        if key not in seen:
            seen.add(key)
            unique.append(d)
    return unique


# ── GitHub Advisory Scan für ROS upstream repos ──────────────────────
def scan_github_advisories(dep: Dependency) -> list[dict]:
    """Scannt GitHub Advisory Database für ein bestimmtes Repository."""
    if not dep.upstream_repo:
        return []

    url = "https://api.github.com/advisories"
    try:
        resp = requests.get(url, timeout=15, params={
            "ecosystem": "pip",  # Trick: GitHub API braucht ein Ecosystem
            "affects": dep.upstream_repo,
        }, headers={
            "Accept": "application/vnd.github+json",
            "X-GitHub-Api-Version": "2022-11-28",
        })
        if resp.status_code == 200:
            return resp.json()
    except requests.RequestException:
        pass

    # Alternative: Direkt über Repository Security Advisories
    repo_url = f"https://api.github.com/repos/{dep.upstream_repo}/security-advisories"
    try:
        resp = requests.get(repo_url, timeout=15, headers={
            "Accept": "application/vnd.github+json",
            "X-GitHub-Api-Version": "2022-11-28",
        })
        if resp.status_code == 200:
            return resp.json()
    except requests.RequestException:
        pass

    return []


# ── OSV-API-Abfragen ────────────────────────────────────────────────
def _build_osv_query(dep: Dependency) -> dict:
    """Baut eine einzelne OSV-Query für eine Abhängigkeit."""
    q: dict = {"package": {"name": dep.name, "ecosystem": dep.ecosystem}}
    if dep.version:
        q["version"] = dep.version
    return q


def query_osv_batch(deps: list[Dependency]) -> list[list[dict]]:
    """Sendet einen Batch-Query an die OSV-API."""
    if not deps:
        return []

    # Nur Deps mit bekanntem OSV-Ecosystem abfragen
    osv_ecosystems = {"PyPI", "Debian", "npm", "crates.io", "Go", "Maven"}

    queries = []
    query_indices = []
    for i, dep in enumerate(deps):
        if dep.ecosystem in osv_ecosystems:
            queries.append(_build_osv_query(dep))
            query_indices.append(i)

    results: list[list[dict]] = [[] for _ in deps]

    if queries:
        try:
            resp = requests.post(
                OSV_BATCH_URL,
                json={"queries": queries},
                timeout=30,
            )
            resp.raise_for_status()
            batch_results = resp.json().get("results", [])
            for j, res in enumerate(batch_results):
                orig_idx = query_indices[j]
                results[orig_idx] = res.get("vulns", [])
        except requests.RequestException as exc:
            print(f"  ⚠  OSV-API Batch-Fehler: {exc}")

    return results


def fetch_vuln_details(vuln_id: str) -> Optional[dict]:
    """Ruft vollständige Vulnerability-Details von der OSV-API ab."""
    try:
        resp = requests.get(f"{OSV_VULN_URL}/{vuln_id}", timeout=15)
        resp.raise_for_status()
        return resp.json()
    except requests.RequestException:
        return None


def enrich_vulnerabilities(vulns_per_dep: list[list[dict]]) -> list[list[dict]]:
    """Reichert Batch-Ergebnisse mit vollen Details an."""
    enriched: list[list[dict]] = []
    for dep_vulns in vulns_per_dep:
        enriched_list: list[dict] = []
        for v in dep_vulns:
            vuln_id = v.get("id", "")
            if not v.get("summary") and not v.get("details"):
                full = fetch_vuln_details(vuln_id)
                if full:
                    enriched_list.append(full)
                    continue
            enriched_list.append(v)
        enriched.append(enriched_list)
    return enriched


# ── Vulnerabilities parsen ──────────────────────────────────────────
def _extract_severity(vuln: dict) -> str:
    """Extrahiert den Schweregrad aus einer OSV-Vulnerability."""
    severity_entries = vuln.get("severity", [])
    for s in severity_entries:
        score_str = s.get("score", "")
        if score_str:
            return score_str
    db_spec = vuln.get("database_specific", {})
    severity = db_spec.get("severity", "")
    if severity:
        return severity
    return "UNKNOWN"


def _compute_cvss_score(vuln: dict, severity_str: str) -> float:
    """Holt den offiziellen CVSS-Score von der GitHub Advisory API."""
    vuln_id = vuln.get("id", "")

    # 1. GHSA-ID ermitteln
    ghsa_id = vuln_id if vuln_id.startswith("GHSA-") else ""
    if not ghsa_id:
        for alias in vuln.get("aliases", []):
            if alias.startswith("GHSA-"):
                ghsa_id = alias
                break

    if ghsa_id:
        api_score = _fetch_ghsa_cvss_score(ghsa_id)
        if api_score is not None:
            return api_score

    # 2. Fallback: Severity-Label
    db_severity = vuln.get("database_specific", {}).get("severity", "").upper()
    if db_severity in SEVERITY_LABEL_SCORES:
        return SEVERITY_LABEL_SCORES[db_severity]

    upper = severity_str.upper()
    for label in SEVERITY_LABEL_SCORES:
        if label in upper:
            return SEVERITY_LABEL_SCORES[label]

    return 0.0


def _extract_fixed_version(affected: dict) -> str:
    """Ermittelt die erste fixe Version aus affected-Ranges."""
    for rng in affected.get("ranges", []):
        for event in rng.get("events", []):
            if "fixed" in event:
                return event["fixed"]
    return "–"


def parse_vulnerabilities(
    dep: Dependency, vulns: list[dict]
) -> list[Vulnerability]:
    """Konvertiert rohe OSV-Einträge in Vulnerability-Objekte."""
    results: list[Vulnerability] = []
    for v in vulns:
        vuln_id = v.get("id", "?")
        summary = v.get("summary", v.get("details", "Keine Beschreibung"))
        severity = _extract_severity(v)
        aliases = v.get("aliases", [])

        affected_versions = ""
        fixed_version = "–"
        for aff in v.get("affected", []):
            pkg = aff.get("package", {})
            if pkg.get("name") == dep.name:
                affected_versions = ", ".join(aff.get("versions", [])[:10])
                if len(aff.get("versions", [])) > 10:
                    affected_versions += " …"
                fixed_version = _extract_fixed_version(aff)
                break

        refs = [r.get("url", "") for r in v.get("references", []) if r.get("url")]
        cvss_score = _compute_cvss_score(v, severity)

        results.append(
            Vulnerability(
                vuln_id=vuln_id,
                summary=summary[:300],
                severity=severity,
                cvss_score=cvss_score,
                affected_package=f"{dep.ecosystem}:{dep.name}"
                    + (f"@{dep.version}" if dep.version else ""),
                affected_versions=affected_versions,
                fixed_version=fixed_version,
                references=refs[:5],
                aliases=aliases,
            )
        )
    return results


def parse_github_advisory(dep: Dependency, advisory: dict) -> Optional[Vulnerability]:
    """Konvertiert ein GitHub Advisory in ein Vulnerability-Objekt."""
    ghsa_id = advisory.get("ghsa_id", "")
    if not ghsa_id:
        return None

    summary = advisory.get("summary", "Keine Beschreibung")
    severity = advisory.get("severity", "UNKNOWN")
    cve_id = advisory.get("cve_id", "")

    cvss_score = 0.0
    score = _fetch_ghsa_cvss_score(ghsa_id)
    if score is not None:
        cvss_score = score
    elif severity.upper() in SEVERITY_LABEL_SCORES:
        cvss_score = SEVERITY_LABEL_SCORES[severity.upper()]

    refs = []
    if advisory.get("html_url"):
        refs.append(advisory["html_url"])

    aliases = [ghsa_id]
    if cve_id:
        aliases.append(cve_id)

    return Vulnerability(
        vuln_id=ghsa_id,
        summary=summary[:300],
        severity=severity,
        cvss_score=cvss_score,
        affected_package=f"ROS:{dep.name}" + (f" ({dep.upstream_repo})" if dep.upstream_repo else ""),
        affected_versions="",
        fixed_version="–",
        references=refs[:5],
        aliases=aliases,
    )


# ── Report-Generierung ──────────────────────────────────────────────
def generate_markdown_report(result: ScanResult) -> str:
    """Erzeugt einen Markdown-Report aus dem Scan-Ergebnis."""
    lines: list[str] = []
    lines.append("# CVE-Scan Report – schunk_svh_ros_driver")
    lines.append("")
    lines.append(f"**Scan-Zeitpunkt:** {result.timestamp}  ")
    lines.append(f"**Repository:** {result.repository}  ")
    lines.append(
        f"**Abhängigkeiten geprüft:** {result.dependencies_scanned}  "
    )
    lines.append(
        f"**Schwachstellen gefunden:** {result.vulnerabilities_found}"
    )
    lines.append("")

    if result.vulnerabilities_found == 0:
        lines.append("> ✅ **Keine bekannten Schwachstellen gefunden.**")
    else:
        lines.append(
            f"> ⚠️ **{result.vulnerabilities_found} Schwachstelle(n) gefunden!**"
        )
    lines.append("")

    # Geprüfte Abhängigkeiten
    lines.append("## Geprüfte Abhängigkeiten")
    lines.append("")
    lines.append("| Paket | Version | Ökosystem | Quelle | Upstream |")
    lines.append("|-------|---------|-----------|--------|----------|")
    for d in result.dependencies:
        upstream = f"[{d.upstream_repo}](https://github.com/{d.upstream_repo})" if d.upstream_repo else "–"
        lines.append(
            f"| {d.name} | {d.version or '–'} | {d.ecosystem} | {d.source} | {upstream} |"
        )
    lines.append("")

    # Schwachstellen
    if result.vulnerabilities:
        lines.append("## Gefundene Schwachstellen")
        lines.append("")
        for v in result.vulnerabilities:
            cve_aliases = [a for a in v.aliases if a.startswith("CVE-")]
            cve_str = ", ".join(cve_aliases) if cve_aliases else "–"
            lines.append(f"### {v.vuln_id}")
            lines.append("")
            lines.append(f"- **Paket:** {v.affected_package}")
            score_display = f"{v.cvss_score:.1f}" if v.cvss_score > 0 else "–"
            high_marker = " 🔴" if v.cvss_score >= CVSS_HIGH_THRESHOLD else ""
            lines.append(f"- **CVSS-Score:** {score_display}{high_marker}")
            lines.append(f"- **Schweregrad:** {v.severity}")
            lines.append(f"- **CVE:** {cve_str}")
            lines.append(f"- **Beschreibung:** {v.summary}")
            lines.append(f"- **Fix-Version:** {v.fixed_version}")
            if v.references:
                lines.append("- **Referenzen:**")
                for ref in v.references:
                    lines.append(f"  - {ref}")
            lines.append("")

    lines.append("---")
    lines.append(
        "*Automatisch generiert von `security/cve_scanner.py` "
        "via [OSV.dev](https://osv.dev) und GitHub Advisory Database.*"
    )
    lines.append("")
    return "\n".join(lines)


def generate_alert_body(result: ScanResult) -> str:
    """Erzeugt den Body für den High-Severity-Alert."""
    high = [v for v in result.vulnerabilities if v.cvss_score >= CVSS_HIGH_THRESHOLD]
    lines: list[str] = []
    lines.append("⚠️ CVE-Alert: Kritische Schwachstellen in schunk_svh_ros_driver")
    lines.append("=" * 65)
    lines.append("")
    lines.append(f"Scan-Zeitpunkt: {result.timestamp}")
    lines.append(f"Repository:     {result.repository}")
    lines.append(f"Schwachstellen: {result.vulnerabilities_found} gesamt, "
                 f"{len(high)} mit CVSS >= {CVSS_HIGH_THRESHOLD}")
    lines.append("")
    lines.append("-" * 65)

    for v in high:
        cve_aliases = [a for a in v.aliases if a.startswith("CVE-")]
        cve_str = ", ".join(cve_aliases) if cve_aliases else "–"
        lines.append("")
        lines.append(f"  {v.vuln_id}  (CVSS {v.cvss_score:.1f})")
        lines.append(f"  Paket:       {v.affected_package}")
        lines.append(f"  CVE:         {cve_str}")
        lines.append(f"  Fix-Version: {v.fixed_version}")
        lines.append(f"  {v.summary[:200]}")
        if v.references:
            lines.append(f"  → {v.references[0]}")

    lines.append("")
    lines.append("-" * 65)
    lines.append("Vollständiger Report: security/reports/cve_report.md")
    lines.append(f"https://github.com/{REPO_NAME}/blob/ros2/security/reports/cve_report.md")
    lines.append("")
    return "\n".join(lines)


def write_alert_output(result: ScanResult, report_dir: Path) -> None:
    """Schreibt Alert-Dateien für GitHub Actions."""
    report_dir.mkdir(parents=True, exist_ok=True)
    alert_flag_path = report_dir / "alert_high_severity.txt"

    if result.high_severity_count > 0:
        alert_body = generate_alert_body(result)
        alert_flag_path.write_text(alert_body, encoding="utf-8")
        print(f"📧 Alert-Datei: {alert_flag_path.relative_to(ROOT_DIR)}")

        gh_output = os.environ.get("GITHUB_OUTPUT")
        if gh_output:
            with open(gh_output, "a") as f:
                f.write("high_severity=true\n")
                f.write(f"high_count={result.high_severity_count}\n")
                f.write(f"alert_subject=⚠️ CVE-Alert: {result.high_severity_count} "
                        "kritische Schwachstelle(n) in schunk_svh_ros_driver\n")
    else:
        if alert_flag_path.exists():
            alert_flag_path.unlink()
        gh_output = os.environ.get("GITHUB_OUTPUT")
        if gh_output:
            with open(gh_output, "a") as f:
                f.write("high_severity=false\n")


def save_reports(result: ScanResult, report_dir: Path) -> tuple[Path, Path]:
    """Speichert JSON- und Markdown-Reports."""
    report_dir.mkdir(parents=True, exist_ok=True)

    json_path = report_dir / "cve_report.json"
    md_path = report_dir / "cve_report.md"

    json_path.write_text(
        json.dumps(asdict(result), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    md_path.write_text(generate_markdown_report(result), encoding="utf-8")

    return json_path, md_path


# ── Hauptlogik ───────────────────────────────────────────────────────
def run_scan() -> ScanResult:
    """Führt den vollständigen CVE-Scan durch."""
    print("🔍 CVE-Scanner gestartet …")
    print(f"   Repository-Root: {ROOT_DIR}")
    print()

    # 1. Abhängigkeiten aus package.xml + CMakeLists.txt sammeln
    pkg_deps = extract_package_xml_deps(ROOT_DIR)
    cmake_deps = extract_cmake_deps(ROOT_DIR)
    all_deps = deduplicate_deps(pkg_deps + cmake_deps)

    print(f"📦 {len(all_deps)} Abhängigkeit(en) erkannt:")
    for d in all_deps:
        upstream = f" → {d.upstream_repo}" if d.upstream_repo else ""
        print(f"   • {d.name}  ({d.ecosystem}){upstream}")
    print()

    # 2. OSV-API Batch-Abfrage (für Deps mit OSV-Ecosystem)
    print("🌐 Frage OSV-API ab …")
    batch_results = query_osv_batch(all_deps)
    batch_results = enrich_vulnerabilities(batch_results)

    # 3. GitHub Advisory Scan für ROS-Pakete mit bekanntem Upstream
    print("🔍 Prüfe GitHub Advisory Database für ROS-Upstream-Repos …")
    all_vulns: list[Vulnerability] = []

    for dep, osv_vulns in zip(all_deps, batch_results):
        # OSV-Ergebnisse verarbeiten
        parsed = parse_vulnerabilities(dep, osv_vulns)
        if parsed:
            print(f"   ⚠  {dep.name}: {len(parsed)} Schwachstelle(n) (OSV)")
        all_vulns.extend(parsed)

        # GitHub Advisories für ROS-Upstream-Repos
        if dep.upstream_repo:
            gh_advisories = scan_github_advisories(dep)
            seen_ids = {v.vuln_id for v in all_vulns}
            for adv in gh_advisories:
                vuln = parse_github_advisory(dep, adv)
                if vuln and vuln.vuln_id not in seen_ids:
                    all_vulns.append(vuln)
                    seen_ids.add(vuln.vuln_id)
                    print(f"   ⚠  {dep.name}: {vuln.vuln_id} (GitHub Advisory)")

    if not all_vulns:
        print("   ✅ Keine Schwachstellen gefunden.")

    high_sev = [v for v in all_vulns if v.cvss_score >= CVSS_HIGH_THRESHOLD]
    if high_sev:
        print(f"   🔴 {len(high_sev)} Schwachstelle(n) mit CVSS ≥ {CVSS_HIGH_THRESHOLD}!")

    result = ScanResult(
        timestamp=datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        repository=REPO_NAME,
        dependencies_scanned=len(all_deps),
        vulnerabilities_found=len(all_vulns),
        high_severity_count=len(high_sev),
        dependencies=all_deps,
        vulnerabilities=all_vulns,
    )

    return result


def main() -> None:
    parser = argparse.ArgumentParser(
        description="CVE-Scanner für schunk_svh_ros_driver Abhängigkeiten"
    )
    parser.add_argument(
        "--json-only",
        action="store_true",
        help="Nur JSON auf stdout ausgeben, keine Dateien schreiben",
    )
    args = parser.parse_args()

    result = run_scan()

    if args.json_only:
        print(json.dumps(asdict(result), indent=2, ensure_ascii=False))
        return

    json_path, md_path = save_reports(result, REPORT_DIR)
    print()
    print(f"📄 JSON-Report: {json_path.relative_to(ROOT_DIR)}")
    print(f"📄 Markdown-Report: {md_path.relative_to(ROOT_DIR)}")

    write_alert_output(result, REPORT_DIR)
    print()

    if result.vulnerabilities_found > 0:
        print(
            f"⚠️  {result.vulnerabilities_found} Schwachstelle(n) gefunden! "
            "Details im Report."
        )
        if result.high_severity_count > 0:
            print(
                f"🔴 {result.high_severity_count} davon mit CVSS ≥ {CVSS_HIGH_THRESHOLD} "
                "– Alert wird ausgelöst."
            )
        sys.exit(1)
    else:
        print("✅ Keine bekannten Schwachstellen – alles sicher.")
        sys.exit(0)


if __name__ == "__main__":
    main()
