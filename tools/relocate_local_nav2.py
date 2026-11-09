"""Repair Debian CMake exports for project-local extraction; keep system deps intact."""
from pathlib import Path
import re
prefix = Path(__file__).resolve().parents[1] / '.deps/nav2_root/opt/ros/humble'
pattern = re.compile('/opt/ros/humble/(?:include|lib|share)/[A-Za-z0-9_.+/-]+')
for file in prefix.rglob('*.cmake'):
    text = file.read_text()

    def replace(match):
        target = prefix / match.group(0).removeprefix('/opt/ros/humble/')
        return str(target) if target.exists() else match.group(0)
    updated = pattern.sub(replace, text)
    if updated != text:
        file.write_text(updated)
