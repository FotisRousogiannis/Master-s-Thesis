"""Catalog of useful commands for the Tools panel (tools.yaml), and how each one is run."""

import shlex
from dataclasses import dataclass
from pathlib import Path

import yaml

CATALOG = Path(__file__).resolve().parent / "tools.yaml"
WHERE = ("local", "pi")


@dataclass(frozen=True)
class Tool:
    id: str
    group: str
    name: str
    where: str
    command: str
    description: str = ""
    moves: bool = False


@dataclass(frozen=True)
class PiTarget:
    host: str = "10.42.0.2"  # the Pi on the direct cable (ADR-0021, software/launcher/pi_link.sh)
    user: str = ""
    repo: str = "Master-s-Thesis"  # relative to the home directory on the Pi, or absolute

    @property
    def ready(self) -> bool:
        return bool(self.host.strip() and self.user.strip())


def load_catalog(path=CATALOG) -> list[Tool]:
    entries = yaml.safe_load(Path(path).read_text(encoding="utf-8"))["tools"]
    tools = [Tool(**e) for e in entries]
    ids = [t.id for t in tools]
    if len(ids) != len(set(ids)):
        raise ValueError("duplicate tool id")
    for t in tools:
        if t.where not in WHERE:
            raise ValueError(f"{t.id}: where must be one of {WHERE}")
    return tools


def command_for(tool: Tool, pi: PiTarget, ssh_dir: Path) -> tuple[str, list[str]]:
    """(program, args) to start the tool. ssh_dir holds id_ed25519 and known_hosts (outside git)."""
    if tool.where == "local":
        return "bash", ["-c", tool.command]
    if not pi.ready:
        raise ValueError("Δεν έχουν οριστεί host και χρήστης του Raspberry Pi")
    repo = pi.repo.strip() or "."
    cd = shlex.quote(repo) if repo.startswith("/") else f"~/{shlex.quote(repo)}"
    return "ssh", [
        "-o", "BatchMode=yes",  # never wait for a password: fail with a message instead
        "-o", "ConnectTimeout=5",
        "-o", "StrictHostKeyChecking=accept-new",
        "-o", f"UserKnownHostsFile={ssh_dir / 'known_hosts'}",
        "-i", str(ssh_dir / "id_ed25519"),
        f"{pi.user.strip()}@{pi.host.strip()}",
        f"cd {cd} && {tool.command}",
    ]
