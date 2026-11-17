"""The calibration wizard of ADR-0023 without Qt: what to ask the Pi, and the proposals it collects.

The Pi's identification mode (arm_pi_driver/identify.py) answers JSON requests; the steps
themselves are arm_calibration.procedure. This class only keeps the state between them:

    request = wizard.scan()            -> the GUI publishes it
    follow_ups = wizard.handle(reply)  -> the GUI publishes these too (deadband sweeps)

Nothing is written to arm.yaml here: proposals wait for the user's approval, and approved ones
go to /arm_calibration as parameters, all at once (the node validates the whole file).
"""

import re
import time

from arm_calibration import procedure


def natural(name: str):
    """joint_2 before joint_10, servo_2 before servo_10 (ROS gives parameters alphabetically)."""
    return [int(part) if part.isdigit() else part for part in re.split(r"(\d+)", name)]


READ_PREFIX = "Ανάγνωση · "
MAGNET_NOTES = {   # what the AS5600 status means for the user (AS5600 datasheet: AGC out of range)
    "TOO_WEAK": " (μαγνήτης μακριά ή μικρός: η γωνία μετρά, με θόρυβο· φέρ' τον πιο κοντά, ~0.5–3 mm)",
    "TOO_STRONG": " (μαγνήτης πολύ κοντά: η γωνία μετρά, με θόρυβο· απομάκρυνέ τον λίγο)",
    "NO_MAGNET": " (δεν βλέπει μαγνήτη: η γωνία δεν ισχύει)",
}
ORDER = {"joints": lambda n: (n == "gripper", natural(n))}      # the gripper last, as in arm.yaml


def nest(values: dict) -> dict:
    """Flat parameters of /arm_calibration -> the nested data of arm.yaml."""
    data: dict = {}
    for name, value in values.items():
        node = data
        *parents, leaf = name.split(".")
        for part in parents:
            node = node.setdefault(part, {})
        if leaf in ("channel", "adc_channel") and value == -1:     # -1 = not set (arm_config.NO_CHANNEL)
            value = None
        node[leaf] = list(value) if isinstance(value, (list, tuple)) else value
    for section in ("servos", "encoders", "force_sensors", "joints", "controller"):
        data.setdefault(section, {})
    for section in ("servos", "encoders", "force_sensors"):
        for item in data[section].values():   # a null of the file has no parameter
            item.setdefault("channel" if section != "force_sensors" else "adc_channel", None)
    for joint in data["joints"].values():
        joint.setdefault("encoder", None)
        joint.setdefault("servos", {})
    for section in ("servos", "encoders", "force_sensors", "joints"):
        key = ORDER.get(section, natural)
        data[section] = {name: data[section][name] for name in sorted(data[section], key=key)}
    return data


class Wizard:
    """State of the calibration procedure between requests to the Pi."""

    def __init__(self):
        self.data = None
        self.active = False                 # the Pi is (we asked it to be) in identification mode
        self.findings = []                  # [procedure.Finding]
        self.proposals = []                 # [[procedure.Proposal, approved]]
        self.readings = {}                  # {channel: {'raw_deg', 'status'}} of the last read
        self.mapping = None                 # procedure.JointMapping of the last joint jog
        self.sweep = None                   # running deadband sweep
        self.busy = False                   # a jog is running on the Pi
        self._next_id = 1
        self._pending = {}                  # {id: (kind, argument)}

    # ---------------------------------------------------------------- requests
    def _request(self, kind: str, request: dict, argument=None) -> dict:
        request = {"id": self._next_id, **request}
        self._pending[self._next_id] = (kind, argument)
        self._next_id += 1
        if request["op"] != "exit":
            self.active = True
        if request["op"] == "jog":
            self.busy = True
        return request

    def scan(self) -> dict:
        return self._request("scan", {"op": "scan"})

    def read(self) -> dict:
        return self._request("read", {"op": "read"})

    def keepalive(self) -> dict | None:
        """A read every few seconds keeps the Pi in identification mode (it leaves after 5 s).

        Its readings refresh the pages quietly; only «Ανάγνωση» writes them into the findings.
        """
        if not self.active or self.busy:
            return None
        return self._request("keepalive", {"op": "read"})

    def jog_joint(self, joint: str, speed: float = procedure.JOG_SPEED,
                  duration_s: float = procedure.JOG_DURATION_S) -> dict:
        self.mapping = None
        return self._request("joint", procedure.joint_jog(self.data, joint, speed, duration_s), joint)

    def servo_deadband(self, servo: str) -> dict:
        self.sweep = procedure.ServoDeadbandSweep(self.data, servo)
        return self._request("sweep", self.sweep.next_request())

    def joint_deadband(self, joint: str) -> dict:
        self.sweep = procedure.JointDeadbandSweep(self.data, joint)
        return self._request("sweep", self.sweep.next_request())

    def stop(self) -> dict:
        self.sweep = None
        return self._request("stop", {"op": "stop"})

    def exit(self) -> dict:
        self.sweep, self.busy, self.active = None, False, False
        return self._request("exit", {"op": "exit"})

    # ---------------------------------------------------------------- replies
    def handle(self, reply: dict) -> list:
        """One reply of the Pi -> follow-up requests (deadband sweeps), and new findings/proposals."""
        kind, argument = self._pending.get(reply.get("id"), (None, None))
        if reply.get("op") == "exit" and reply.get("id") is None:          # the Pi left on its own
            self.active, self.busy, self.sweep = False, False, None
            self.findings.append(procedure.Finding(procedure.WARNING, reply.get("result", "")))
            return []
        if kind is None:
            return []
        if not reply.get("ok"):
            self._pending.pop(reply["id"], None)
            self.busy, self.sweep = False, None
            self.findings.append(procedure.Finding(procedure.ERROR, reply.get("error", "σφάλμα")))
            return []
        result = reply.get("result")
        if reply.get("op") == "jog" and isinstance(result, str):            # "started": wait
            return []
        self._pending.pop(reply["id"], None)
        if reply.get("op") == "jog":
            self.busy = False
        if kind == "scan":
            self.readings = {int(ch): v["encoder"] for ch, v in result.items() if v.get("encoder")}
            self.findings = procedure.check_scan(self.data, result)
        elif kind == "keepalive":
            self.readings = {int(ch): v for ch, v in result.items()}
        elif kind == "read":
            self.readings = {int(ch): v for ch, v in result.items()}
            self.findings = [f for f in self.findings if not f.text.startswith(READ_PREFIX)]
            self.findings += self.reading_findings()
        elif kind == "joint":
            self.mapping = procedure.analyze_jog(self.data, argument, result)
            self.findings += self.mapping.findings
            self.add(self.mapping.proposals)
            moved = ", ".join(f"κανάλι {ch}: {d:+.1f}°" for ch, d in result.get("moved", {}).items())
            self.findings.append(procedure.Finding(procedure.INFO, f"{argument}: {moved or 'καμία κίνηση'}"
                                                   f" ({result.get('reason')})"))
        elif kind == "sweep" and self.sweep is not None:
            self.sweep.record(result)
            request = self.sweep.next_request()
            if request is not None:
                return [self._request("sweep", request)]
            proposals = self.sweep.proposals(self.data)
            self.add(proposals)
            text = "νεκρή ζώνη: " + ("βρέθηκε" if proposals else "καμία αλλαγή ή δεν βρέθηκε κίνηση")
            self.findings.append(procedure.Finding(procedure.INFO, text))
            self.sweep = None
        return []

    def reading_findings(self) -> list:
        """One finding per channel that has an encoder: angle, magnet, and whose it is in the file."""
        owner = {}
        for joint, j in (self.data or {}).get("joints", {}).items():
            channel = self.data["encoders"].get(j.get("encoder"), {}).get("channel") if j.get("encoder") else None
            if channel is not None:
                owner[channel] = joint
        result = []
        for channel, r in sorted(self.readings.items()):
            if r.get("status") == "NO_RESPONSE":
                continue
            raw = "—" if r.get("raw_deg") is None else f"{r['raw_deg']:.1f}°"
            status = r.get("status", "?")
            note = MAGNET_NOTES.get(status, "")
            level = procedure.INFO if status == "OK" else (procedure.WARNING if status in procedure.USABLE
                                                           else procedure.ERROR)
            whose = owner.get(channel, "χωρίς άρθρωση")
            now = time.strftime("%H:%M:%S")
            text = f"{READ_PREFIX}{now} · κανάλι {channel} ({whose}): {raw} · μαγνήτης {status}{note}"
            result.append(procedure.Finding(level, text))
        if not result:
            result.append(procedure.Finding(procedure.ERROR, f"{READ_PREFIX}κανένας encoder δεν απαντά"))
        return result

    # ---------------------------------------------------------------- user answers
    def add(self, proposals: list) -> None:
        """New proposals replace older ones for the same parameter (not approved yet)."""
        names = {p.name for p in proposals}
        self.proposals = [row for row in self.proposals if row[0].name not in names]
        self.proposals += [[p, False] for p in proposals]

    def relabel(self, actual: str) -> None:
        """The last jog moved what the user calls `actual`."""
        if self.mapping is not None:
            self.add(procedure.relabel_proposals(self.data, self.mapping.joint, actual))

    def direction(self, positive: bool) -> None:
        """The user saw the last jog and says whether it was in the positive direction."""
        if self.mapping is not None and self.mapping.channel is not None:
            self.add(procedure.direction_proposals(self.data, self.mapping.joint, self.mapping.delta, positive))

    def _raw(self, joint: str):
        encoder = self.data["joints"][joint].get("encoder")
        channel = self.data["encoders"].get(encoder, {}).get("channel") if encoder else None
        reading = self.readings.get(channel) if channel is not None else None
        if not reading or reading.get("raw_deg") is None or reading.get("status") not in procedure.USABLE:
            return None
        return reading["raw_deg"]

    def zero(self, joint: str) -> bool:
        """Step 7 from the last read; False if the joint's encoder has no usable reading."""
        raw = self._raw(joint)
        if raw is not None:
            self.add(procedure.zero_proposals(self.data, {joint: raw}))
        return raw is not None

    def gripper(self, joint: str, closed: bool) -> bool:
        """Step 8a: remember the reading fully closed / fully open; both known -> proposals."""
        raw = self._raw(joint)
        if raw is None:
            return False
        self._gripper = {**getattr(self, "_gripper", {}), closed: raw}
        if len(self._gripper) == 2:
            proposals, findings = procedure.gripper_limit_proposals(
                self.data, joint, self._gripper[True], self._gripper[False])
            self.add(proposals)
            self.findings += findings
            self._gripper = {}
        return True

    def approved(self) -> tuple[dict, list]:
        """({parameter: value} of the approved proposals, [names that cannot be sent as parameters])."""
        values, manual = {}, []
        for proposal, ok in self.proposals:
            if not ok:
                continue
            if proposal.new is None:
                manual.append(proposal.name)     # removing an entry: only by editing the file
            else:
                values[proposal.name] = proposal.new
        return values, manual

    def applied(self, names: set) -> None:
        """These parameters were accepted by /arm_calibration: their proposals are done."""
        self.proposals = [row for row in self.proposals if row[0].name not in names]
