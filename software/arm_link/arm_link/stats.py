"""Στατιστικα συνδεσης απο PING: rtt, jitter, packet loss, και latency"""
from arm_link.ping import Ping

class LinkStats:
    def __init__(self):
        self.pending: set = set()
        self.sent_count: int = 0
        self.rtts_ns: list = list()
    def sent(self, seq : int) -> None:
        self.pending.add(seq)
        self.sent_count = self.sent_count + 1
    def received(self, ping: Ping, now_ns: int) -> None:
        if ping.seq not in self.pending:
            return
        self.pending.remove(ping.seq)
        rtt = now_ns - ping.sent_ns
        self.rtts_ns.append(rtt)
    @property
    def loss_percent(self) -> float:
        if self.sent_count == 0:
            return 0.0
        received = len(self.rtts_ns)
        lost = self.sent_count - received
        return lost / self.sent_count * 100
    @property
    def avg_ms(self) -> float:
        if len(self.rtts_ns) == 0:
            return 0.0
        return (sum(self.rtts_ns)/len(self.rtts_ns))/10**6
    @property
    def min_ms(self) -> float:
        if len(self.rtts_ns) == 0:
            return 0.0
        return (min(self.rtts_ns))/10**6
    @property
    def max_ms(self) -> float:
        if len(self.rtts_ns) == 0:
            return 0.0
        return (max(self.rtts_ns))/10**6