import time
from arm_link.ping import Ping
from arm_link.stats import LinkStats
from arm_link.transport import Transport

class LinkProbe:
    def __init__(self, transport: Transport):
        self.transp = transport
    def run(self, count: int, timeout_s: float) -> LinkStats:
        LinkStat = LinkStats()
        for i in range(count):
            now_ns = time.monotonic_ns()
            ping = Ping(i, now_ns)
            self.transp.send(ping.to_bytes())
            LinkStat.sent(i)
            data = self.transp.receive(timeout_s)
            if data is None:
                continue
            try:
                ping2 = Ping.from_bytes(data)
            except ValueError:
                continue
            now_ns2 = time.monotonic_ns()
            LinkStat.received(ping2, now_ns2)
        return LinkStat